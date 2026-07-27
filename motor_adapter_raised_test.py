#!/usr/bin/env python3
"""Physically validate straight and steering output with both wheels raised."""

from __future__ import print_function

import datetime
import json
import os
import signal
import sys
import time

from control_core import cruise_normalized_command, mix_forward
from motor_mapping import load_motor_config
from safe_motor_output import MotionDeadlineStopper, SafeMotorOutput
from validation_manager import (
    GATES_PATH,
    capture_artifact_snapshot,
    read_json,
    require_gate_prerequisites,
)


PULSE_SECONDS = 0.60
REFRESH_SECONDS = 0.10
STEERING_CORRECTION = 0.12
DEADLINE_STOP_MARGIN_SECONDS = 0.08


class TerminationRequested(Exception):
    pass


def termination_handler(signum, _frame):
    raise TerminationRequested("SIGNAL_{}".format(signum))


def utc_now():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def make_phase(name, left_command, right_command, expected, token):
    return {
        "name": name,
        "left_normalized_command": float(left_command),
        "right_normalized_command": float(right_command),
        "expected_visual_result": expected,
        "confirmation_token": token,
        "operator_confirmed": False,
        "status": "NOT_RUN",
        "samples": 0,
        "left_mapped_duty": None,
        "right_mapped_duty": None,
        "minimum_voltage_v": None,
        "actual_energized_duration_s": None,
        "deadline_stopper": None,
    }


def run_refreshed_pulse(output, phase, sleep_function=time.sleep):
    """Run one short pulse while refreshing the command lease."""
    started = time.monotonic()
    effective_deadline = started + max(
        0.05, PULSE_SECONDS - DEADLINE_STOP_MARGIN_SECONDS
    )
    total_before = float(output.total_energized_duration_s)
    deadline_stopper = MotionDeadlineStopper(
        output,
        effective_deadline,
        "{}_HARD_DEADLINE".format(phase["name"]),
    ).start()
    voltages = []
    result = None
    phase["status"] = "RUNNING"
    try:
        while True:
            remaining = effective_deadline - time.monotonic()
            if remaining <= 0.0:
                break
            try:
                result = output.command(
                    phase["left_normalized_command"],
                    phase["right_normalized_command"],
                    reason="RAISED_{}".format(phase["name"]),
                    deadline_monotonic=effective_deadline,
                )
            except RuntimeError:
                if (
                    time.monotonic() >= effective_deadline
                    and output.hardware_off_confirmed
                ):
                    break
                raise
            phase["samples"] += 1
            voltages.append(float(result["battery_voltage"]))
            print(
                "{} | PWM L {:.3f} R {:.3f} | Battery {:.3f} V".format(
                    phase["name"],
                    result["left_duty"],
                    result["right_duty"],
                    result["battery_voltage"],
                )
            )
            remaining = effective_deadline - time.monotonic()
            if remaining > 0.0:
                sleep_function(min(REFRESH_SECONDS, remaining))
    finally:
        try:
            output.stop("{}_PULSE_COMPLETE".format(phase["name"]))
        finally:
            deadline_stopper.cancel()

    phase["deadline_stopper"] = deadline_stopper.snapshot()
    if deadline_stopper.stop_error is not None:
        raise RuntimeError(
            "Independent deadline OFF failed: {}".format(
                deadline_stopper.stop_error
            )
        )
    phase["actual_energized_duration_s"] = max(
        0.0, float(output.total_energized_duration_s) - total_before
    )
    if phase["actual_energized_duration_s"] > PULSE_SECONDS:
        raise RuntimeError(
            "Measured energized duration {:.6f}s exceeded {:.6f}s limit."
            .format(phase["actual_energized_duration_s"], PULSE_SECONDS)
        )

    if result is not None:
        phase["left_mapped_duty"] = float(result["left_duty"])
        phase["right_mapped_duty"] = float(result["right_duty"])
    if voltages:
        phase["minimum_voltage_v"] = min(voltages)
    phase["status"] = "AWAITING_OPERATOR_CONFIRMATION"


def save_evidence(evidence):
    project_directory = os.path.dirname(os.path.abspath(__file__))
    evidence_directory = os.path.join(project_directory, "evidence")
    if not os.path.isdir(evidence_directory):
        os.makedirs(evidence_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    evidence_path = os.path.join(
        evidence_directory,
        "raised_motor_adapter_{}.json".format(stamp),
    )
    with open(evidence_path, "w") as output_file:
        json.dump(evidence, output_file, indent=2, sort_keys=True)
        output_file.write("\n")
    return evidence_path


def main(output_factory=SafeMotorOutput, input_function=input):
    for signal_name in ("SIGTERM", "SIGHUP", "SIGQUIT"):
        signal_value = getattr(signal, signal_name, None)
        if signal_value is not None:
            signal.signal(signal_value, termination_handler)

    try:
        gate_state = read_json(GATES_PATH)
        require_gate_prerequisites(gate_state, "raised_motor_adapter")
    except Exception as error:
        print("REFUSED: raised motor test prerequisites are not current: {}".format(
            error
        ))
        return 1

    motor_config = load_motor_config()
    base_command = cruise_normalized_command(motor_config)
    right_left, right_right = mix_forward(
        base_command, STEERING_CORRECTION
    )
    left_left, left_right = mix_forward(
        base_command, -STEERING_CORRECTION
    )
    phases = [
        make_phase(
            "STRAIGHT",
            base_command,
            base_command,
            "Both wheels turn forward smoothly, then both stop.",
            "STRAIGHT-PASS",
        ),
        make_phase(
            "RIGHT_CORRECTION",
            right_left,
            right_right,
            "Both turn forward; LEFT is faster than RIGHT, indicating a right correction.",
            "RIGHT-PASS",
        ),
        make_phase(
            "LEFT_CORRECTION",
            left_left,
            left_right,
            "Both turn forward; RIGHT is faster than LEFT, indicating a left correction.",
            "LEFT-PASS",
        ),
    ]

    started_utc = utc_now()
    output = None
    watchdog_software_passed = False
    watchdog_visual_passed = False
    watchdog_stop_reason = None
    watchdog_energized_duration = None
    failure = None
    interrupted = False

    print("=" * 68)
    print("RAISED STEERING AND COMMAND-LEASE VALIDATION")
    print("BOTH WHEELS MUST BE COMPLETELY OFF THE FLOOR.")
    print("MOTOR BATTERY MUST BE PHYSICALLY DISCONNECTED FOR INITIALISATION.")
    print("Do not touch or restrain a turning wheel.")
    print("A spotter must hold the motor-battery disconnect.")
    print("Positive steering must speed LEFT and slow RIGHT.")
    print("Negative steering must slow LEFT and speed RIGHT.")
    print("The final pulse intentionally stops through the command lease.")
    print("Ctrl+C closes every motor output.")
    print("=" * 68)
    if input_function(
        "Type RAISED-INITIALISE-OFF after disconnecting motor power: "
    ).strip() != "RAISED-INITIALISE-OFF":
        print("Cancelled. No motor command was sent.")
        return 0

    try:
        # The operator prompt may remain open for an arbitrary time. Recheck
        # every prerequisite and reload the mapping immediately before any
        # hardware object can be opened.
        gate_state = read_json(GATES_PATH)
        require_gate_prerequisites(gate_state, "raised_motor_adapter")
        motor_config = load_motor_config()
        base_command = cruise_normalized_command(motor_config)
        right_left, right_right = mix_forward(
            base_command, STEERING_CORRECTION
        )
        left_left, left_right = mix_forward(
            base_command, -STEERING_CORRECTION
        )
        command_pairs = (
            (base_command, base_command),
            (right_left, right_right),
            (left_left, left_right),
        )
        for phase, command_pair in zip(phases, command_pairs):
            phase["left_normalized_command"] = float(command_pair[0])
            phase["right_normalized_command"] = float(command_pair[1])
        artifact_snapshot_start = capture_artifact_snapshot(
            "raised_motor_adapter"
        )
    except Exception as error:
        print("REFUSED: could not bind the test to its artifacts: {}".format(
            error
        ))
        return 1

    try:
        output = output_factory()
        print("Controller initialised; hardware OFF write confirmed.")
        print("Keep both wheels raised. Spotter may now connect motor power.")
        if input_function(
            "Type BATTERY-CONNECTED-RAISED only after everyone is clear: "
        ).strip() != "BATTERY-CONNECTED-RAISED":
            raise RuntimeError("Motor-power connection was not confirmed.")
        gate_state = read_json(GATES_PATH)
        require_gate_prerequisites(gate_state, "raised_motor_adapter")
        for phase in phases:
            gate_state = read_json(GATES_PATH)
            require_gate_prerequisites(gate_state, "raised_motor_adapter")
            print("\n{}".format(phase["name"]))
            print("Expected: {}".format(phase["expected_visual_result"]))
            run_refreshed_pulse(output, phase)
            response = input_function(
                "Type {} only if that exact motion occurred: ".format(
                    phase["confirmation_token"]
                )
            ).strip()
            phase["operator_confirmed"] = (
                response == phase["confirmation_token"]
            )
            phase["status"] = (
                "PASS" if phase["operator_confirmed"] else "FAIL"
            )
            if not phase["operator_confirmed"]:
                raise RuntimeError(
                    "Visual confirmation failed for {}. Further steering "
                    "tests are refused.".format(phase["name"])
                )
            time.sleep(0.25)

        print("\nCOMMAND-LEASE EXPIRY")
        gate_state = read_json(GATES_PATH)
        require_gate_prerequisites(gate_state, "raised_motor_adapter")
        print(
            "Both wheels will start straight. No refresh will be sent; "
            "they must stop automatically."
        )
        watchdog_total_before = float(output.total_energized_duration_s)
        watchdog_result = output.command(
            base_command,
            base_command,
            reason="RAISED_COMMAND_LEASE_TEST",
        )
        print(
            "WATCHDOG START | PWM L {:.3f} R {:.3f} | Battery {:.3f} V".format(
                watchdog_result["left_duty"],
                watchdog_result["right_duty"],
                watchdog_result["battery_voltage"],
            )
        )
        time.sleep(output.lease_seconds + 0.25)
        watchdog_energized_duration = max(
            0.0,
            float(output.total_energized_duration_s) - watchdog_total_before,
        )
        watchdog_software_passed = (
            output.last_stop_reason == "COMMAND_LEASE_EXPIRED"
            and output.last_duties == (0.0, 0.0)
            and output.watchdog_fault == "COMMAND_LEASE_EXPIRED"
            and output.watchdog_trip_count == 1
            and output.hardware_off_confirmed
            and not output.hardware_off_retry_required
        )
        watchdog_stop_reason = output.last_stop_reason
        print(
            "Software command-lease result: {} ({})".format(
                "PASS" if watchdog_software_passed else "FAIL",
                output.last_stop_reason,
            )
        )
        if not watchdog_software_passed:
            print(
                "LEASE FAILURE: SPOTTER DISCONNECT THE MOTOR BATTERY NOW."
            )
            try:
                output.stop("COMMAND_LEASE_VALIDATION_FAILED")
            finally:
                raise RuntimeError(
                    "The command-lease software OFF state was not verified; "
                    "visual confirmation was not requested."
                )
        response = input_function(
            "Type WATCHDOG-PASS only if BOTH wheels visibly stopped "
            "without another command: "
        ).strip()
        watchdog_visual_passed = response == "WATCHDOG-PASS"
        if not watchdog_software_passed or not watchdog_visual_passed:
            raise RuntimeError(
                "The command-lease stop was not verified in software and visually."
            )

    except KeyboardInterrupt:
        interrupted = True
        failure = "EMERGENCY_STOP_FROM_KEYBOARD"
        print("\nEMERGENCY STOP")
    except TerminationRequested as error:
        interrupted = True
        failure = str(error)
        print("\nEMERGENCY STOP: {}".format(error))
    except Exception as error:
        failure = str(error)
        print("\nTEST STOPPED: {}".format(error))
    finally:
        if output is not None:
            try:
                output.close()
            except Exception as close_error:
                if failure is None:
                    failure = "MOTOR_CLOSE_ERROR: {}".format(close_error)
                print("MOTOR CLOSE ERROR: {}".format(close_error))

    try:
        artifact_snapshot_end = capture_artifact_snapshot(
            "raised_motor_adapter"
        )
    except Exception as error:
        artifact_snapshot_end = {}
        if failure is None:
            failure = "ARTIFACT_SNAPSHOT_ERROR: {}".format(error)
    artifact_snapshot_stable = (
        artifact_snapshot_start == artifact_snapshot_end
    )
    if not artifact_snapshot_stable and failure is None:
        failure = "TEST_ARTIFACTS_CHANGED_DURING_RUN"

    passed = bool(
        failure is None
        and all(phase["operator_confirmed"] for phase in phases)
        and watchdog_software_passed
        and watchdog_visual_passed
    )
    minimum_voltage = None
    if output is not None and output.minimum_voltage != float("inf"):
        minimum_voltage = float(output.minimum_voltage)
    evidence = {
        "schema_version": 2,
        "test": "RAISED_STEERING_AND_COMMAND_LEASE_VALIDATION",
        "passed": passed,
        "classification": "PHYSICALLY_VERIFIED" if passed else "UNVERIFIED",
        "physical_motor_commands": True,
        "wheels_raised": True,
        "base_normalized_command": float(base_command),
        "steering_correction": float(STEERING_CORRECTION),
        "pulse_seconds": float(PULSE_SECONDS),
        "maximum_actual_energized_duration_s": max(
            [
                float(phase["actual_energized_duration_s"])
                for phase in phases
                if phase["actual_energized_duration_s"] is not None
            ]
            or [0.0]
        ),
        "phases": phases,
        "command_lease_seconds": (
            None if output is None else float(output.lease_seconds)
        ),
        "command_lease_software_passed": watchdog_software_passed,
        "command_lease_visual_confirmation": watchdog_visual_passed,
        "command_lease_stop_reason": watchdog_stop_reason,
        "command_lease_actual_energized_duration_s": (
            watchdog_energized_duration
        ),
        "hardware_off_confirmed": bool(
            output is not None and output.hardware_off_confirmed
        ),
        "hardware_off_retry_required": bool(
            output is not None and output.hardware_off_retry_required
        ),
        "watchdog_trip_count": (
            None if output is None else int(output.watchdog_trip_count)
        ),
        "minimum_voltage_v": minimum_voltage,
        "failure": failure,
        "interrupted": interrupted,
        "artifact_snapshot_stable": artifact_snapshot_stable,
        "artifact_sha256_at_test": artifact_snapshot_end,
        "started_utc": started_utc,
        "completed_utc": utc_now(),
    }
    evidence_path = save_evidence(evidence)
    print("\n{}".format(json.dumps(evidence, indent=2, sort_keys=True)))
    print("Evidence: {}".format(evidence_path))
    print("RESULT: {}".format("PASS" if passed else "NOT VERIFIED"))
    if interrupted:
        return 130
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
