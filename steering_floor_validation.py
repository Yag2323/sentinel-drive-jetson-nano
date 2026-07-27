#!/usr/bin/env python3
"""Manual short-pulse floor validation for straight, right and left motion."""

from __future__ import print_function

import argparse
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


REFRESH_SECONDS = 0.10
DEFAULT_PULSE_SECONDS = 0.35
DEFAULT_STEERING_CORRECTION = 0.12
DEADLINE_STOP_MARGIN_SECONDS = 0.08


class TerminationRequested(Exception):
    pass


def termination_handler(signum, _frame):
    raise TerminationRequested("SIGNAL_{}".format(signum))


def utc_now():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def make_phase(
    name,
    left_command,
    right_command,
    ready_token,
    pass_token,
    expected,
):
    return {
        "name": name,
        "left_normalized_command": float(left_command),
        "right_normalized_command": float(right_command),
        "ready_token": ready_token,
        "confirmation_token": pass_token,
        "expected_visual_result": expected,
        "operator_confirmed": False,
        "status": "NOT_RUN",
        "samples": 0,
        "left_mapped_duty": None,
        "right_mapped_duty": None,
        "minimum_voltage_v": None,
        "actual_energized_duration_s": None,
        "deadline_stopper": None,
    }


def run_pulse(output, phase, pulse_seconds, sleep_function=time.sleep):
    """Apply one forward-only command while continuously refreshing its lease."""
    started = time.monotonic()
    effective_deadline = started + max(
        0.05, pulse_seconds - DEADLINE_STOP_MARGIN_SECONDS
    )
    total_before = float(output.total_energized_duration_s)
    deadline_stopper = MotionDeadlineStopper(
        output,
        effective_deadline,
        "{}_HARD_DEADLINE".format(phase["name"]),
    ).start()
    result = None
    voltages = []
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
                    reason="FLOOR_{}".format(phase["name"]),
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
    if phase["actual_energized_duration_s"] > pulse_seconds:
        raise RuntimeError(
            "Measured energized duration {:.6f}s exceeded {:.6f}s limit."
            .format(phase["actual_energized_duration_s"], pulse_seconds)
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
        "steering_floor_validation_{}.json".format(stamp),
    )
    with open(evidence_path, "w") as output_file:
        json.dump(evidence, output_file, indent=2, sort_keys=True)
        output_file.write("\n")
    return evidence_path


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Run separate, manually armed straight/right/left floor pulses."
        )
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=DEFAULT_PULSE_SECONDS,
        help="Duration of each pulse (0.20 to 0.60 seconds).",
    )
    parser.add_argument(
        "--steering",
        type=float,
        default=DEFAULT_STEERING_CORRECTION,
        help="Normalized steering correction (0.05 to 0.18).",
    )
    return parser.parse_args()


def main(arguments=None, output_factory=SafeMotorOutput, input_function=input):
    for signal_name in ("SIGTERM", "SIGHUP", "SIGQUIT"):
        signal_value = getattr(signal, signal_name, None)
        if signal_value is not None:
            signal.signal(signal_value, termination_handler)

    if arguments is None:
        arguments = parse_arguments()
    pulse_seconds = float(arguments.seconds)
    steering_correction = float(arguments.steering)
    if not 0.20 <= pulse_seconds <= 0.60:
        print("REFUSED: --seconds must be from 0.20 to 0.60.")
        return 1
    if not 0.05 <= steering_correction <= 0.18:
        print("REFUSED: --steering must be from 0.05 to 0.18.")
        return 1

    try:
        gate_state = read_json(GATES_PATH)
        require_gate_prerequisites(gate_state, "short_floor_steering")
    except Exception as error:
        print("REFUSED: floor steering prerequisites are not current: {}".format(
            error
        ))
        return 1

    motor_config = load_motor_config()
    base_command = cruise_normalized_command(motor_config)
    right_left, right_right = mix_forward(
        base_command, steering_correction
    )
    left_left, left_right = mix_forward(
        base_command, -steering_correction
    )
    phases = [
        make_phase(
            "STRAIGHT",
            base_command,
            base_command,
            "READY-STRAIGHT",
            "STRAIGHT-PASS",
            "Robot moves forward approximately straight and stops automatically.",
        ),
        make_phase(
            "RIGHT_CORRECTION",
            right_left,
            right_right,
            "READY-RIGHT",
            "RIGHT-PASS",
            "Robot makes a short right-hand curve and stops automatically.",
        ),
        make_phase(
            "LEFT_CORRECTION",
            left_left,
            left_right,
            "READY-LEFT",
            "LEFT-PASS",
            "Robot makes a short left-hand curve and stops automatically.",
        ),
    ]

    print("=" * 70)
    print("MANUAL FLOOR STEERING VALIDATION - NO AUTONOMOUS INPUT")
    print("Each motion is a separate {:.2f}-second pulse.".format(pulse_seconds))
    print("Use a flat, open lane longer than 3 metres.")
    print("No people, traffic, stairs, drop-offs, edges or obstacles.")
    print("Point the robot away from people and property.")
    print("A spotter must hold the motor-battery disconnect throughout.")
    print("MOTOR BATTERY MUST BE PHYSICALLY DISCONNECTED FOR INITIALISATION.")
    print("Disconnect motor power before every repositioning step.")
    print("Ctrl+C and every error close all motor outputs.")
    print("=" * 70)
    if input_function(
        "Type FLOOR-INITIALISE-OFF after disconnecting motor power: "
    ).strip() != "FLOOR-INITIALISE-OFF":
        print("Cancelled. No motor command was sent.")
        return 0

    try:
        # Recheck after the unbounded operator prompt, then rebuild the exact
        # commands from the current gate-bound configuration.
        gate_state = read_json(GATES_PATH)
        require_gate_prerequisites(gate_state, "short_floor_steering")
        motor_config = load_motor_config()
        base_command = cruise_normalized_command(motor_config)
        right_left, right_right = mix_forward(
            base_command, steering_correction
        )
        left_left, left_right = mix_forward(
            base_command, -steering_correction
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
            "short_floor_steering"
        )
    except Exception as error:
        print("REFUSED: could not bind the test to its artifacts: {}".format(
            error
        ))
        return 1

    output = None
    failure = None
    interrupted = False
    operator_note = ""
    started_utc = utc_now()
    try:
        output = output_factory()
        print("Controller initialised; hardware OFF write confirmed.")
        for index, phase in enumerate(phases):
            print("\nPHASE {}/3: {}".format(index + 1, phase["name"]))
            print("1. Verify that all motor outputs are OFF.")
            print("2. Spotter disconnects the motor battery.")
            print("3. Reposition the robot on the marked start point.")
            print("4. Reconnect only after everyone is clear.")
            print("Expected: {}".format(phase["expected_visual_result"]))
            ready = input_function(
                "Type {} to arm this one pulse: ".format(
                    phase["ready_token"]
                )
            ).strip()
            if ready != phase["ready_token"]:
                phase["status"] = "REFUSED_NOT_ARMED"
                raise RuntimeError(
                    "{} was not explicitly armed.".format(phase["name"])
                )

            gate_state = read_json(GATES_PATH)
            require_gate_prerequisites(gate_state, "short_floor_steering")
            run_pulse(output, phase, pulse_seconds)
            print("Motor outputs are OFF.")
            confirmation = input_function(
                "Type {} only if the expected motion and stop occurred: ".format(
                    phase["confirmation_token"]
                )
            ).strip()
            phase["operator_confirmed"] = (
                confirmation == phase["confirmation_token"]
            )
            phase["status"] = (
                "PASS" if phase["operator_confirmed"] else "FAIL"
            )
            if not phase["operator_confirmed"]:
                raise RuntimeError(
                    "{} was not visually verified. Remaining floor pulses "
                    "are refused.".format(phase["name"])
                )

        operator_note = input_function(
            "Optional observation/measurement note (press Enter to skip): "
        ).strip()
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
            "short_floor_steering"
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
        and output is not None
        and output.hardware_off_confirmed
        and not output.hardware_off_retry_required
        and output.watchdog_fault is None
        and output.watchdog_trip_count == 0
    )
    minimum_voltage = None
    if output is not None and output.minimum_voltage != float("inf"):
        minimum_voltage = float(output.minimum_voltage)
    evidence = {
        "schema_version": 1,
        "test": "SHORT_FLOOR_STEERING_VALIDATION",
        "passed": passed,
        "classification": "PHYSICALLY_VERIFIED" if passed else "UNVERIFIED",
        "physical_motor_commands": True,
        "wheels_on_floor": True,
        "autonomous_input_used": False,
        "operator_confirmed_clear_lane_and_spotter": True,
        "pulse_seconds": pulse_seconds,
        "maximum_pulse_s": pulse_seconds,
        "maximum_actual_energized_duration_s": max(
            [
                float(phase["actual_energized_duration_s"])
                for phase in phases
                if phase["actual_energized_duration_s"] is not None
            ]
            or [0.0]
        ),
        "straight_motion_passed": bool(
            phases[0]["operator_confirmed"]
        ),
        "right_correction_passed": bool(
            phases[1]["operator_confirmed"]
        ),
        "left_correction_passed": bool(
            phases[2]["operator_confirmed"]
        ),
        "controlled_stop_confirmed": bool(
            all(phase["operator_confirmed"] for phase in phases)
        ),
        "base_normalized_command": float(base_command),
        "steering_correction": steering_correction,
        "phases": phases,
        "minimum_voltage_v": minimum_voltage,
        "hardware_off_confirmed": bool(
            output is not None and output.hardware_off_confirmed
        ),
        "hardware_off_retry_required": bool(
            output is not None and output.hardware_off_retry_required
        ),
        "watchdog_fault": (
            None if output is None else output.watchdog_fault
        ),
        "watchdog_trip_count": (
            None if output is None else int(output.watchdog_trip_count)
        ),
        "operator_note": operator_note,
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
