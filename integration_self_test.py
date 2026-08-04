#!/usr/bin/env python3
"""Hardware-free regression tests for mapping, PID mixing and watchdog stop."""

from __future__ import print_function

import copy
import math
import time

from control_core import (
    PIDController,
    SafetySupervisor,
    cruise_normalized_command,
    load_controller_config,
    mix_forward,
    validate_controller_config,
)
from motor_mapping import load_motor_config, map_duty
from safe_motor_output import MotionDeadlineStopper, SafeMotorOutput


class FakeMotorController(object):
    instances = []

    def __init__(self, reverse_left=False, reverse_right=False):
        self.reverse_left = reverse_left
        self.reverse_right = reverse_right
        self.channels = {}
        self.off_count = 0
        self.closed = False
        FakeMotorController.instances.append(self)

    def all_off(self):
        self.channels = {}
        self.off_count += 1

    def set_channel_duty(self, channel, duty):
        self.channels[int(channel)] = float(duty)

    def configure_motor_direction(self, command, input_a, input_b, reversed_motor):
        command = -float(command) if reversed_motor else float(command)
        self.channels[int(input_a)] = 1.0 if command > 0.0 else 0.0
        self.channels[int(input_b)] = 1.0 if command < 0.0 else 0.0

    def read_battery_voltage(self):
        return 7.80

    def close(self):
        self.closed = True


class SaggingFakeMotorController(FakeMotorController):
    def __init__(self, reverse_left=False, reverse_right=False):
        super(SaggingFakeMotorController, self).__init__(
            reverse_left=reverse_left,
            reverse_right=reverse_right,
        )
        self.voltages = [7.80, 6.50]

    def read_battery_voltage(self):
        if len(self.voltages) > 1:
            return self.voltages.pop(0)
        return self.voltages[0]


class RefreshReadFailureController(FakeMotorController):
    """Fail the next pre-command read after one command energized PWM."""

    def __init__(self, reverse_left=False, reverse_right=False):
        super(RefreshReadFailureController, self).__init__(
            reverse_left=reverse_left,
            reverse_right=reverse_right,
        )
        self.read_count = 0

    def read_battery_voltage(self):
        self.read_count += 1
        if self.read_count >= 3:
            raise IOError("EXPECTED_REFRESH_VOLTAGE_READ_FAILURE")
        return 7.80


class DirectionFailureController(FakeMotorController):
    """Fail the first direction write of a later reversal transaction."""

    def __init__(self, reverse_left=False, reverse_right=False):
        super(DirectionFailureController, self).__init__(
            reverse_left=reverse_left,
            reverse_right=reverse_right,
        )
        self.direction_count = 0

    def configure_motor_direction(self, command, input_a, input_b, reversed_motor):
        self.direction_count += 1
        if self.direction_count >= 3:
            raise IOError("EXPECTED_DIRECTION_WRITE_FAILURE")
        return super(DirectionFailureController, self).configure_motor_direction(
            command, input_a, input_b, reversed_motor
        )


class PwmInterruptController(FakeMotorController):
    def set_channel_duty(self, channel, duty):
        if int(channel) == 5 and float(duty) > 0.0:
            raise KeyboardInterrupt("EXPECTED_PWM_KEYBOARD_INTERRUPT")
        return super(PwmInterruptController, self).set_channel_duty(
            channel, duty
        )


class SlowRightPwmController(FakeMotorController):
    def set_channel_duty(self, channel, duty):
        if int(channel) == 5 and float(duty) > 0.0:
            time.sleep(0.45)
        return super(SlowRightPwmController, self).set_channel_duty(
            channel, duty
        )


class SlowLeftPastDeadlineController(FakeMotorController):
    """Hold the first left EN write beyond a caller observation deadline."""

    def __init__(self, reverse_left=False, reverse_right=False):
        super(SlowLeftPastDeadlineController, self).__init__(
            reverse_left=reverse_left,
            reverse_right=reverse_right,
        )
        self.pwm_writes = []

    def set_channel_duty(self, channel, duty):
        channel = int(channel)
        duty = float(duty)
        self.pwm_writes.append((channel, duty, time.monotonic()))
        if channel == 0 and duty > 0.0:
            time.sleep(0.12)
        return super(SlowLeftPastDeadlineController, self).set_channel_duty(
            channel, duty
        )


class OneFailedOffController(FakeMotorController):
    def __init__(self, reverse_left=False, reverse_right=False):
        self.off_attempts = 0
        super(OneFailedOffController, self).__init__(
            reverse_left=reverse_left,
            reverse_right=reverse_right,
        )

    def all_off(self):
        self.off_attempts += 1
        if self.off_attempts == 2:
            raise IOError("EXPECTED_TRANSIENT_ALL_OFF_FAILURE")
        return super(OneFailedOffController, self).all_off()


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    motor_config = load_motor_config()
    controller_config = load_controller_config()
    base = cruise_normalized_command(motor_config)
    require(abs(base - 0.3947368421) < 0.00001, "Cruise inverse mapping changed.")
    require(abs(map_duty(base, False, motor_config) - 0.75) < 0.00001, "Left cruise mapping failed.")
    require(abs(map_duty(base, True, motor_config) - 0.735) < 0.00001, "Right trim mapping failed.")
    require(
        map_duty(0.021, False, motor_config)
        >= motor_config["left_deadband_duty"],
        "Left mapping fell below its physical floor.",
    )
    require(
        map_duty(0.021, True, motor_config)
        >= motor_config["right_deadband_duty"],
        "Right mapping fell below its physical floor.",
    )
    require(
        abs(map_duty(1.0, False, motor_config) - 0.98) < 0.00001,
        "Left maximum mapping failed.",
    )
    require(
        abs(map_duty(1.0, True, motor_config) - 0.9604) < 0.00001,
        "Right maximum mapping failed.",
    )

    pid = PIDController(controller_config)
    terms = pid.update(0.20, timestamp=1.0)
    left, right = mix_forward(base, terms["output"])
    require(left > right, "Positive lane offset must command a right correction.")
    negative_pid = PIDController(controller_config)
    negative_terms = negative_pid.update(-0.20, timestamp=1.0)
    negative_left, negative_right = mix_forward(
        base, negative_terms["output"]
    )
    require(
        negative_right > negative_left,
        "Negative lane offset must command a left correction.",
    )
    require(mix_forward(0.0, 0.1) == (0.0, 0.0), "STOP mapping failed.")

    invalid_config = copy.deepcopy(controller_config)
    invalid_config["allow_partial_lane_motion"] = "false"
    try:
        validate_controller_config(invalid_config)
        raise AssertionError("String boolean was accepted in controller config.")
    except ValueError:
        pass

    safety = SafetySupervisor(controller_config)
    stopped = safety.evaluate(
        "FULL",
        0.9,
        0.01,
        {"has_result": False, "error": None},
        {"state": "DRIVE", "reason": "CLEAR"},
        now=1.0,
    )
    require(stopped["state"] == "STOP", "YOLO startup must fail safe.")
    partial = safety.evaluate(
        "PARTIAL_LEFT",
        0.9,
        0.01,
        {
            "has_result": True,
            "error": None,
            "age_seconds": 0.01,
            "source_frame_lag": 0,
        },
        {"state": "DRIVE", "reason": "PATH_CLEAR"},
        now=1.1,
    )
    require(
        partial["reason"] == "PARTIAL_LANE_MOTION_NOT_VALIDATED",
        "Partial lane must remain physically disabled.",
    )
    current_yolo = {
        "has_result": True,
        "error": None,
        "age_seconds": 0.01,
        "source_frame_lag": 0,
    }
    clear_path = {"state": "DRIVE", "reason": "PATH_CLEAR"}
    valid_line = safety.evaluate(
        "FULL", 0.90, 0.01, current_yolo, clear_path, now=1.2
    )
    require(
        valid_line == {
            "state": "DRIVE",
            "reason": "ALL_GATES_VALID",
            "speed_scale": 1.0,
        },
        "A current, unambiguous single line must reach DRIVE.",
    )
    for rejected_status in (
        "LOST",
        "AMBIGUOUS",
        "INVALID_GEOMETRY",
        "RUN_OVERFLOW",
    ):
        rejected = safety.evaluate(
            rejected_status,
            0.90,
            0.01,
            current_yolo,
            clear_path,
            now=1.3,
        )
        require(
            rejected["state"] == "STOP"
            and rejected["reason"] == "LANE_{}".format(rejected_status),
            "{} did not fail safe.".format(rejected_status),
        )
    low_confidence = safety.evaluate(
        "FULL", 0.549, 0.01, current_yolo, clear_path, now=1.4
    )
    require(
        low_confidence["reason"] == "LANE_CONFIDENCE_LOW",
        "Sub-threshold single-line confidence was accepted.",
    )
    invalid_confidence = safety.evaluate(
        "FULL", float("nan"), 0.01, current_yolo, clear_path, now=1.5
    )
    require(
        invalid_confidence["reason"] == "LANE_CONFIDENCE_INVALID",
        "Non-finite single-line confidence was accepted.",
    )

    output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=FakeMotorController,
    )
    result = output.command(base, base)
    require(result["left_duty"] > 0.0, "Fake motor command was not applied.")
    require(
        result["battery_pre_command_v"] == 7.80
        and result["battery_post_command_v"] == 7.80,
        "Pre/post-command voltage evidence was not captured.",
    )
    time.sleep(float(controller_config["command_lease_s"]) + 0.15)
    require(output.last_stop_reason == "COMMAND_LEASE_EXPIRED", "Watchdog did not stop.")
    require(
        output.watchdog_fault == "COMMAND_LEASE_EXPIRED"
        and output.watchdog_trip_count == 1,
        "Command-lease expiry was not latched and counted.",
    )
    require(output.controller.off_count >= 2, "Watchdog did not call all_off.")
    try:
        output.command(base, base)
        raise AssertionError("Motor command resumed after command-lease expiry.")
    except RuntimeError:
        pass
    output.close()
    require(output.controller.closed, "Controller did not close.")

    # Deterministically suppress the background poll, then prove command()
    # itself refuses a refresh that arrives after the lease deadline.
    late_refresh_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=FakeMotorController,
    )
    late_refresh_output.stop_event.set()
    late_refresh_output.watchdog.join(timeout=1.0)
    late_refresh_output.command(base, base)
    with late_refresh_output.lock:
        late_refresh_output.last_command_time -= (
            late_refresh_output.lease_seconds + 0.01
        )
    try:
        late_refresh_output.command(base, base)
        raise AssertionError("A refresh after lease expiry was accepted.")
    except RuntimeError:
        pass
    require(
        late_refresh_output.watchdog_fault == "COMMAND_LEASE_EXPIRED"
        and late_refresh_output.watchdog_trip_count == 1
        and late_refresh_output.last_duties == (0.0, 0.0),
        "Late refresh did not synchronously latch and stop lease expiry.",
    )
    late_refresh_output.close()

    read_failure_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=RefreshReadFailureController,
    )
    read_failure_output.command(base, base)
    try:
        read_failure_output.command(base, base)
        raise AssertionError("A failed refresh voltage read was accepted.")
    except IOError as error:
        require(
            "EXPECTED_REFRESH_VOLTAGE_READ_FAILURE" in str(error),
            "The original voltage-read exception was not preserved.",
        )
    require(
        read_failure_output.last_duties == (0.0, 0.0)
        and read_failure_output.last_command_time is None
        and read_failure_output.controller.off_count >= 2,
        "A refresh-read exception did not force all outputs off.",
    )
    read_failure_output.close()

    direction_failure_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=DirectionFailureController,
        allow_reverse=True,
    )
    direction_failure_output.command(base, base)
    try:
        direction_failure_output.command(-base, -base)
        raise AssertionError("A failed reversal direction write was accepted.")
    except IOError as error:
        require(
            "EXPECTED_DIRECTION_WRITE_FAILURE" in str(error),
            "The original direction-write exception was not preserved.",
        )
    require(
        direction_failure_output.last_duties == (0.0, 0.0)
        and direction_failure_output.last_command_time is None
        and direction_failure_output.controller.off_count >= 2,
        "A direction-write exception did not force all outputs off.",
    )
    direction_failure_output.close()

    interrupt_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=PwmInterruptController,
    )
    try:
        interrupt_output.command(base, base)
        raise AssertionError("KeyboardInterrupt during PWM was swallowed.")
    except KeyboardInterrupt as error:
        require(
            "EXPECTED_PWM_KEYBOARD_INTERRUPT" in str(error),
            "The original KeyboardInterrupt was not preserved.",
        )
    require(
        interrupt_output.hardware_off_confirmed
        and interrupt_output.last_duties == (0.0, 0.0)
        and interrupt_output.last_command_time is None,
        "KeyboardInterrupt during PWM did not force hardware OFF.",
    )
    interrupt_output.close()

    blocked_write_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=SlowRightPwmController,
    )
    try:
        blocked_write_output.command(base, base)
        raise AssertionError("An EN write beyond the lease was accepted.")
    except RuntimeError:
        pass
    require(
        blocked_write_output.watchdog_fault is not None
        and blocked_write_output.hardware_off_confirmed
        and blocked_write_output.last_duties == (0.0, 0.0),
        "In-progress blocked EN write did not latch and stop.",
    )
    blocked_write_output.close()

    # If the first (left) EN transaction itself crosses an observation
    # deadline, command() must stop without issuing a later non-zero write to
    # the right EN channel.
    expired_between_writes_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=SlowLeftPastDeadlineController,
    )
    between_write_deadline = time.monotonic() + 0.08
    try:
        expired_between_writes_output.command(
            base,
            base,
            deadline_monotonic=between_write_deadline,
            deadline_reason="TEST_OBSERVATION_DEADLINE_EXPIRED",
        )
        raise AssertionError(
            "A right EN write was allowed after the observation deadline."
        )
    except RuntimeError:
        pass
    right_nonzero_writes = [
        item
        for item in expired_between_writes_output.controller.pwm_writes
        if item[0] == expired_between_writes_output.RIGHT_ENB
        and item[1] > 0.0
    ]
    require(
        not right_nonzero_writes,
        "A non-zero right EN write occurred after the left write crossed "
        "the observation deadline.",
    )
    require(
        expired_between_writes_output.hardware_off_confirmed
        and expired_between_writes_output.last_duties == (0.0, 0.0),
        "Between-write deadline expiry did not force hardware OFF.",
    )
    expired_between_writes_output.close()

    retry_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=OneFailedOffController,
    )
    retry_output.command(base, base)
    try:
        retry_output.stop("TEST_TRANSIENT_OFF_FAILURE")
        raise AssertionError("Injected all_off failure did not propagate.")
    except IOError:
        pass
    require(
        retry_output.hardware_off_retry_required
        and not retry_output.hardware_off_confirmed,
        "Failed all_off did not leave an explicit retry-required state.",
    )
    time.sleep(0.20)
    require(
        retry_output.hardware_off_confirmed
        and not retry_output.hardware_off_retry_required
        and retry_output.watchdog_fault is not None,
        "Watchdog did not retry a transient all_off failure.",
    )
    try:
        retry_output.command(base, base)
        raise AssertionError("Command resumed after an all_off fault.")
    except RuntimeError:
        pass
    retry_output.close()

    deadline_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=FakeMotorController,
    )
    hard_deadline = time.monotonic() + 0.18
    deadline_stopper = MotionDeadlineStopper(
        deadline_output,
        hard_deadline,
        "SELF_TEST_HARD_DEADLINE",
    ).start()
    deadline_output.command(
        base,
        base,
        deadline_monotonic=hard_deadline,
    )
    time.sleep(0.25)
    deadline_stopper.cancel()
    require(
        deadline_stopper.triggered
        and deadline_stopper.stop_error is None
        and deadline_output.hardware_off_confirmed
        and deadline_output.last_duties == (0.0, 0.0),
        "Independent hard-deadline stopper did not confirm OFF.",
    )
    require(
        0.0 < deadline_output.last_energized_duration_s <= 0.23,
        "Measured hard-deadline energized duration is invalid.",
    )
    try:
        deadline_output.command(
            base,
            base,
            deadline_monotonic=hard_deadline,
        )
        raise AssertionError("A command restarted after its hard deadline.")
    except RuntimeError:
        pass
    deadline_output.close()

    # Prove a caller's shorter perception deadline is enforced by the motor
    # watchdog even if the control loop never returns to refresh or stop.
    perception_deadline_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=FakeMotorController,
    )
    perception_deadline = time.monotonic() + 0.16
    deadline_result = perception_deadline_output.command(
        base,
        base,
        deadline_monotonic=perception_deadline,
        deadline_reason="PERCEPTION_DEADLINE_EXPIRED",
    )
    require(
        deadline_result["command_expiry_reason"]
        == "PERCEPTION_DEADLINE_EXPIRED",
        "Short perception deadline was not bound to the command.",
    )
    time.sleep(0.25)
    require(
        perception_deadline_output.watchdog_fault
        == "PERCEPTION_DEADLINE_EXPIRED"
        and perception_deadline_output.watchdog_trip_count == 1
        and perception_deadline_output.hardware_off_confirmed
        and perception_deadline_output.last_duties == (0.0, 0.0),
        "Perception deadline did not independently force hardware OFF.",
    )
    perception_deadline_output.close()

    sagging_output = SafeMotorOutput(
        motor_config=motor_config,
        controller_config=controller_config,
        controller_factory=SaggingFakeMotorController,
    )
    try:
        sagging_output.command(base, base)
        raise AssertionError("Post-command undervoltage was not rejected.")
    except RuntimeError:
        require(
            sagging_output.last_stop_reason == "BATTERY_UNDERVOLTAGE",
            "Undervoltage did not force an all-off state.",
        )
    finally:
        sagging_output.close()

    print("INTEGRATION SELF-TEST: PASS")
    print("Cruise normalized command: {:.6f}".format(base))
    print("Mapped cruise duties: left {:.3f}, right {:.3f}".format(
        map_duty(base, False, motor_config),
        map_duty(base, True, motor_config),
    ))
    print("Command-lease watchdog: PASS")
    print("Command-lease expiry latch: PASS")
    print("Late-refresh lease interlock: PASS")
    print("Whole-transaction fail-safe OFF: PASS")
    print("Interrupt/in-progress I2C fail-safe OFF: PASS")
    print("Between-EN-write deadline interlock: PASS")
    print("Failed-OFF retry and restart latch: PASS")
    print("Independent hard motion deadline and duration meter: PASS")
    print("Observation-bound command expiry watchdog: PASS")
    print("Pre/post-command voltage guard: PASS")
    print("Strict configuration validation: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
