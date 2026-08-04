#!/usr/bin/env python3
"""Continuous, fail-safe PCA9685/L298N output adapter.

Unlike the original pulse helper, this adapter does not insert 160 ms of
dead-time on every control frame.  Direction pins are changed only while PWM
is disabled, and a command-lease watchdog turns every output off if the
perception loop stops refreshing commands.
"""

from __future__ import print_function

import math
import threading
import time

from control_core import load_controller_config
from motor_mapping import load_motor_config, map_signed_duty


class MotionDeadlineStopper(object):
    """Best-effort independent OFF request at an absolute monotonic deadline.

    The command path must also receive the same deadline.  That closes the
    race where this thread stops an old command immediately before the main
    loop attempts to publish a new one.  A physical battery disconnect is
    still required because no Python thread can recover from a wedged I2C
    transaction, an OS failure, or loss of process scheduling.
    """

    def __init__(self, output, deadline_monotonic, reason):
        self.output = output
        self.deadline_monotonic = float(deadline_monotonic)
        if not math.isfinite(self.deadline_monotonic):
            raise ValueError("Motion deadline must be finite.")
        self.reason = str(reason)
        self.cancel_event = threading.Event()
        self.triggered = False
        self.triggered_monotonic = None
        self.stop_confirmed_monotonic = None
        self.stop_error = None
        self.thread = threading.Thread(
            target=self._run,
            name="IndependentMotionDeadlineStopper",
        )
        self.thread.daemon = True

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        remaining = max(0.0, self.deadline_monotonic - time.monotonic())
        if self.cancel_event.wait(remaining):
            return
        self.triggered = True
        self.triggered_monotonic = time.monotonic()
        try:
            self.output.stop(self.reason)
            if not self.output.hardware_off_confirmed:
                raise RuntimeError("Hardware OFF was not confirmed.")
            self.stop_confirmed_monotonic = time.monotonic()
        except BaseException as error:
            self.stop_error = repr(error)

    def cancel(self):
        self.cancel_event.set()
        self.thread.join(timeout=1.0)
        if self.thread.is_alive() and self.stop_error is None:
            self.stop_error = "DEADLINE_STOP_THREAD_DID_NOT_FINISH"

    def snapshot(self):
        return {
            "deadline_monotonic": self.deadline_monotonic,
            "triggered": bool(self.triggered),
            "triggered_monotonic": self.triggered_monotonic,
            "stop_confirmed_monotonic": self.stop_confirmed_monotonic,
            "stop_error": self.stop_error,
        }


class SafeMotorOutput(object):
    def __init__(
        self,
        motor_config=None,
        controller_config=None,
        controller_factory=None,
        allow_reverse=False,
    ):
        self.motor_config = (
            load_motor_config() if motor_config is None else motor_config
        )
        self.controller_config = (
            load_controller_config()
            if controller_config is None
            else controller_config
        )
        self.allow_reverse = bool(allow_reverse)
        self.lease_seconds = float(self.controller_config["command_lease_s"])
        self.voltage_stop = float(
            self.controller_config["motor_voltage_stop_v"]
        )
        self.maximum_voltage = float(
            self.controller_config["maximum_motor_voltage_v"]
        )
        self.post_command_settle = float(
            self.controller_config["post_command_settle_s"]
        )

        if controller_factory is None:
            from manual_motor_control import (
                LEFT_ENA,
                LEFT_IN1,
                LEFT_IN2,
                RIGHT_ENB,
                RIGHT_IN3,
                RIGHT_IN4,
                ManualMotorController,
            )

            self.LEFT_ENA = LEFT_ENA
            self.LEFT_IN1 = LEFT_IN1
            self.LEFT_IN2 = LEFT_IN2
            self.RIGHT_ENB = RIGHT_ENB
            self.RIGHT_IN3 = RIGHT_IN3
            self.RIGHT_IN4 = RIGHT_IN4
            controller_factory = ManualMotorController
        else:
            # These are the project's fixed PCA9685 channel assignments.
            self.LEFT_ENA = 0
            self.LEFT_IN1 = 1
            self.LEFT_IN2 = 2
            self.RIGHT_IN3 = 3
            self.RIGHT_IN4 = 4
            self.RIGHT_ENB = 5

        self.controller = controller_factory(
            reverse_left=bool(self.motor_config.get("reverse_left", False)),
            reverse_right=bool(self.motor_config.get("reverse_right", False)),
        )
        self.lock = threading.RLock()
        self.closed = False
        self.last_signs = (0, 0)
        self.last_duties = (0.0, 0.0)
        self.last_command_time = None
        self.last_command_expiry_monotonic = None
        self.last_command_expiry_reason = None
        self.minimum_voltage = float("inf")
        self.last_voltage = None
        self.last_stop_reason = "INITIALISED_OFF"
        self.watchdog_fault = None
        self.watchdog_trip_count = 0
        self.hardware_off_confirmed = False
        self.hardware_off_retry_required = False
        self.active_since_monotonic = None
        self.last_energized_duration_s = 0.0
        self.maximum_energized_duration_s = 0.0
        self.total_energized_duration_s = 0.0
        self.stop_event = threading.Event()
        try:
            self.controller.all_off()
            self.hardware_off_confirmed = True
        except Exception:
            try:
                self.controller.close()
            except Exception:
                pass
            raise

        self.watchdog = threading.Thread(
            target=self._watchdog_loop,
            name="MotorCommandLeaseWatchdog",
        )
        self.watchdog.daemon = True
        self.watchdog.start()

    @staticmethod
    def _sign(value):
        if value > 0.0:
            return 1
        if value < 0.0:
            return -1
        return 0

    def _watchdog_loop(self):
        # A caller safety deadline can be much shorter than the normal lease.
        # Poll at 20 ms so a stale perception permission cannot linger for the
        # former worst-case 100 ms interval.
        interval = max(0.01, min(0.02, self.lease_seconds / 4.0))
        while not self.stop_event.wait(interval):
            last_command_time = self.last_command_time
            command_expiry = self.last_command_expiry_monotonic
            now = time.monotonic()
            lease_expired = bool(
                last_command_time is not None
                and now - last_command_time > self.lease_seconds
            )
            deadline_expired = bool(
                command_expiry is not None and now >= command_expiry
            )
            retry_off = bool(self.hardware_off_retry_required)
            if self.closed or not (
                lease_expired or deadline_expired or retry_off
            ):
                continue
            acquired = self.lock.acquire(timeout=0.05)
            if not acquired:
                # Software cannot de-assert PWM while the I2C lock/call is
                # blocked. Surface this limitation; a physical disconnect or
                # hardware OE watchdog remains the independent protection.
                if lease_expired or deadline_expired:
                    self.watchdog_fault = (
                        "I2C_LOCK_UNAVAILABLE_AFTER_COMMAND_DEADLINE"
                    )
                continue
            try:
                if self.hardware_off_retry_required:
                    try:
                        self._stop_locked("HARDWARE_OFF_RETRY")
                    except Exception:
                        # _stop_locked keeps the retry flag and latches detail.
                        pass
                if (
                    not self.closed
                    and self.last_command_time is not None
                    and self._command_expired_locked()
                ):
                    self._expire_command_locked()
            finally:
                self.lock.release()

    def _command_expired_locked(self):
        """Return whether the lease or a stricter caller deadline expired."""
        if self.last_command_time is None:
            return False
        now = time.monotonic()
        if now - self.last_command_time > self.lease_seconds:
            return True
        return bool(
            self.last_command_expiry_monotonic is not None
            and now >= self.last_command_expiry_monotonic
        )

    def _arm_command_expiry_locked(
        self, deadline_monotonic=None, deadline_reason=None
    ):
        """Arm the earliest of the fixed lease and caller safety deadline."""
        now = time.monotonic()
        lease_expiry = now + self.lease_seconds
        expiry = lease_expiry
        reason = "COMMAND_LEASE_EXPIRED"
        if deadline_monotonic is not None:
            deadline = float(deadline_monotonic)
            if not math.isfinite(deadline):
                raise ValueError("Motion deadline must be finite.")
            if deadline < expiry:
                expiry = deadline
                reason = str(deadline_reason or "COMMAND_DEADLINE_EXPIRED")
        self.last_command_time = now
        self.last_command_expiry_monotonic = expiry
        self.last_command_expiry_reason = reason

    def _expire_command_locked(self):
        """Stop and latch the active command's earliest expiry."""
        reason = self.last_command_expiry_reason
        if (
            self.last_command_time is not None
            and time.monotonic() - self.last_command_time
            > self.lease_seconds
        ):
            reason = "COMMAND_LEASE_EXPIRED"
        reason = str(reason or "COMMAND_LEASE_EXPIRED")
        if self.watchdog_fault == reason:
            return
        try:
            self._stop_locked(reason)
            self.watchdog_trip_count += 1
            self.watchdog_fault = reason
        except Exception as error:
            self.watchdog_fault = (
                "COMMAND_EXPIRY_STOP_FAILED: {!r}".format(error)
            )

    def _expire_lease_locked(self):
        """Stop and latch one lease expiry while the shared lock is held."""
        self.last_command_expiry_reason = "COMMAND_LEASE_EXPIRED"
        self._expire_command_locked()

    def _stop_locked(self, reason):
        if (
            self.last_signs == (0, 0)
            and self.last_command_time is None
            and self.hardware_off_confirmed
            and not self.hardware_off_retry_required
        ):
            self.last_stop_reason = str(reason)
            self.last_duties = (0.0, 0.0)
            return
        try:
            self.controller.all_off()
            self.hardware_off_confirmed = True
            self.hardware_off_retry_required = False
            self._record_hardware_off_locked(time.monotonic())
        except Exception as error:
            self.hardware_off_confirmed = False
            self.hardware_off_retry_required = True
            self.watchdog_fault = "ALL_OFF_FAILED: {!r}".format(error)
            raise
        finally:
            # Software state is made fail-closed even when the hardware OFF
            # write reports an error. The latched fault prevents a restart;
            # only the physical disconnect can then guarantee de-energizing.
            self.last_signs = (0, 0)
            self.last_duties = (0.0, 0.0)
            self.last_command_time = None
            self.last_command_expiry_monotonic = None
            self.last_command_expiry_reason = None
            self.last_stop_reason = str(reason)

    def _record_hardware_off_locked(self, stopped_monotonic):
        """Close and retain a conservative non-zero PWM interval."""
        if self.active_since_monotonic is None:
            return
        duration = max(
            0.0, float(stopped_monotonic) - self.active_since_monotonic
        )
        self.last_energized_duration_s = duration
        self.maximum_energized_duration_s = max(
            self.maximum_energized_duration_s, duration
        )
        self.total_energized_duration_s += duration
        self.active_since_monotonic = None

    def stop(self, reason="REQUESTED_STOP"):
        with self.lock:
            if not self.closed:
                self._stop_locked(reason)

    def read_voltage(self):
        """Read and record motor-bus voltage under the shared I2C lock."""
        with self.lock:
            if self.closed:
                raise RuntimeError("Motor output is closed.")
            voltage = float(self.controller.read_battery_voltage())
            if not math.isfinite(voltage):
                self._stop_locked("BATTERY_READING_INVALID")
                raise RuntimeError("Battery reading is invalid.")
            self.last_voltage = voltage
            self.minimum_voltage = min(self.minimum_voltage, voltage)
            if voltage < self.voltage_stop:
                self._stop_locked("BATTERY_UNDERVOLTAGE")
                raise RuntimeError(
                    "Battery guard stopped motion: {:.3f} V < {:.3f} V."
                    .format(voltage, self.voltage_stop)
                )
            if voltage > self.maximum_voltage:
                self._stop_locked("BATTERY_OVERVOLTAGE_OR_SENSOR_FAULT")
                raise RuntimeError(
                    "Battery reading exceeds the 2S limit: {:.3f} V > {:.3f} V."
                    .format(voltage, self.maximum_voltage)
                )
            return voltage

    def _configure_directions_locked(self, left_sign, right_sign):
        old_left, old_right = self.last_signs
        reversal = (
            old_left != 0 and left_sign != 0 and old_left != left_sign
        ) or (
            old_right != 0 and right_sign != 0 and old_right != right_sign
        )

        self.controller.set_channel_duty(self.LEFT_ENA, 0.0)
        self.controller.set_channel_duty(self.RIGHT_ENB, 0.0)
        self.hardware_off_confirmed = True
        self._record_hardware_off_locked(time.monotonic())
        if reversal:
            time.sleep(0.08)

        self.controller.configure_motor_direction(
            left_sign,
            self.LEFT_IN1,
            self.LEFT_IN2,
            self.controller.reverse_left,
        )
        self.controller.configure_motor_direction(
            right_sign,
            self.RIGHT_IN3,
            self.RIGHT_IN4,
            self.controller.reverse_right,
        )
        time.sleep(0.02)
        self.last_signs = (left_sign, right_sign)

    def command(
        self,
        left_normalized,
        right_normalized,
        reason="CONTROL_LOOP",
        deadline_monotonic=None,
        deadline_reason="COMMAND_DEADLINE_EXPIRED",
    ):
        """Apply normalized wheel commands and refresh the watchdog lease."""
        left_normalized = float(left_normalized)
        right_normalized = float(right_normalized)
        if not math.isfinite(left_normalized) or not math.isfinite(right_normalized):
            self.stop("NONFINITE_COMMAND")
            raise ValueError("Motor commands must be finite.")
        if not self.allow_reverse and (
            left_normalized < 0.0 or right_normalized < 0.0
        ):
            self.stop("REVERSE_COMMAND_REFUSED")
            raise ValueError("Reverse commands are disabled for autonomous tests.")

        left_normalized = max(-1.0, min(1.0, left_normalized))
        right_normalized = max(-1.0, min(1.0, right_normalized))

        with self.lock:
            if self.closed:
                raise RuntimeError("Motor output is closed.")
            self._assert_motion_deadline_allowed_locked(deadline_monotonic)
            self._assert_lease_refresh_allowed_locked()

            try:
                pre_voltage = self.read_voltage()

                left_duty = map_signed_duty(
                    left_normalized,
                    is_right=False,
                    config=self.motor_config,
                )
                right_duty = map_signed_duty(
                    right_normalized,
                    is_right=True,
                    config=self.motor_config,
                )
                signs = (self._sign(left_duty), self._sign(right_duty))
                if signs != self.last_signs:
                    self._configure_directions_locked(signs[0], signs[1])

                # A pre-voltage/direction I2C call may have held the lock past
                # the previous lease deadline. Never let the eventual refresh
                # erase that expiry or a watchdog lock-timeout fault.
                self._assert_motion_deadline_allowed_locked(
                    deadline_monotonic
                )
                self._assert_lease_refresh_allowed_locked()

                previous_duties = self.last_duties
                commanded_duties = (abs(left_duty), abs(right_duty))
                potentially_active = max(commanded_duties) > 0.0
                if potentially_active:
                    # Arm the lease before the first EN write. If the second
                    # I2C write blocks after one side energizes, the watchdog
                    # can now detect the in-progress command deadline.
                    self.last_duties = commanded_duties
                    self._arm_command_expiry_locked(
                        deadline_monotonic, deadline_reason
                    )
                    self.last_stop_reason = "COMMAND_WRITE_IN_PROGRESS"
                    if self.active_since_monotonic is None:
                        # Timestamp before the first non-zero EN write.  The
                        # resulting duration is a conservative upper bound.
                        self.active_since_monotonic = time.monotonic()
                    self.hardware_off_confirmed = False
                self.controller.set_channel_duty(
                    self.LEFT_ENA, abs(left_duty)
                )
                # The left EN write is an external I2C transaction and may
                # return only after the observation or command lease has
                # expired.  Re-check before issuing a non-zero command to the
                # right EN channel.  Without this interlock, a delayed left
                # write could cause a *new* right-side actuation after the
                # permission deadline.
                if potentially_active:
                    self._assert_motion_deadline_allowed_locked(
                        deadline_monotonic
                    )
                    self._assert_lease_refresh_allowed_locked()
                self.controller.set_channel_duty(
                    self.RIGHT_ENB, abs(right_duty)
                )
                if potentially_active:
                    self._assert_motion_deadline_allowed_locked(
                        deadline_monotonic
                    )
                    self._assert_lease_refresh_allowed_locked()
                    self._arm_command_expiry_locked(
                        deadline_monotonic, deadline_reason
                    )
                else:
                    self._stop_locked(reason)
                self.last_stop_reason = str(reason)
                self.last_duties = commanded_duties
                if not potentially_active:
                    self.hardware_off_confirmed = True
                requires_settled_check = (
                    max(previous_duties) <= 0.0
                    or abs(abs(left_duty) - previous_duties[0]) >= 0.10
                    or abs(abs(right_duty) - previous_duties[1]) >= 0.10
                )
                post_voltage = None
                if requires_settled_check and max(self.last_duties) > 0.0:
                    time.sleep(self.post_command_settle)
                    post_voltage = self.read_voltage()
                    self._assert_motion_deadline_allowed_locked(
                        deadline_monotonic
                    )
                    self._assert_lease_refresh_allowed_locked()
                    self._arm_command_expiry_locked(
                        deadline_monotonic, deadline_reason
                    )
                return {
                    "left_duty": abs(float(left_duty)),
                    "right_duty": abs(float(right_duty)),
                    "battery_pre_command_v": pre_voltage,
                    "battery_post_command_v": post_voltage,
                    "battery_voltage": (
                        post_voltage if post_voltage is not None else pre_voltage
                    ),
                    "command_expiry_monotonic": (
                        self.last_command_expiry_monotonic
                    ),
                    "command_expiry_reason": self.last_command_expiry_reason,
                }
            except BaseException:
                already_stopped = (
                    self.last_signs == (0, 0)
                    and self.last_duties == (0.0, 0.0)
                    and self.last_command_time is None
                    and self.hardware_off_confirmed
                    and not self.hardware_off_retry_required
                )
                if not already_stopped:
                    try:
                        self._stop_locked("COMMAND_TRANSACTION_FAILED")
                    except Exception as stop_error:
                        self.watchdog_fault = (
                            "COMMAND_TRANSACTION_FAILSAFE_OFF_FAILED: {!r}"
                            .format(stop_error)
                        )
                # Preserve the original voltage/direction/PWM exception.
                raise

    def _assert_motion_deadline_allowed_locked(self, deadline_monotonic):
        if deadline_monotonic is None:
            return
        deadline = float(deadline_monotonic)
        if not math.isfinite(deadline):
            self._stop_locked("INVALID_MOTION_DEADLINE")
            raise ValueError("Motion deadline must be finite.")
        if time.monotonic() >= deadline:
            self._stop_locked("MOTION_DEADLINE_EXPIRED")
            raise RuntimeError("Motion deadline expired before command refresh.")

    def _assert_lease_refresh_allowed_locked(self):
        if self.watchdog_fault is not None:
            fault = self.watchdog_fault
            self._stop_locked("SOFTWARE_WATCHDOG_FAULT")
            raise RuntimeError(
                "Software command lease fault: {}".format(fault)
            )
        if self._command_expired_locked():
            # Do not let a late refresh erase an expiry merely because it
            # arrived before the watchdog thread next acquired this lock.
            self._expire_command_locked()
            raise RuntimeError(
                "Software command deadline expired before this refresh."
            )

    def close(self):
        stop_error = None
        with self.lock:
            if self.closed:
                return
            try:
                for attempt in range(3):
                    try:
                        self._stop_locked("CLOSED")
                        stop_error = None
                        break
                    except Exception as error:
                        stop_error = error
                        if attempt < 2:
                            time.sleep(0.02)
            finally:
                self.closed = True
                self.stop_event.set()
        self.watchdog.join(timeout=1.0)
        controller_error = None
        try:
            self.controller.close()
        except Exception as error:
            controller_error = error
        if stop_error is not None:
            raise stop_error
        if controller_error is not None:
            raise controller_error

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.close()
