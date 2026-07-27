#!/usr/bin/env python3
"""Pure lane-control and safety logic with no hardware dependencies."""

from __future__ import print_function

import json
import math
import os
import time


PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONTROLLER_CONFIG = os.path.join(
    PROJECT_DIRECTORY, "controller_config.json"
)
DEFAULT_MOTOR_CONFIG = os.path.join(PROJECT_DIRECTORY, "motor_config.json")


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def load_json(path):
    with open(path, "r") as input_file:
        return json.load(input_file)


def validate_controller_config(config):
    required = (
        "kp",
        "ki",
        "kd",
        "integral_limit",
        "derivative_filter_alpha",
        "steering_limit",
        "lane_confidence_minimum",
        "partial_lane_timeout_s",
        "camera_stale_timeout_s",
        "yolo_confidence_threshold",
        "yolo_iou_threshold",
        "yolo_stale_timeout_s",
        "command_lease_s",
        "motor_voltage_stop_v",
        "maximum_motor_voltage_v",
        "post_command_settle_s",
        "slow_speed_scale",
        "partial_lane_speed_scale",
    )
    required_non_numeric = (
        "verification_state",
        "allow_partial_lane_motion",
        "yolo_image_size",
        "yolo_max_frame_lag",
    )
    missing = [
        key for key in required + required_non_numeric if key not in config
    ]
    if missing:
        raise ValueError(
            "Controller config missing: {}".format(", ".join(missing))
        )
    for key in required:
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("{} must be a JSON number.".format(key))
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("{} must be finite.".format(key))
    if not isinstance(config["allow_partial_lane_motion"], bool):
        raise ValueError("allow_partial_lane_motion must be JSON true or false.")
    if config["verification_state"] not in (
        "SOFTWARE_INITIAL_VALUES_NOT_PHYSICALLY_TUNED",
        "PHYSICALLY_TUNED",
    ):
        raise ValueError("verification_state is not recognized.")
    if (
        isinstance(config["yolo_image_size"], bool)
        or not isinstance(config["yolo_image_size"], int)
        or not 224 <= config["yolo_image_size"] <= 416
        or config["yolo_image_size"] % 32 != 0
    ):
        raise ValueError(
            "yolo_image_size must be a multiple of 32 from 224 to 416."
        )
    if (
        isinstance(config["yolo_max_frame_lag"], bool)
        or not isinstance(config["yolo_max_frame_lag"], int)
        or not 1 <= config["yolo_max_frame_lag"] <= 30
    ):
        raise ValueError("yolo_max_frame_lag must be an integer from 1 to 30.")

    ranges = {
        "kp": (0.0, 2.0),
        "ki": (0.0, 2.0),
        "kd": (0.0, 2.0),
        "integral_limit": (0.0, 2.0),
        "derivative_filter_alpha": (0.0, 1.0),
        "steering_limit": (0.01, 0.35),
        "lane_confidence_minimum": (0.40, 0.95),
        "partial_lane_timeout_s": (0.0, 1.0),
        "camera_stale_timeout_s": (0.05, 0.50),
        "yolo_confidence_threshold": (0.25, 0.60),
        "yolo_iou_threshold": (0.30, 0.70),
        "yolo_stale_timeout_s": (0.25, 1.0),
        "command_lease_s": (0.25, 0.50),
        "motor_voltage_stop_v": (6.50, 7.20),
        "maximum_motor_voltage_v": (8.40, 8.70),
        "post_command_settle_s": (0.02, 0.15),
        "slow_speed_scale": (0.0, 0.8),
        "partial_lane_speed_scale": (0.0, 0.8),
    }
    for key, limits in ranges.items():
        value = float(config[key])
        if not limits[0] <= value <= limits[1]:
            raise ValueError(
                "{} must be within {}..{}.".format(key, limits[0], limits[1])
            )
    if float(config["motor_voltage_stop_v"]) >= float(
        config["maximum_motor_voltage_v"]
    ):
        raise ValueError("Motor voltage stop must be below maximum voltage.")
    if float(config["post_command_settle_s"]) > (
        0.5 * float(config["command_lease_s"])
    ):
        raise ValueError(
            "post_command_settle_s must be at most half command_lease_s."
        )
    return config


def load_controller_config(path=DEFAULT_CONTROLLER_CONFIG):
    return validate_controller_config(load_json(path))


def cruise_normalized_command(motor_config):
    """Invert the deadband mapping for the calibrated cruise duty."""
    deadband = float(motor_config["deadband_duty"])
    maximum = float(motor_config["max_duty"])
    cruise = float(motor_config["cruise_duty"])
    if not 0.0 <= deadband < cruise <= maximum <= 1.0:
        raise ValueError(
            "Expected deadband_duty < cruise_duty <= max_duty <= 1."
        )
    return (cruise - deadband) / (maximum - deadband)


class PIDController(object):
    """Time-aware PID with derivative filtering and anti-windup."""

    def __init__(self, config):
        self.kp = float(config["kp"])
        self.ki = float(config["ki"])
        self.kd = float(config["kd"])
        self.integral_limit = abs(float(config["integral_limit"]))
        self.derivative_alpha = float(config["derivative_filter_alpha"])
        self.output_limit = abs(float(config["steering_limit"]))
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None
        self.filtered_derivative = 0.0

    def update(self, error, timestamp=None):
        error = float(error)
        if not math.isfinite(error):
            raise ValueError("PID error must be finite.")
        timestamp = time.monotonic() if timestamp is None else float(timestamp)

        if self.previous_time is None:
            dt = 0.0
            derivative = 0.0
        else:
            dt = clamp(timestamp - self.previous_time, 0.001, 0.25)
            derivative = (error - self.previous_error) / dt

        alpha = self.derivative_alpha
        self.filtered_derivative = (
            alpha * derivative + (1.0 - alpha) * self.filtered_derivative
        )

        candidate_integral = self.integral
        if dt > 0.0:
            candidate_integral = clamp(
                self.integral + error * dt,
                -self.integral_limit,
                self.integral_limit,
            )

        p_term = self.kp * error
        i_term = self.ki * candidate_integral
        d_term = self.kd * self.filtered_derivative
        unclamped = p_term + i_term + d_term
        output = clamp(unclamped, -self.output_limit, self.output_limit)

        # Conditional integration: do not deepen output saturation.
        if (
            unclamped == output
            or (unclamped > output and error < 0.0)
            or (unclamped < output and error > 0.0)
        ):
            self.integral = candidate_integral
        i_term = self.ki * self.integral

        self.previous_error = error
        self.previous_time = timestamp
        return {
            "output": float(output),
            "p": float(p_term),
            "i": float(i_term),
            "d": float(d_term),
            "dt": float(dt),
        }


def mix_forward(base_command, steering_command):
    """Mix forward speed and steering into normalized wheel commands.

    Positive steering turns right: the left wheel is accelerated and the
    right wheel is slowed. Reverse commands are intentionally not generated.
    """
    base = clamp(float(base_command), 0.0, 1.0)
    if base <= 0.0:
        return 0.0, 0.0
    # Preserve at least 20% of the base command on the inner wheel. This keeps
    # a caution-speed correction out of the measured stop/deadband boundary
    # and avoids an unintended pivot caused by one side repeatedly reaching 0.
    steering = clamp(float(steering_command), -0.8 * base, 0.8 * base)
    left = clamp(base + steering, 0.0, 1.0)
    right = clamp(base - steering, 0.0, 1.0)
    return left, right


class SafetySupervisor(object):
    """Fail-safe perception gate for a physical motion command."""

    def __init__(self, config):
        self.config = config
        self.partial_started = None

    def stop(self, reason):
        return {"state": "STOP", "reason": reason, "speed_scale": 0.0}

    def evaluate(
        self,
        lane_status,
        lane_confidence,
        camera_age_s,
        yolo_state,
        obstacle_action,
        now=None,
    ):
        now = time.monotonic() if now is None else float(now)
        if camera_age_s is None or not math.isfinite(float(camera_age_s)):
            return self.stop("CAMERA_TIMESTAMP_MISSING")
        if float(camera_age_s) < 0.0:
            return self.stop("CAMERA_TIMESTAMP_FUTURE")
        if float(camera_age_s) > float(self.config["camera_stale_timeout_s"]):
            return self.stop("CAMERA_STALE")

        usable_full = lane_status == "FULL"
        usable_partial = str(lane_status).startswith("PARTIAL")
        if not usable_full and not usable_partial:
            self.partial_started = None
            return self.stop("LANE_{}".format(lane_status))
        if not math.isfinite(float(lane_confidence)):
            return self.stop("LANE_CONFIDENCE_INVALID")
        if usable_partial and not self.config["allow_partial_lane_motion"]:
            return self.stop("PARTIAL_LANE_MOTION_NOT_VALIDATED")
        if float(lane_confidence) < float(
            self.config["lane_confidence_minimum"]
        ):
            return self.stop("LANE_CONFIDENCE_LOW")

        if yolo_state.get("error"):
            return self.stop("YOLO_WORKER_ERROR")
        if not yolo_state.get("has_result"):
            return self.stop("YOLO_STARTUP_NO_RESULT")
        yolo_age = float(yolo_state.get("age_seconds", float("inf")))
        if not math.isfinite(yolo_age) or yolo_age < 0.0 or yolo_age > float(
            self.config["yolo_stale_timeout_s"]
        ):
            return self.stop("YOLO_STALE")
        frame_lag = yolo_state.get("source_frame_lag")
        if frame_lag is None or int(frame_lag) > int(
            self.config["yolo_max_frame_lag"]
        ):
            return self.stop("YOLO_FRAME_LAG")

        if obstacle_action["state"] == "STOP":
            self.partial_started = None
            return self.stop(obstacle_action["reason"])

        speed_scale = 1.0
        reasons = []
        if obstacle_action["state"] == "SLOW":
            speed_scale = min(
                speed_scale, float(self.config["slow_speed_scale"])
            )
            reasons.append(obstacle_action["reason"])

        if usable_partial:
            if self.partial_started is None:
                self.partial_started = now
            if now - self.partial_started > float(
                self.config["partial_lane_timeout_s"]
            ):
                return self.stop("PARTIAL_LANE_TIMEOUT")
            speed_scale = min(
                speed_scale, float(self.config["partial_lane_speed_scale"])
            )
            reasons.append("PARTIAL_LANE")
        else:
            self.partial_started = None

        if speed_scale < 1.0:
            if speed_scale <= 0.0:
                return self.stop("{}_MOTION_NOT_VALIDATED".format(
                    "+".join(reasons) if reasons else "REDUCED_SPEED"
                ))
            return {
                "state": "CAUTION",
                "reason": "+".join(reasons),
                "speed_scale": speed_scale,
            }
        return {"state": "DRIVE", "reason": "ALL_GATES_VALID", "speed_scale": 1.0}
