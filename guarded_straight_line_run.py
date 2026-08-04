#!/usr/bin/env python3
"""Guarded, camera-steered commissioning run for one straight black tape line.

This tool is deliberately narrower than the project's integrated runner.  It
uses the current physical CSI camera, current IPM/single-line detector, current
motor calibration, INA219 voltage checks, motor command-lease watchdog and an
independent hard-duration stopper.  YOLO is intentionally not loaded so that
the Jetson Nano can use the fastest available loop for steering.

This is therefore a LINE-FOLLOWING COMMISSIONING TEST, not evidence of
automatic obstacle avoidance.  The lane must be empty and flat, and a spotter
must hold the physical motor-battery disconnect throughout the run.  Every
validation gate must be current before the motor adapter can be imported.
"""

from __future__ import print_function

import argparse
import atexit
import csv
import datetime
import hashlib
import inspect
import json
import math
import os
import signal
import sys
import time


# Keep OpenCV/NumPy from oversubscribing the Nano's CPU cores.
for _name in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
):
    os.environ[_name] = "1"


MINIMUM_DURATION_SECONDS = 0.5
MAXIMUM_DURATION_SECONDS = 4.0
DEADLINE_STOP_MARGIN_SECONDS = 0.10

CAMERA_CAPTURE_WIDTH = 1280
CAMERA_CAPTURE_HEIGHT = 720
CAMERA_OUTPUT_WIDTH = 480
CAMERA_OUTPUT_HEIGHT = 270
CAMERA_FRAMERATE = 21
CAMERA_SENSOR_MODE = 4
CAMERA_FLIP_METHOD = 2
CAMERA_STALE_LIMIT_SECONDS = 0.25

LINE_CONFIDENCE_MINIMUM = 0.55
ARM_REQUIRED_CONSECUTIVE_FRAMES = 2
# These are practical placement limits, not camera-calibration targets.  The
# camera remains mechanically fixed after calibration; the controller handles
# small vehicle-placement errors within this envelope.
ARM_MAXIMUM_NEAR_OFFSET = 0.15
ARM_MAXIMUM_LOOKAHEAD_OFFSET = 0.25
ARM_MAXIMUM_HEADING_RAD = 0.25
ARM_TIMEOUT_SECONDS = 30.0

# These are multipliers of the calibrated normalized cruise command, not raw
# PWM percentages.  With the recorded motor_config.json they map to:
# breakaway L 0.7875 / R 0.77175 and cruise L 0.7275 / R 0.71475.
BREAKAWAY_SPEED_SCALE = 1.25
BREAKAWAY_SECONDS = 0.25
STRAIGHT_SPEED_SCALE = 0.85

# Straight-only controller.  Positive steering speeds the logical left wheel
# and slows the logical right wheel, turning the car right.  The measured-dt
# integral term can learn steady drivetrain yaw without altering the persisted
# motor calibration or hard-coding an unverified trim.
TRACKING_NEAR_WEIGHT = 0.45
TRACKING_LOOKAHEAD_WEIGHT = 0.55
TRACKING_ERROR_DEADBAND = 0.025
STRAIGHT_KP = 0.75
STRAIGHT_KI = 0.80
STRAIGHT_INTEGRAL_LIMIT = 0.35
STEERING_LIMIT = 0.35
DEFAULT_RIGHT_STEERING_BIAS = 0.0

# Never command a loaded motor just above its measured 0.60 PWM deadband.
# Exact zero remains OFF.  A non-zero command is raised to at least the
# side-specific deadband plus this margin.
ACTIVE_DUTY_MARGIN = 0.055
COMMAND_LEASE_SECONDS = 0.35


_EMERGENCY_OUTPUT = None


def clamp(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), float(value)))


def finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise RuntimeError("Missing or invalid {}.".format(name))
    if not math.isfinite(number):
        raise RuntimeError("{} is not finite.".format(name))
    return number


def utc_now():
    return datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        while True:
            block = source.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def safe_label(value):
    cleaned = "".join(
        character if character.isalnum() or character in "-_" else "_"
        for character in str(value)
    )
    return cleaned[:48] or "straight_line"


class StraightPI(object):
    """Measured-dt PI controller with bounded integral state."""

    def __init__(self):
        self.integral = 0.0
        self.previous_time = None

    def reset(self):
        self.integral = 0.0
        self.previous_time = None

    def update(self, error, timestamp):
        error = finite(error, "tracking error")
        timestamp = finite(timestamp, "controller timestamp")
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = clamp(timestamp - self.previous_time, 0.001, 0.25)
        candidate = self.integral
        if dt > 0.0:
            candidate = clamp(
                self.integral + error * dt,
                -STRAIGHT_INTEGRAL_LIMIT,
                STRAIGHT_INTEGRAL_LIMIT,
            )
        p_term = STRAIGHT_KP * error
        i_term = STRAIGHT_KI * candidate
        raw = p_term + i_term
        output = clamp(raw, -STEERING_LIMIT, STEERING_LIMIT)
        # Do not deepen saturation.
        if (
            raw == output
            or (raw > output and error < 0.0)
            or (raw < output and error > 0.0)
        ):
            self.integral = candidate
            i_term = STRAIGHT_KI * self.integral
        self.previous_time = timestamp
        return {
            "output": float(output),
            "p": float(p_term),
            "i": float(i_term),
            "dt": float(dt),
        }


def cruise_normalized_command(motor_config):
    deadband = finite(motor_config["deadband_duty"], "deadband duty")
    cruise = finite(motor_config["cruise_duty"], "cruise duty")
    maximum = finite(motor_config["max_duty"], "maximum duty")
    if not 0.0 <= deadband < cruise <= maximum <= 1.0:
        raise RuntimeError("Invalid motor calibration ordering.")
    return (cruise - deadband) / (maximum - deadband)


def mix_forward(base_command, steering_command):
    """Positive steering turns right; reverse is never generated."""
    base = clamp(base_command, 0.0, 1.0)
    if base <= 0.0:
        return 0.0, 0.0
    steering = clamp(steering_command, -0.8 * base, 0.8 * base)
    return (
        clamp(base + steering, 0.0, 1.0),
        clamp(base - steering, 0.0, 1.0),
    )


def enforce_active_duty_margin(left_command, right_command, motor_config):
    logical_deadband = finite(
        motor_config["deadband_duty"], "logical deadband"
    )
    logical_cruise = finite(motor_config["cruise_duty"], "logical cruise")
    logical_maximum = finite(motor_config["max_duty"], "logical maximum")
    cruise_command = (
        (logical_cruise - logical_deadband)
        / (logical_maximum - logical_deadband)
    )
    adjusted = []
    for command, side in (
        (finite(left_command, "left command"), "left"),
        (finite(right_command, "right command"), "right"),
    ):
        if command <= 0.0:
            adjusted.append(0.0)
            continue
        side_deadband = finite(
            motor_config[side + "_deadband_duty"],
            side + " deadband",
        )
        side_cruise = finite(
            motor_config[side + "_cruise_duty"],
            side + " cruise",
        )
        available = side_cruise - side_deadband
        if available <= ACTIVE_DUTY_MARGIN:
            raise RuntimeError(
                "{} motor calibration cannot provide active margin."
                .format(side)
            )
        minimum_command = (
            cruise_command * ACTIVE_DUTY_MARGIN / available
        )
        adjusted.append(clamp(max(command, minimum_command), 0.0, 1.0))
    return tuple(adjusted)


def observation_values(observation):
    return {
        "status": str(observation.get("status")),
        "confidence": finite(
            observation.get("confidence"), "line confidence"
        ),
        "near": finite(
            observation.get("near_field_offset"), "near-field offset"
        ),
        "lookahead": finite(
            observation.get("lookahead_offset"), "lookahead offset"
        ),
        "heading": finite(
            observation.get("heading_error_rad"), "line heading"
        ),
    }


def line_is_usable(observation):
    values = observation_values(observation)
    if values["status"] != "FULL":
        return False, "LINE_{}".format(values["status"]), values
    if values["confidence"] < LINE_CONFIDENCE_MINIMUM:
        return False, "LINE_CONFIDENCE_LOW", values
    return True, "LINE_VALID", values


def line_is_armed(observation):
    usable, reason, values = line_is_usable(observation)
    if not usable:
        return False, reason, values
    if abs(values["near"]) > ARM_MAXIMUM_NEAR_OFFSET:
        return False, "START_NEAR_OFFSET_TOO_LARGE", values
    if abs(values["lookahead"]) > ARM_MAXIMUM_LOOKAHEAD_OFFSET:
        return False, "START_LOOKAHEAD_OFFSET_TOO_LARGE", values
    if abs(values["heading"]) > ARM_MAXIMUM_HEADING_RAD:
        return False, "START_HEADING_TOO_LARGE", values
    return True, "START_ALIGNED", values


def apply_tracking_deadband(error):
    """Suppress centre-line image noise without creating a discontinuity."""
    value = clamp(finite(error, "tracking error"), -1.0, 1.0)
    magnitude = abs(value)
    if magnitude <= TRACKING_ERROR_DEADBAND:
        return 0.0
    adjusted = (
        (magnitude - TRACKING_ERROR_DEADBAND)
        / (1.0 - TRACKING_ERROR_DEADBAND)
    )
    return math.copysign(adjusted, value)


def validate_camera_metadata(metadata, after_processing=True):
    if not isinstance(metadata, dict):
        raise RuntimeError("CAMERA_METADATA_MISSING")
    if not metadata.get("timestamp_valid"):
        raise RuntimeError("CAMERA_TIMESTAMP_INVALID")
    if not metadata.get("source_age_trustworthy"):
        raise RuntimeError("CAMERA_SOURCE_AGE_UNTRUSTWORTHY")
    if not metadata.get("fresh"):
        raise RuntimeError("CAMERA_FRAME_NOT_FRESH")
    capture_time = finite(
        metadata.get("capture_monotonic"), "camera capture timestamp"
    )
    age = time.monotonic() - capture_time
    if age < 0.0:
        raise RuntimeError("CAMERA_TIMESTAMP_FUTURE")
    if age > CAMERA_STALE_LIMIT_SECONDS:
        phase = "AFTER_PROCESSING" if after_processing else "AT_ARRIVAL"
        raise RuntimeError(
            "CAMERA_STALE_{}: {:.1f} ms".format(phase, age * 1000.0)
        )
    return float(age)


def build_camera(camera_class):
    arguments = {
        "sensor_id": 0,
        "capture_width": CAMERA_CAPTURE_WIDTH,
        "capture_height": CAMERA_CAPTURE_HEIGHT,
        "output_width": CAMERA_OUTPUT_WIDTH,
        "output_height": CAMERA_OUTPUT_HEIGHT,
        "framerate": CAMERA_FRAMERATE,
        "flip_method": CAMERA_FLIP_METHOD,
    }
    try:
        parameters = inspect.signature(camera_class.__init__).parameters
        if "sensor_mode" in parameters:
            arguments["sensor_mode"] = CAMERA_SENSOR_MODE
    except (TypeError, ValueError):
        pass
    return camera_class(**arguments)


def capture_observation(camera, detector):
    success, frame, metadata = camera.read_with_metadata(
        timeout_seconds=2.0
    )
    if not success or frame is None:
        raise RuntimeError("CAMERA_READ_FAILED")
    arrival_age = validate_camera_metadata(metadata, after_processing=False)
    processing_started = time.monotonic()
    observation = detector.observe(frame)
    processing_ms = (time.monotonic() - processing_started) * 1000.0
    current_age = validate_camera_metadata(metadata, after_processing=True)
    return {
        "frame": frame,
        "metadata": metadata,
        "observation": observation,
        "arrival_age_s": arrival_age,
        "camera_age_s": current_age,
        "processing_ms": processing_ms,
    }


def show_sample(cv2_module, sample, title):
    observation = sample["observation"]
    original = observation.get("original_overlay")
    warped = observation.get("warped_overlay")
    if original is not None:
        cv2_module.imshow(title + " - camera", original)
    if warped is not None:
        cv2_module.imshow(title + " - warped", warped)
    key = int(cv2_module.waitKey(1)) & 0xFF
    if key in (27, ord("q"), ord("Q")):
        raise KeyboardInterrupt()


def wait_for_alignment(
    camera,
    detector,
    cv2_module,
    headless,
    label,
    require_start_alignment=True,
):
    consecutive = 0
    attempts = 0
    started = time.monotonic()
    last_reason = "NO_FRAME"
    last_sample = None
    while time.monotonic() - started < ARM_TIMEOUT_SECONDS:
        attempts += 1
        try:
            sample = capture_observation(camera, detector)
            last_sample = sample
            if require_start_alignment:
                ready, reason, values = line_is_armed(
                    sample["observation"]
                )
            else:
                ready, reason, values = line_is_usable(
                    sample["observation"]
                )
            if ready:
                consecutive += 1
            else:
                consecutive = 0
            last_reason = reason
            print(
                "{} | {} | conf {:.3f} near {:+.3f} look {:+.3f} "
                "heading {:+.3f} | age {:.0f} ms | ready {}/{}".format(
                    label,
                    values["status"],
                    values["confidence"],
                    values["near"],
                    values["lookahead"],
                    values["heading"],
                    sample["camera_age_s"] * 1000.0,
                    consecutive,
                    ARM_REQUIRED_CONSECUTIVE_FRAMES,
                )
            )
            if not headless:
                show_sample(cv2_module, sample, "Straight-line alignment")
            if consecutive >= ARM_REQUIRED_CONSECUTIVE_FRAMES:
                return sample, attempts
        except KeyboardInterrupt:
            raise
        except Exception as error:
            consecutive = 0
            last_reason = str(error)
            print("{} | waiting: {}".format(label, last_reason))
    raise RuntimeError(
        "Alignment did not remain ready for {} consecutive frames in "
        "{:.0f} s. Last reason: {}".format(
            ARM_REQUIRED_CONSECUTIVE_FRAMES,
            ARM_TIMEOUT_SECONDS,
            last_reason,
        )
    )


def emergency_stop():
    output = _EMERGENCY_OUTPUT
    if output is not None:
        try:
            output.stop("PROCESS_EXIT_EMERGENCY_STOP")
        except Exception:
            pass


def signal_stop(_signum, _frame):
    emergency_stop()
    raise KeyboardInterrupt()


def run_self_test():
    config = {
        "deadband_duty": 0.60,
        "cruise_duty": 0.75,
        "max_duty": 0.98,
        "left_deadband_duty": 0.60,
        "right_deadband_duty": 0.60,
        "left_cruise_duty": 0.75,
        "right_cruise_duty": 0.735,
    }
    base = cruise_normalized_command(config)
    breakaway = base * BREAKAWAY_SPEED_SCALE
    cruise = base * STRAIGHT_SPEED_SCALE

    def mapped(command, side):
        side_deadband = float(config[side + "_deadband_duty"])
        side_cruise = float(config[side + "_cruise_duty"])
        if float(command) <= base:
            fraction = float(command) / base
            return side_deadband + fraction * (
                side_cruise - side_deadband
            )
        side_maximum = 0.98 if side == "left" else 0.9604
        fraction = (float(command) - base) / (1.0 - base)
        return side_cruise + fraction * (
            side_maximum - side_cruise
        )

    assert abs(base - 0.39473684210526316) < 1e-12
    assert breakaway > cruise > 0.0
    assert abs(mapped(breakaway, "left") - 0.7875) < 1e-12
    assert abs(mapped(breakaway, "right") - 0.77175) < 1e-12
    assert abs(mapped(cruise, "left") - 0.7275) < 1e-12
    assert abs(mapped(cruise, "right") - 0.71475) < 1e-12
    left, right = mix_forward(cruise, +0.08)
    assert left > right, "Positive steering must turn right."
    left, right = mix_forward(cruise, -0.08)
    assert left < right, "Negative steering must turn left."
    left, right = enforce_active_duty_margin(cruise, 0.001, config)
    assert left > 0.0 and right > 0.001
    assert enforce_active_duty_margin(0.0, 0.0, config) == (0.0, 0.0)
    assert apply_tracking_deadband(0.01) == 0.0
    assert apply_tracking_deadband(-0.01) == 0.0
    assert apply_tracking_deadband(+0.10) > 0.0
    assert apply_tracking_deadband(-0.10) < 0.0
    assert line_is_armed({
        "status": "FULL",
        "confidence": 0.90,
        "near_field_offset": 0.0,
        "lookahead_offset": 0.0,
        "heading_error_rad": 0.0,
    })[0]
    assert not line_is_armed({
        "status": "LOST",
        "confidence": 0.0,
        "near_field_offset": 0.0,
        "lookahead_offset": 0.0,
        "heading_error_rad": 0.0,
    })[0]
    assert not line_is_armed({
        "status": "FULL",
        "confidence": 0.20,
        "near_field_offset": 0.0,
        "lookahead_offset": 0.0,
        "heading_error_rad": 0.0,
    })[0]
    for bad_status in ("PARTIAL_LEFT", "PARTIAL_RIGHT", "AMBIGUOUS", "INVALID_WIDTH"):
        assert not line_is_armed({
            "status": bad_status,
            "confidence": 0.90,
            "near_field_offset": 0.0,
            "lookahead_offset": 0.0,
            "heading_error_rad": 0.0,
        })[0]
    metadata = {
        "timestamp_valid": True,
        "source_age_trustworthy": True,
        "fresh": True,
        "capture_monotonic": time.monotonic(),
    }
    assert validate_camera_metadata(metadata) <= CAMERA_STALE_LIMIT_SECONDS
    stale = dict(metadata)
    stale["capture_monotonic"] = time.monotonic() - 1.0
    try:
        validate_camera_metadata(stale)
        raise AssertionError("Stale camera metadata was accepted.")
    except RuntimeError:
        pass
    future = dict(metadata)
    future["capture_monotonic"] = time.monotonic() + 1.0
    try:
        validate_camera_metadata(future)
        raise AssertionError("Future camera metadata was accepted.")
    except RuntimeError:
        pass
    minimum_left, minimum_right = enforce_active_duty_margin(
        0.001, 0.001, config
    )
    assert mapped(minimum_left, "left") >= 0.655 - 1e-12
    assert mapped(minimum_right, "right") >= 0.655 - 1e-12
    assert MINIMUM_DURATION_SECONDS == 0.5
    assert MAXIMUM_DURATION_SECONDS == 4.0
    print("PASS calibrated breakaway PWM L 0.7875 / R 0.77175")
    print("PASS calibrated straight PWM L 0.7275 / R 0.71475")
    print("PASS positive steering increases left and reduces right command")
    print("PASS negative steering increases right and reduces left command")
    print("PASS non-zero inner wheel is held above loaded deadband margin")
    print("PASS exact zero remains fully OFF")
    print("PASS centred image noise produces zero steering correction")
    print("PASS arming requires FULL centred line")
    print("PASS LOST line cannot arm")
    print("PASS partial, ambiguous and invalid-width observations cannot arm")
    print("PASS low line confidence cannot arm")
    print("PASS stale and future camera data are rejected")
    print("PASS duration is restricted to 0.5..4.0 seconds")
    print("GUARDED STRAIGHT-LINE POLICY SELF-TEST: PASS")
    return 0


def main():
    global _EMERGENCY_OUTPUT

    parser = argparse.ArgumentParser(
        description="Guarded straight black-tape line commissioning run"
    )
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--seconds", type=float, default=0.5)
    parser.add_argument("--run-label", default="straight_line")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--steering-bias",
        type=float,
        default=DEFAULT_RIGHT_STEERING_BIAS,
        help=(
            "Small normalized steering bias; positive turns right. "
            "The default is zero so persisted motor calibration remains "
            "unchanged."
        ),
    )
    args = parser.parse_args()

    if args.self_test:
        return run_self_test()
    if not args.enable_motors:
        print("REFUSED: physical run requires --enable-motors.")
        return 1
    if not MINIMUM_DURATION_SECONDS <= args.seconds <= MAXIMUM_DURATION_SECONDS:
        print(
            "REFUSED: --seconds must be between {:.1f} and {:.1f}."
            .format(MINIMUM_DURATION_SECONDS, MAXIMUM_DURATION_SECONDS)
        )
        return 1
    if not -0.10 <= args.steering_bias <= 0.10:
        print("REFUSED: --steering-bias must be within -0.10..+0.10.")
        return 1

    try:
        from validation_manager import (
            GATES_PATH,
            REQUIRED_MOTION_GATES,
            read_json,
            verify_gate_state,
        )

        gate_state = verify_gate_state(read_json(GATES_PATH))
        missing = [
            name
            for name in REQUIRED_MOTION_GATES
            if gate_state.get("gates", {}).get(name, {}).get("passed") is not True
        ]
        if missing or gate_state.get("physical_motion_authorized") is not True:
            raise RuntimeError(
                "physical motion is locked; unresolved/current-integrity gates: {}"
                .format(", ".join(missing) if missing else "authorization false")
            )
    except Exception as error:
        print("REFUSED: current validation authorization failed: {}".format(error))
        print("Run: python validation_manager.py verify")
        return 1

    try:
        import cv2
        from control_core import load_controller_config
        from gst_camera_bridge import GstCamera
        from ipm_lane import IPMLaneDetector
        from motor_mapping import load_motor_config, map_duty
        from safe_motor_output import MotionDeadlineStopper, SafeMotorOutput
    except Exception as error:
        print("REFUSED: Jetson runtime import failed: {}".format(error))
        return 1

    project_directory = os.path.dirname(os.path.abspath(__file__))
    controller_config = dict(load_controller_config())
    controller_config["allow_partial_lane_motion"] = False
    if controller_config["camera_stale_timeout_s"] > CAMERA_STALE_LIMIT_SECONDS:
        print("REFUSED: configured camera stale limit exceeds 0.25 seconds.")
        return 1
    if controller_config["command_lease_s"] > COMMAND_LEASE_SECONDS:
        print("REFUSED: configured command lease exceeds 0.35 seconds.")
        return 1
    motor_config = load_motor_config()
    detector = IPMLaneDetector()
    if detector.config.get("detector_mode") != "SINGLE_LINE_POLYNOMIAL_V1":
        print("REFUSED: ipm_config.json is not in single-line mode.")
        return 1
    if not detector.physically_calibrated:
        print("REFUSED: current IPM/camera geometry is not physically calibrated.")
        return 1

    base_cruise = cruise_normalized_command(motor_config)
    breakaway_base = clamp(
        base_cruise * BREAKAWAY_SPEED_SCALE, 0.0, 1.0
    )
    straight_base = clamp(
        base_cruise * STRAIGHT_SPEED_SCALE, 0.0, 1.0
    )
    expected_duties = {
        "breakaway_left": map_duty(
            breakaway_base, is_right=False, config=motor_config
        ),
        "breakaway_right": map_duty(
            breakaway_base, is_right=True, config=motor_config
        ),
        "straight_left": map_duty(
            straight_base, is_right=False, config=motor_config
        ),
        "straight_right": map_duty(
            straight_base, is_right=True, config=motor_config
        ),
    }

    print("=" * 72)
    print("GUARDED SINGLE STRAIGHT-LINE FOLLOWING TEST")
    print("=" * 72)
    print("Maximum motion time: {:.2f} s".format(args.seconds))
    print("Target: one straight black tape line under the camera centre.")
    print("Detector: {} / {}".format(
        detector.config.get("detector_mode"),
        detector.config.get("target_line_role", "UNLABELLED"),
    ))
    print("Line gate: FULL, confidence >= {:.2f}.".format(
        LINE_CONFIDENCE_MINIMUM
    ))
    print("Camera freshness: <= {:.0f} ms; motor lease: {:.0f} ms.".format(
        CAMERA_STALE_LIMIT_SECONDS * 1000.0,
        COMMAND_LEASE_SECONDS * 1000.0,
    ))
    print("Breakaway PWM: L {:.4f} / R {:.4f}.".format(
        expected_duties["breakaway_left"],
        expected_duties["breakaway_right"],
    ))
    print("Cruise PWM before steering: L {:.4f} / R {:.4f}.".format(
        expected_duties["straight_left"],
        expected_duties["straight_right"],
    ))
    print("Steering bias: {:+.3f} (default is zero).".format(args.steering_bias))
    print("YOLO is NOT loaded in this line-following commissioning test.")
    print("LINE-ONLY COMMISSIONING - OBSTACLE DETECTION IS DISABLED")
    print("The lane must be empty; this run has no automatic obstacle detector.")
    print("No people, stairs, edges, traffic, cables or obstacles in the lane.")
    print("A spotter must hold the physical motor-battery disconnect.")
    print("MOTOR BATTERY MUST BE DISCONNECTED DURING INITIALISATION.")
    print("Ctrl+C, Q/Esc, line/camera/voltage/watchdog/deadline fault => OFF.")
    print("=" * 72)

    camera = None
    motor_output = None
    deadline_stopper = None
    cv2_module = cv2
    atexit.register(emergency_stop)
    for signal_name in ("SIGTERM", "SIGHUP", "SIGQUIT"):
        signal_value = getattr(signal, signal_name, None)
        if signal_value is not None:
            signal.signal(signal_value, signal_stop)

    results_directory = os.path.join(project_directory, "results")
    if not os.path.isdir(results_directory):
        os.makedirs(results_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%S%fZ")
    run_id = "{}_{}".format(safe_label(args.run_label), stamp)
    csv_path = os.path.join(results_directory, run_id + ".csv")
    json_path = os.path.join(results_directory, run_id + ".json")
    fields = (
        "utc", "frame", "camera_sequence", "camera_age_ms",
        "processing_ms", "line_status", "line_confidence", "near_offset",
        "lookahead_offset", "heading_rad", "tracking_error", "p_term",
        "i_term", "steering_feedback", "steering_bias", "steering_total",
        "phase", "left_command", "right_command", "left_duty",
        "right_duty", "battery_voltage_v", "loop_fps",
    )
    rows = []
    stop_reason = "NOT_STARTED"
    completed = False
    started_voltage = None
    ending_voltage = None
    minimum_voltage = None
    motion_started = None
    motion_ended = None
    frame_count = 0
    previous_loop = None
    hardware_off_confirmed = False
    deadline_snapshot = None

    try:
        camera = build_camera(GstCamera)
        print("Initial camera/line health check; do not adjust the fixed camera.")
        _initial_sample, initial_attempts = wait_for_alignment(
            camera,
            detector,
            cv2_module,
            args.headless,
            "INITIAL",
            require_start_alignment=False,
        )
        token = "STRAIGHT-LINE-NO-OBSTACLE-DETECTION"
        if input(
            "Keep motor battery disconnected; type {} to initialise OFF: "
            .format(token)
        ).strip() != token:
            stop_reason = "INITIALISATION_CONFIRMATION_REFUSED"
            print("Cancelled; motor hardware was never opened.")
            return 0

        motor_output = SafeMotorOutput(
            motor_config=motor_config,
            controller_config=controller_config,
        )
        _EMERGENCY_OUTPUT = motor_output
        print("Controller initialised; hardware OFF write confirmed.")
        confirmation = "MOTOR-POWER-CONNECTED-AREA-CLEAR"
        if input(
            "Connect motor power, clear the lane, then type {}: ".format(
                confirmation
            )
        ).strip() != confirmation:
            raise RuntimeError("FINAL_AREA_CONFIRMATION_REFUSED")

        started_voltage = motor_output.read_voltage()
        minimum_voltage = started_voltage
        print(
            "One practical start-position check; motor outputs remain OFF."
        )
        armed_sample, final_attempts = wait_for_alignment(
            camera, detector, cv2_module, args.headless, "FINAL"
        )

        controller = StraightPI()
        motion_started = time.monotonic()
        motion_deadline = (
            motion_started
            + float(args.seconds)
            - DEADLINE_STOP_MARGIN_SECONDS
        )
        deadline_stopper = MotionDeadlineStopper(
            motor_output,
            motion_deadline,
            "STRAIGHT_RUN_HARD_DEADLINE",
        ).start()
        pending_sample = armed_sample
        stop_reason = "RUNNING"
        print("STRAIGHT-LINE RUN STARTED. Ctrl+C or spotter disconnect stops.")

        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            while time.monotonic() < motion_deadline:
                loop_started = time.monotonic()
                used_pending_sample = pending_sample is not None
                if pending_sample is not None:
                    sample = pending_sample
                    pending_sample = None
                    # The final arm sample was processed before the operator-
                    # free transition into motion; re-check its age now.
                    validate_camera_metadata(sample["metadata"])
                    sample["camera_age_s"] = (
                        time.monotonic()
                        - finite(
                            sample["metadata"].get("capture_monotonic"),
                            "camera capture timestamp",
                        )
                    )
                else:
                    sample = capture_observation(camera, detector)

                usable, reason, values = line_is_usable(
                    sample["observation"]
                )
                if not usable:
                    raise RuntimeError(reason)
                # Re-check after all image processing and immediately before
                # computing/publishing the motor command.
                camera_age = validate_camera_metadata(sample["metadata"])

                now = time.monotonic()
                raw_tracking_error = clamp(
                    TRACKING_NEAR_WEIGHT * values["near"]
                    + TRACKING_LOOKAHEAD_WEIGHT * values["lookahead"],
                    -1.0,
                    1.0,
                )
                tracking_error = apply_tracking_deadband(
                    raw_tracking_error
                )
                terms = controller.update(tracking_error, now)
                steering = clamp(
                    terms["output"] + float(args.steering_bias),
                    -STEERING_LIMIT,
                    STEERING_LIMIT,
                )
                elapsed = now - motion_started
                if elapsed < BREAKAWAY_SECONDS:
                    phase = "BREAKAWAY"
                    base = breakaway_base
                else:
                    phase = "CRUISE"
                    base = straight_base
                left_command, right_command = mix_forward(base, steering)
                left_command, right_command = enforce_active_duty_margin(
                    left_command, right_command, motor_config
                )

                # Perception can finish after a loop that began before the
                # deadline.  End cleanly instead of attempting one late
                # command that SafeMotorOutput must reject.
                if time.monotonic() >= motion_deadline:
                    break

                command_result = motor_output.command(
                    left_command,
                    right_command,
                    reason="STRAIGHT_LINE_{}".format(phase),
                    deadline_monotonic=motion_deadline,
                )
                voltage = finite(
                    command_result.get("battery_voltage"),
                    "battery voltage",
                )
                minimum_voltage = min(minimum_voltage, voltage)
                frame_count += 1
                loop_fps = None
                if not used_pending_sample and previous_loop is not None:
                    interval = max(0.000001, loop_started - previous_loop)
                    loop_fps = 1.0 / interval
                previous_loop = (
                    None if used_pending_sample else loop_started
                )
                row = {
                    "utc": utc_now(),
                    "frame": frame_count,
                    "camera_sequence": sample["metadata"].get("sequence"),
                    "camera_age_ms": round(camera_age * 1000.0, 3),
                    "processing_ms": round(sample["processing_ms"], 3),
                    "line_status": values["status"],
                    "line_confidence": round(values["confidence"], 6),
                    "near_offset": round(values["near"], 6),
                    "lookahead_offset": round(values["lookahead"], 6),
                    "heading_rad": round(values["heading"], 6),
                    "tracking_error": round(tracking_error, 6),
                    "p_term": round(terms["p"], 6),
                    "i_term": round(terms["i"], 6),
                    "steering_feedback": round(terms["output"], 6),
                    "steering_bias": round(float(args.steering_bias), 6),
                    "steering_total": round(steering, 6),
                    "phase": phase,
                    "left_command": round(left_command, 6),
                    "right_command": round(right_command, 6),
                    "left_duty": round(command_result["left_duty"], 6),
                    "right_duty": round(command_result["right_duty"], 6),
                    "battery_voltage_v": round(voltage, 6),
                    "loop_fps": (
                        None if loop_fps is None else round(loop_fps, 4)
                    ),
                }
                writer.writerow(row)
                csv_file.flush()
                rows.append(row)
                print(
                    "F{:02d} {} | line {:+.3f}/{:+.3f} conf {:.3f} | "
                    "steer {:+.3f} | PWM L {:.3f} R {:.3f} | {:.3f} V"
                    .format(
                        frame_count,
                        phase,
                        values["near"],
                        values["lookahead"],
                        values["confidence"],
                        steering,
                        command_result["left_duty"],
                        command_result["right_duty"],
                        voltage,
                    )
                )
                if not args.headless:
                    show_sample(cv2_module, sample, "Straight-line run")

        completed = time.monotonic() >= motion_deadline
        stop_reason = "DURATION_COMPLETE" if completed else "LOOP_ENDED"
        motor_output.stop(stop_reason)
        hardware_off_confirmed = bool(
            motor_output.hardware_off_confirmed
        )
        motion_ended = time.monotonic()
        ending_voltage = motor_output.read_voltage()

    except KeyboardInterrupt:
        stop_reason = "EMERGENCY_STOP_CTRL_C_OR_WINDOW"
        print("\nEMERGENCY STOP")
    except Exception as error:
        stop_reason = str(error)
        print("\nSTRAIGHT-LINE RUN STOPPED: {}".format(error))
    finally:
        if motor_output is not None:
            try:
                motor_output.stop(stop_reason)
                hardware_off_confirmed = bool(
                    motor_output.hardware_off_confirmed
                )
            except Exception as error:
                stop_reason = "{}_OFF_ERROR_{!r}".format(stop_reason, error)
                hardware_off_confirmed = False
            if motion_ended is None:
                motion_ended = time.monotonic()
        if deadline_stopper is not None:
            deadline_stopper.cancel()
            deadline_snapshot = deadline_stopper.snapshot()
            if deadline_snapshot.get("stop_error") is not None:
                stop_reason = "{}_DEADLINE_ERROR_{}".format(
                    stop_reason, deadline_snapshot.get("stop_error")
                )
        if motor_output is not None:
            try:
                if ending_voltage is None:
                    ending_voltage = motor_output.read_voltage()
            except Exception:
                pass
            try:
                motor_output.close()
            except Exception as error:
                stop_reason = "{}_CLOSE_ERROR_{!r}".format(stop_reason, error)
        _EMERGENCY_OUTPUT = None
        if camera is not None:
            try:
                camera.close()
            except Exception:
                pass
        try:
            cv2_module.destroyAllWindows()
        except Exception:
            pass

    duration = None
    if motion_started is not None and motion_ended is not None:
        duration = max(0.0, motion_ended - motion_started)
    fps_values = [
        float(row["loop_fps"])
        for row in rows
        if row.get("loop_fps") not in (None, "")
    ]
    result = {
        "schema_version": 1,
        "test": "GUARDED_SINGLE_STRAIGHT_LINE_COMMISSIONING",
        "run_id": run_id,
        "completed_utc": utc_now(),
        "passed": bool(
            completed
            and frame_count > 0
            and stop_reason == "DURATION_COMPLETE"
            and hardware_off_confirmed
            and (
                deadline_snapshot is None
                or deadline_snapshot.get("stop_error") is None
            )
        ),
        "commissioning_only": True,
        "automatic_obstacle_detection": False,
        "obstacle_detection_enabled": False,
        "valid_as_obstacle_avoidance_evidence": False,
        "valid_as_historical_validation_evidence": False,
        "safety_note": (
            "Empty lane and spotter-held physical battery disconnect were "
            "mandatory because YOLO was intentionally not loaded."
        ),
        "requested_seconds": float(args.seconds),
        "motion_duration_s": duration,
        "frames": frame_count,
        "average_control_fps": (
            sum(fps_values) / len(fps_values) if fps_values else None
        ),
        "stop_reason": stop_reason,
        "hardware_off_confirmed": hardware_off_confirmed,
        "deadline_stopper": deadline_snapshot,
        "starting_voltage_v": started_voltage,
        "minimum_voltage_v": minimum_voltage,
        "ending_voltage_v": ending_voltage,
        "steering_bias": float(args.steering_bias),
        "controller": {
            "near_weight": TRACKING_NEAR_WEIGHT,
            "lookahead_weight": TRACKING_LOOKAHEAD_WEIGHT,
            "tracking_error_deadband": TRACKING_ERROR_DEADBAND,
            "kp": STRAIGHT_KP,
            "ki": STRAIGHT_KI,
            "integral_limit": STRAIGHT_INTEGRAL_LIMIT,
            "steering_limit": STEERING_LIMIT,
            "breakaway_scale": BREAKAWAY_SPEED_SCALE,
            "breakaway_seconds": BREAKAWAY_SECONDS,
            "straight_scale": STRAIGHT_SPEED_SCALE,
            "active_duty_margin": ACTIVE_DUTY_MARGIN,
        },
        "camera": {
            "output": [CAMERA_OUTPUT_WIDTH, CAMERA_OUTPUT_HEIGHT],
            "framerate": CAMERA_FRAMERATE,
            "flip_method": CAMERA_FLIP_METHOD,
            "stale_limit_s": CAMERA_STALE_LIMIT_SECONDS,
            "pipeline": (
                camera.pipeline_description
                if camera is not None
                else None
            ),
        },
        "detector_mode": detector.config.get("detector_mode"),
        "target_line_role": detector.config.get("target_line_role"),
        "line_confidence_minimum": LINE_CONFIDENCE_MINIMUM,
        "motor_config_sha256": sha256_file(
            os.path.join(project_directory, "motor_config.json")
        ),
        "ipm_config_sha256": sha256_file(
            os.path.join(project_directory, "ipm_config.json")
        ),
        "runner_sha256": sha256_file(os.path.abspath(__file__)),
        "csv": csv_path if os.path.isfile(csv_path) else None,
    }
    with open(json_path, "w") as output:
        json.dump(result, output, indent=2, sort_keys=True)
        output.write("\n")

    print("\n" + "=" * 72)
    print("STRAIGHT-LINE TEST {}".format("COMPLETE" if result["passed"] else "STOPPED"))
    print("=" * 72)
    print("Frames: {}".format(frame_count))
    print("Motion duration: {}".format(
        "not started" if duration is None else "{:.3f} s".format(duration)
    ))
    print("Stop reason: {}".format(stop_reason))
    if minimum_voltage is not None:
        print("Minimum voltage: {:.3f} V".format(minimum_voltage))
    if result["average_control_fps"] is not None:
        print("Average control rate: {:.2f} FPS".format(
            result["average_control_fps"]
        ))
    if hardware_off_confirmed:
        print("Motor outputs are OFF.")
    else:
        print("MOTOR OUTPUT OFF NOT CONFIRMED - DISCONNECT MOTOR BATTERY NOW.")
    if os.path.isfile(csv_path):
        print("CSV: {}".format(csv_path))
    print("JSON: {}".format(json_path))
    print("=" * 72)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
