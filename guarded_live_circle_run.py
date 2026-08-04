#!/usr/bin/env python3
"""Validation-gated outer-circle commissioning run.

This is not a results/evidence runner.  It requires every persisted validation
gate and its current artifact hashes before the motor adapter can be imported,
then derives continuing drive permission from the current CSI frame, current
single-line observation, current YOLO result and current motor-bus voltage.

The independent physical motor-battery disconnect remains mandatory.  No
software process can guarantee an emergency stop after an OS, I2C or power
failure.
"""

from __future__ import print_function

import argparse
import atexit
import csv
import datetime
import json
import math
import os
import signal
import statistics
import sys
import time

from control_core import (
    PIDController,
    SafetySupervisor,
    clamp,
    cruise_normalized_command,
    load_controller_config,
    mix_forward,
)


MINIMUM_DURATION_SECONDS = 0.5
MAXIMUM_DURATION_SECONDS = 10.0
DEADLINE_STOP_MARGIN_SECONDS = 0.10
# Warm-up has no wall-clock deadline.  Motor commands are unavailable during
# the initial check and remain OFF during the final check, so waiting for two
# consecutive genuinely fresh decisions is safer than rejecting a slow Jetson
# after an arbitrary number of seconds.  Ctrl+C still cancels immediately.
INITIAL_WARMUP_TIMEOUT_SECONDS = None
FINAL_WARMUP_TIMEOUT_SECONDS = None
WARMUP_REQUIRED_CONSECUTIVE_READY_FRAMES = 2
EXPECTED_DETECTOR_MODE = "SINGLE_LINE_POLYNOMIAL_V1"
EXPECTED_TARGET_ROLE = "OUTER_CIRCLE_CENTERLINE"

# The two first physical runs completed only 4.70--4.79 control decisions per
# second and travelled far enough to leave the line after 4--6 motor-command
# frames.  Use the measured 0.60 duty floor as the lower boundary and trade
# straight-line speed for steering authority.  These normalized scales map to
# approximately 66% straight and 65% high-turn PWM with the calibrated motor
# map, while the short breakaway remains above static friction.
COMMISSIONING_BREAKAWAY_SPEED_SCALE = 0.55
COMMISSIONING_STRAIGHT_SPEED_SCALE = 0.40
COMMISSIONING_TURN_SPEED_SCALE = 0.35
COMMISSIONING_BREAKAWAY_SECONDS = 0.20
COMMISSIONING_TURN_THRESHOLD = 0.08

# The persisted PID values were provisional and used only lateral offset.
# This runner uses a P-only physical-commissioning controller plus geometric
# feed-forward from the measured single-line heading.  No values are written
# back to controller_config.json until a complete physical run is reviewed.
COMMISSIONING_KP = 0.55
COMMISSIONING_KI = 0.0
COMMISSIONING_KD = 0.0
COMMISSIONING_STEERING_LIMIT = 0.24
TRACKING_NEAR_WEIGHT = 0.45
TRACKING_LOOKAHEAD_WEIGHT = 0.55
TRACKING_CURVE_FEEDFORWARD = 0.65
TRACKING_HEADING_FEEDFORWARD = 0.25

# Arm only when the tape is close to the vehicle centre and its local heading
# is compatible with a forward start.  One brief LOST event may stop and
# re-arm while stationary; every other perception/safety fault is terminal.
ARMING_MAXIMUM_NEAR_OFFSET = 0.16
ARMING_MAXIMUM_HEADING_RAD = 0.45
MAXIMUM_STATIONARY_REACQUISITIONS = 1
STATIONARY_REACQUISITION_SECONDS = 0.90

# 480x270 preserves the calibrated 16:9 field of view while cutting detector
# pixels by 44%.  Requesting the camera's native 60 fps mode avoids Argus
# selecting its 120 fps mode for the unsupported historical 21 fps request.
CAMERA_OUTPUT_WIDTH = 480
CAMERA_OUTPUT_HEIGHT = 270
CAMERA_CAPTURE_FRAMERATE = 60

_EMERGENCY_MOTOR_OUTPUT = None


def load_runtime_dependencies():
    """Load camera, vision and motor modules only for a physical run.

    Keeping these imports out of the policy self-test makes that test usable
    on a development PC without Jetson/OpenCV packages.  A physical run still
    fails closed if any required runtime module is unavailable.
    """
    global cv2
    global GstCamera, IPMLaneDetector, load_motor_config
    global MotionDeadlineStopper, SafeMotorOutput
    global AsyncYoloWorker, YoloV5Detector, draw_detections
    global evaluate_obstacles, format_detections, mark_path_overlap

    import cv2 as cv2_module
    from gst_camera_bridge import GstCamera as GstCameraClass
    from ipm_lane import IPMLaneDetector as IPMLaneDetectorClass
    from motor_mapping import load_motor_config as load_motor_config_function
    from safe_motor_output import (
        MotionDeadlineStopper as MotionDeadlineStopperClass,
        SafeMotorOutput as SafeMotorOutputClass,
    )
    from yolov5_runtime import (
        AsyncYoloWorker as AsyncYoloWorkerClass,
        YoloV5Detector as YoloV5DetectorClass,
        draw_detections as draw_detections_function,
        evaluate_obstacles as evaluate_obstacles_function,
        format_detections as format_detections_function,
        mark_path_overlap as mark_path_overlap_function,
    )

    cv2 = cv2_module
    GstCamera = GstCameraClass
    IPMLaneDetector = IPMLaneDetectorClass
    load_motor_config = load_motor_config_function
    MotionDeadlineStopper = MotionDeadlineStopperClass
    SafeMotorOutput = SafeMotorOutputClass
    AsyncYoloWorker = AsyncYoloWorkerClass
    YoloV5Detector = YoloV5DetectorClass
    draw_detections = draw_detections_function
    evaluate_obstacles = evaluate_obstacles_function
    format_detections = format_detections_function
    mark_path_overlap = mark_path_overlap_function


class TerminationRequested(Exception):
    pass


def utc_now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def emergency_process_cleanup():
    global _EMERGENCY_MOTOR_OUTPUT
    output = _EMERGENCY_MOTOR_OUTPUT
    if output is not None:
        try:
            output.close()
        except Exception:
            pass


def termination_handler(signum, _frame):
    raise TerminationRequested("SIGNAL_{}".format(signum))


def require_trustworthy_camera_metadata(metadata):
    if (
        metadata is None
        or not metadata.get("timestamp_valid")
        or not metadata.get("source_age_trustworthy")
        or not metadata.get("fresh")
        or metadata.get("capture_monotonic") is None
    ):
        raise RuntimeError(
            "CSI frame lacks a fresh, pipeline-clock source timestamp."
        )


def yolo_is_current(yolo_state, config, current_frame_number=None,
                    minimum_source_frame=None):
    if not yolo_state.get("has_result") or yolo_state.get("error"):
        return False
    age = yolo_state.get("age_seconds")
    lag = yolo_state.get("source_frame_lag")
    source_frame = yolo_state.get("source_frame")
    try:
        age = float(age)
        lag = int(lag)
        source_frame = int(source_frame)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(age) or age < 0.0:
        return False
    if age > float(config["yolo_stale_timeout_s"]):
        return False
    if lag < 0 or lag > int(config["yolo_max_frame_lag"]):
        return False
    if minimum_source_frame is not None and source_frame < int(
        minimum_source_frame
    ):
        return False
    if current_frame_number is not None and source_frame > int(
        current_frame_number
    ):
        return False
    return True


def finite_observation_value(observation, key):
    value = observation.get(key)
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise RuntimeError("Single-line observation missing {}.".format(key))
    if not math.isfinite(value):
        raise RuntimeError("Single-line observation {} is not finite.".format(key))
    return value


def rounded_optional(value, digits=6):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return round(value, int(digits))


def line_alignment_ready(observation):
    """Return whether a FULL line is positioned safely for motor arming."""
    if observation.get("status") != "FULL":
        return False, "LINE_{}".format(observation.get("status"))
    try:
        near_offset = finite_observation_value(
            observation, "near_field_offset"
        )
        heading = finite_observation_value(observation, "heading_error_rad")
    except RuntimeError as error:
        return False, str(error)
    if abs(near_offset) > ARMING_MAXIMUM_NEAR_OFFSET:
        return False, "LINE_NOT_CENTRED_FOR_ARMING"
    if abs(heading) > ARMING_MAXIMUM_HEADING_RAD:
        return False, "LINE_HEADING_TOO_LARGE_FOR_ARMING"
    return True, "LINE_ALIGNED"


def single_line_steering(pid, observation, timestamp=None):
    """Fuse near position, lookahead position and heading into steering.

    Positive image offset/heading means that the tape bends to the right.
    ``mix_forward`` uses the same sign: positive steering speeds the logical
    left motor and slows the logical right motor.
    """
    near_offset = finite_observation_value(observation, "near_field_offset")
    lookahead_offset = finite_observation_value(
        observation, "lookahead_offset"
    )
    heading = finite_observation_value(observation, "heading_error_rad")
    tracking_error = clamp(
        TRACKING_NEAR_WEIGHT * near_offset
        + TRACKING_LOOKAHEAD_WEIGHT * lookahead_offset,
        -1.0,
        1.0,
    )
    terms = pid.update(tracking_error, timestamp=timestamp)
    feedforward = (
        TRACKING_CURVE_FEEDFORWARD * (lookahead_offset - near_offset)
        + TRACKING_HEADING_FEEDFORWARD * heading
    )
    steering = clamp(
        float(terms["output"]) + feedforward,
        -COMMISSIONING_STEERING_LIMIT,
        COMMISSIONING_STEERING_LIMIT,
    )
    terms.update({
        "tracking_error": float(tracking_error),
        "near_offset": float(near_offset),
        "lookahead_offset": float(lookahead_offset),
        "heading_error_rad": float(heading),
        "feedforward": float(feedforward),
        "steering": float(steering),
    })
    return terms


def line_loss_reacquisition_allowed(fault_reason, sample, config):
    """Permit only a pure line-segmentation loss while all other gates pass."""
    if fault_reason != "LINE_LOST":
        return False
    try:
        camera_age = float(sample["camera_age"])
    except (KeyError, TypeError, ValueError):
        return False
    if (
        not math.isfinite(camera_age)
        or camera_age < 0.0
        or camera_age > float(config["camera_stale_timeout_s"])
    ):
        return False
    yolo_state = sample.get("yolo_state") or {}
    if not sample.get("yolo_current") or yolo_state.get("error"):
        return False
    obstacle = sample.get("obstacle") or {}
    return obstacle.get("state") == "DRIVE"


def live_observation(camera, lane_detector, worker, safety, config,
                     minimum_yolo_source_frame=None,
                     camera_read_timeout_seconds=2.0):
    """Return one current-frame decision without touching motor hardware."""
    success, frame, metadata = camera.read_with_metadata(
        timeout_seconds=max(0.01, float(camera_read_timeout_seconds))
    )
    if not success:
        raise RuntimeError("CSI frame unavailable.")
    require_trustworthy_camera_metadata(metadata)
    source_frame = int(metadata["sequence"])
    # Start GPU inference before CPU/IPM work so the two pipelines overlap.
    # The previous order serialized them and contributed to the measured
    # 4.70--4.79 Hz control loop.
    worker.submit(
        frame,
        source_frame,
        capture_monotonic=metadata["capture_monotonic"],
        source_pts_ns=metadata["pts_ns"],
    )
    observation = lane_detector.observe(frame)
    yolo_state = worker.snapshot(current_frame_number=source_frame)
    current_yolo = yolo_is_current(
        yolo_state,
        config,
        current_frame_number=source_frame,
        minimum_source_frame=minimum_yolo_source_frame,
    )
    if current_yolo:
        detections = mark_path_overlap(
            yolo_state["detections"], observation["source_polygon"]
        )
        obstacle = evaluate_obstacles(detections)
    else:
        detections = []
        obstacle = {"state": "DRIVE", "reason": "NO_FRESH_RESULT"}

    camera_age = time.monotonic() - metadata["capture_monotonic"]
    # SafetySupervisor checks camera age, line status/confidence, YOLO age and
    # lag, and the obstacle action.  Partial-line motion is disabled in the
    # copied runtime config before this function is called.
    decision = safety.evaluate(
        observation["status"],
        observation["confidence"],
        camera_age,
        yolo_state,
        obstacle,
    )
    # Enforce independent fault precedence.  A lost line must never mask a
    # simultaneous stale camera, non-current/error YOLO result, or obstacle
    # STOP and thereby become eligible for stationary line reacquisition.
    if camera_age < 0.0:
        decision = safety.stop("CAMERA_TIMESTAMP_FUTURE")
    elif camera_age > float(config["camera_stale_timeout_s"]):
        decision = safety.stop("CAMERA_STALE")
    elif yolo_state.get("error"):
        decision = safety.stop("YOLO_WORKER_ERROR")
    elif not current_yolo:
        decision = safety.stop("YOLO_NOT_CURRENT")
    elif obstacle["state"] == "STOP":
        decision = safety.stop(obstacle["reason"])
    elif observation["status"] != "FULL":
        decision = safety.stop("LINE_{}".format(observation["status"]))
    elif float(observation["confidence"]) < float(
        config["lane_confidence_minimum"]
    ):
        decision = safety.stop("LINE_CONFIDENCE_LOW")

    return {
        "frame": frame,
        "metadata": metadata,
        "source_frame": source_frame,
        "observation": observation,
        "yolo_state": yolo_state,
        "yolo_current": current_yolo,
        "detections": detections,
        "obstacle": obstacle,
        "camera_age": camera_age,
        "decision": decision,
    }


def warm_live_perception(camera, lane_detector, worker, config,
                         timeout_seconds, minimum_yolo_source_frame=None):
    """Require consecutive current live DRIVE decisions before motor I/O."""
    if timeout_seconds is None or float(timeout_seconds) <= 0.0:
        deadline = None
    else:
        deadline = time.monotonic() + float(timeout_seconds)
    safety = SafetySupervisor(config)
    ready_streak = 0
    attempts = 0
    last = None
    last_print = 0.0
    while deadline is None or time.monotonic() < deadline:
        attempts += 1
        if deadline is None:
            read_timeout = 2.0
        else:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            read_timeout = min(2.0, remaining)
        last = live_observation(
            camera,
            lane_detector,
            worker,
            safety,
            config,
            minimum_yolo_source_frame=minimum_yolo_source_frame,
            camera_read_timeout_seconds=read_timeout,
        )
        # The camera read and CPU detector also consume time.  Never accept a
        # recovery sample after the bounded stationary window has expired.
        if deadline is not None and time.monotonic() >= deadline:
            break
        aligned, alignment_reason = line_alignment_ready(last["observation"])
        if last["decision"]["state"] == "DRIVE" and aligned:
            ready_streak += 1
        else:
            ready_streak = 0
        now = time.monotonic()
        if now - last_print >= 1.0:
            yolo_age = last["yolo_state"].get("age_seconds")
            print(
                "Live check | line {} conf {:.3f} | camera {:.1f} ms | "
                "YOLO {} ms | {} ({}/{})".format(
                    last["observation"]["status"],
                    float(last["observation"]["confidence"]),
                    last["camera_age"] * 1000.0,
                    "none" if yolo_age is None else "{:.1f}".format(
                        float(yolo_age) * 1000.0
                    ),
                    (
                        last["decision"]["reason"]
                        if last["decision"]["state"] != "DRIVE"
                        else alignment_reason
                    ),
                    ready_streak,
                    WARMUP_REQUIRED_CONSECUTIVE_READY_FRAMES,
                )
            )
            last_print = now
        if ready_streak >= WARMUP_REQUIRED_CONSECUTIVE_READY_FRAMES:
            return (
                int(last["yolo_state"]["source_frame"]),
                attempts,
                last,
            )
        time.sleep(0.005)

    if last is None:
        detail = "no camera observation"
    else:
        detail = (
            "line={} confidence={:.3f} camera_age_ms={:.1f} "
            "yolo_age_s={} yolo_lag={} reason={}"
        ).format(
            last["observation"]["status"],
            float(last["observation"]["confidence"]),
            last["camera_age"] * 1000.0,
            last["yolo_state"].get("age_seconds"),
            last["yolo_state"].get("source_frame_lag"),
            last["decision"]["reason"],
        )
    raise RuntimeError(
        "Live perception did not become ready within {:.1f} s: {}."
        .format(float(timeout_seconds), detail)
    )


def run_policy_self_test():
    config = dict(load_controller_config())
    config["allow_partial_lane_motion"] = False
    config["kp"] = COMMISSIONING_KP
    config["ki"] = COMMISSIONING_KI
    config["kd"] = COMMISSIONING_KD
    config["steering_limit"] = COMMISSIONING_STEERING_LIMIT
    safety = SafetySupervisor(config)
    good_yolo = {
        "has_result": True,
        "error": None,
        "age_seconds": 0.05,
        "source_frame_lag": 1,
        "source_frame": 10,
    }
    clear = {"state": "DRIVE", "reason": "CLEAR"}
    assert safety.evaluate("FULL", 0.90, 0.02, good_yolo, clear)[
        "state"
    ] == "DRIVE"
    for bad_status in ("LOST", "AMBIGUOUS", "INVALID_WIDTH", "PARTIAL_LEFT"):
        assert SafetySupervisor(config).evaluate(
            bad_status, 0.90, 0.02, good_yolo, clear
        )["state"] == "STOP"
    assert SafetySupervisor(config).evaluate(
        "FULL", 0.10, 0.02, good_yolo, clear
    )["state"] == "STOP"
    assert SafetySupervisor(config).evaluate(
        "FULL", 0.90, config["camera_stale_timeout_s"] + 0.01,
        good_yolo, clear
    )["state"] == "STOP"
    stale_yolo = dict(good_yolo)
    stale_yolo["age_seconds"] = config["yolo_stale_timeout_s"] + 0.01
    assert SafetySupervisor(config).evaluate(
        "FULL", 0.90, 0.02, stale_yolo, clear
    )["state"] == "STOP"
    obstacle = {"state": "STOP", "reason": "PERSON_IN_PATH"}
    assert SafetySupervisor(config).evaluate(
        "FULL", 0.90, 0.02, good_yolo, obstacle
    )["state"] == "STOP"
    assert yolo_is_current(good_yolo, config, current_frame_number=11)
    bad_lag_yolo = dict(good_yolo)
    bad_lag_yolo["source_frame_lag"] = config["yolo_max_frame_lag"] + 1
    assert not yolo_is_current(
        bad_lag_yolo, config, current_frame_number=20
    )
    assert not yolo_is_current(
        stale_yolo, config, current_frame_number=11
    )
    centred = {
        "status": "FULL",
        "near_field_offset": 0.0,
        "lookahead_offset": 0.0,
        "heading_error_rad": 0.0,
    }
    right_curve = dict(
        centred,
        lookahead_offset=0.12,
        heading_error_rad=0.20,
    )
    left_curve = dict(
        centred,
        lookahead_offset=-0.12,
        heading_error_rad=-0.20,
    )
    assert line_alignment_ready(centred)[0]
    off_centre = dict(centred, near_field_offset=0.25)
    assert not line_alignment_ready(off_centre)[0]
    right_terms = single_line_steering(
        PIDController(config), right_curve, timestamp=1.0
    )
    left_terms = single_line_steering(
        PIDController(config), left_curve, timestamp=1.0
    )
    assert right_terms["steering"] > 0.0
    assert left_terms["steering"] < 0.0
    left_command, right_command = mix_forward(
        0.15, right_terms["steering"]
    )
    assert left_command > right_command > 0.02
    pure_line_loss = {
        "camera_age": 0.10,
        "yolo_current": True,
        "yolo_state": {"error": None},
        "obstacle": {"state": "DRIVE"},
    }
    assert line_loss_reacquisition_allowed(
        "LINE_LOST", pure_line_loss, config
    )
    stale_camera_loss = dict(pure_line_loss, camera_age=1.0)
    assert not line_loss_reacquisition_allowed(
        "LINE_LOST", stale_camera_loss, config
    )
    stale_yolo_loss = dict(pure_line_loss, yolo_current=False)
    assert not line_loss_reacquisition_allowed(
        "LINE_LOST", stale_yolo_loss, config
    )
    obstacle_loss = dict(
        pure_line_loss, obstacle={"state": "STOP"}
    )
    assert not line_loss_reacquisition_allowed(
        "LINE_LOST", obstacle_loss, config
    )
    assert MINIMUM_DURATION_SECONDS == 0.5
    assert MAXIMUM_DURATION_SECONDS == 10.0
    assert INITIAL_WARMUP_TIMEOUT_SECONDS is None
    assert FINAL_WARMUP_TIMEOUT_SECONDS is None
    assert config["camera_stale_timeout_s"] <= 0.25
    assert config["yolo_stale_timeout_s"] <= 0.75
    assert config["command_lease_s"] <= 0.35
    assert (
        0.0 < COMMISSIONING_TURN_SPEED_SCALE
        <= COMMISSIONING_STRAIGHT_SPEED_SCALE
        < COMMISSIONING_BREAKAWAY_SPEED_SCALE
        < 1.0
    )
    assert MAXIMUM_STATIONARY_REACQUISITIONS == 1
    print("PASS live FULL/confidence gate")
    print("PASS camera and YOLO freshness gates")
    print("PASS obstacle STOP")
    print("PASS single-line curve steering sign and motor mixing")
    print("PASS centred arming and fault-isolated stationary reacquisition")
    print("PASS unlimited output-OFF perception warm-up")
    print("PASS persisted camera, YOLO and command-lease safety limits")
    print("PASS duration cap 0.5..10.0 seconds")
    print("GUARDED LIVE CIRCLE POLICY SELF-TEST: PASS")
    return 0


def main():
    global _EMERGENCY_MOTOR_OUTPUT
    parser = argparse.ArgumentParser(
        description="Live-only guarded outer-circle commissioning run"
    )
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--run-label", default="live_circle")
    parser.add_argument("--self-test", action="store_true")
    arguments = parser.parse_args()

    if arguments.self_test:
        return run_policy_self_test()
    if not arguments.enable_motors:
        print("REFUSED: --enable-motors is required for physical motion.")
        return 1
    if not MINIMUM_DURATION_SECONDS <= arguments.seconds <= MAXIMUM_DURATION_SECONDS:
        print(
            "REFUSED: --seconds must be between {:.1f} and {:.1f}.".format(
                MINIMUM_DURATION_SECONDS, MAXIMUM_DURATION_SECONDS
            )
        )
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
        load_runtime_dependencies()
    except Exception as error:
        print("REFUSED: required Jetson runtime dependency failed: {}".format(error))
        return 1

    controller_config = dict(load_controller_config())
    # This runner permits only FULL line observations regardless of a local
    # configuration value.  Do not alter the persisted file.
    controller_config["allow_partial_lane_motion"] = False
    controller_config["kp"] = COMMISSIONING_KP
    controller_config["ki"] = COMMISSIONING_KI
    controller_config["kd"] = COMMISSIONING_KD
    controller_config["steering_limit"] = COMMISSIONING_STEERING_LIMIT
    motor_config = load_motor_config()
    lane_detector = IPMLaneDetector()
    if lane_detector.config.get("detector_mode") != EXPECTED_DETECTOR_MODE:
        print("REFUSED: current detector is not the outer-circle single-line detector.")
        return 1
    if lane_detector.config.get("target_line_role") != EXPECTED_TARGET_ROLE:
        print("REFUSED: target line is not the outer-circle centreline.")
        return 1
    if not lane_detector.physically_calibrated:
        print("REFUSED: current IPM geometry is not physically calibrated.")
        return 1

    pid = PIDController(controller_config)
    safety = SafetySupervisor(controller_config)
    cruise_command = cruise_normalized_command(motor_config)
    detector = YoloV5Detector(
        image_size=controller_config["yolo_image_size"],
        confidence=controller_config["yolo_confidence_threshold"],
        iou=controller_config["yolo_iou_threshold"],
    )
    if detector.device.type != "cuda":
        print("REFUSED: live physical motion requires CUDA YOLO inference.")
        return 1
    worker = AsyncYoloWorker(detector)
    worker.start()
    camera = GstCamera(
        sensor_id=0,
        capture_width=1280,
        capture_height=720,
        output_width=CAMERA_OUTPUT_WIDTH,
        output_height=CAMERA_OUTPUT_HEIGHT,
        framerate=CAMERA_CAPTURE_FRAMERATE,
    )

    atexit.register(emergency_process_cleanup)
    for signal_name in ("SIGTERM", "SIGHUP", "SIGQUIT"):
        signal_value = getattr(signal, signal_name, None)
        if signal_value is not None:
            signal.signal(signal_value, termination_handler)

    try:
        print(
            "Checking current camera, outer line and YOLO with no time limit; "
            "motors unavailable. Ctrl+C cancels."
        )
        warmed_yolo_frame, initial_attempts, _initial_sample = (
            warm_live_perception(
                camera,
                lane_detector,
                worker,
                controller_config,
                INITIAL_WARMUP_TIMEOUT_SECONDS,
            )
        )
    except KeyboardInterrupt:
        camera.close()
        worker.stop()
        print("\nCancelled during output-OFF live perception check.")
        return 130
    except Exception as error:
        camera.close()
        worker.stop()
        print("REFUSED: live perception check failed: {}".format(error))
        return 1

    print("=" * 70)
    print("GUARDED LIVE OUTER-CIRCLE COMMISSIONING RUN")
    print("Maximum motion time: {:.2f} seconds".format(arguments.seconds))
    print("Validation authorization: CURRENT / REQUIRED")
    print("Result status: COMMISSIONING ONLY, NOT VALIDATION EVIDENCE")
    print("Controller state: {}".format(
        controller_config["verification_state"]
    ))
    print("Motion requires live FULL line and confidence >= {:.2f}.".format(
        controller_config["lane_confidence_minimum"]
    ))
    print("Camera/line/YOLO/obstacle/voltage/watchdog/deadline fault => STOP.")
    print(
        "Validated limits: camera <= {:.0f} ms, YOLO <= {:.0f} ms, "
        "command lease {:.0f} ms.".format(
            controller_config["camera_stale_timeout_s"] * 1000.0,
            controller_config["yolo_stale_timeout_s"] * 1000.0,
            controller_config["command_lease_s"] * 1000.0,
        )
    )
    print(
        "Commissioning speed scales: breakaway {:.0f}%, straight {:.0f}%, "
        "turn {:.0f}%.".format(
            COMMISSIONING_BREAKAWAY_SPEED_SCALE * 100.0,
            COMMISSIONING_STRAIGHT_SPEED_SCALE * 100.0,
            COMMISSIONING_TURN_SPEED_SCALE * 100.0,
        )
    )
    print(
        "Curve steering: near/lookahead/heading fusion, P-only Kp {:.2f}, "
        "limit {:.2f}.".format(COMMISSIONING_KP, COMMISSIONING_STEERING_LIMIT)
    )
    print(
        "One LINE_LOST event may re-arm only while outputs are OFF for "
        "at most {:.2f} s.".format(STATIONARY_REACQUISITION_SECONDS)
    )
    print("Use only the OUTER circle tape; completely mask/remove inner tape.")
    print(
        "Start on the flattest part: tape under camera/vehicle centre and "
        "robot pointing tangent (parallel) to the tape."
    )
    print("Closed flat area only. No people, stairs, edges or traffic.")
    print("A spotter must hold the motor-battery disconnect throughout.")
    print("MOTOR BATTERY MUST BE DISCONNECTED DURING INITIALISATION.")
    print("=" * 70)
    token = "LIVE-CIRCLE-{:.1f}".format(arguments.seconds)
    if input("Type {} to initialise OFF: ".format(token)).strip() != token:
        camera.close()
        worker.stop()
        print("Cancelled; motor hardware was never opened.")
        return 0

    project_directory = os.path.dirname(os.path.abspath(__file__))
    results_directory = os.path.join(project_directory, "results")
    if not os.path.isdir(results_directory):
        os.makedirs(results_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    safe_label = "".join(
        char if char.isalnum() or char in "-_" else "_"
        for char in arguments.run_label
    )[:40]
    run_id = "{}_{}".format(safe_label or "live_circle", stamp)
    csv_path = os.path.join(results_directory, run_id + ".csv")
    json_path = os.path.join(results_directory, run_id + ".json")
    fields = (
        "timestamp_utc", "frame", "camera_sequence", "camera_age_ms",
        "line_status", "line_confidence", "line_offset",
        "near_field_offset", "lookahead_offset", "heading_error_rad",
        "tracking_error", "pid_output", "curve_feedforward",
        "steering_command", "speed_scale", "yolo_age_s", "yolo_frame_lag",
        "objects", "obstacle_state", "decision", "decision_reason",
        "stationary_reacquisition_count", "left_command", "right_command",
        "left_duty", "right_duty", "battery_voltage_v", "control_fps",
    )
    csv_file = open(csv_path, "w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=fields)
    writer.writeheader()

    motor_output = None
    deadline_stopper = None
    motion_deadline = None
    run_start = None
    measurement_end = None
    frame_count = 0
    motion_frames = 0
    stop_reason = None
    completed_duration = False
    starting_voltage = None
    ending_voltage = None
    voltages = []
    fps_values = []
    previous_timestamp = None
    final_attempts = 0
    stationary_reacquisition_count = 0
    stationary_reacquisition_attempts = 0
    stationary_reacquisition_error = None

    try:
        motor_output = SafeMotorOutput(
            motor_config=motor_config,
            controller_config=controller_config,
        )
        _EMERGENCY_MOTOR_OUTPUT = motor_output
        print("Controller initialised; hardware OFF write confirmed.")
        print(
            "Place the car on the flattest outer-tape section: tape under "
            "camera/vehicle centre, robot body tangent to the tape."
        )
        confirmation = "MOTOR-POWER-CONNECTED-AREA-CLEAR"
        if input("Connect motor power, clear the area, then type {}: ".format(
            confirmation
        )).strip() != confirmation:
            raise RuntimeError("Final motor-power/area-clear confirmation refused.")

        # Read voltage before the final perception arm.  No I2C work or
        # operator pause is then inserted between the accepted live sample
        # and the first control decision.
        starting_voltage = motor_output.read_voltage()
        voltages.append(starting_voltage)
        print(
            "Final live recheck with no time limit; motor outputs remain OFF. "
            "Ctrl+C cancels."
        )
        warmed_yolo_frame, final_attempts, armed_sample = warm_live_perception(
            camera,
            lane_detector,
            worker,
            controller_config,
            FINAL_WARMUP_TIMEOUT_SECONDS,
            minimum_yolo_source_frame=warmed_yolo_frame + 1,
        )
        run_start = time.monotonic()
        run_deadline = run_start + float(arguments.seconds)
        motion_deadline = run_deadline - DEADLINE_STOP_MARGIN_SECONDS
        deadline_stopper = MotionDeadlineStopper(
            motor_output, motion_deadline, "LIVE_RUN_HARD_DEADLINE"
        ).start()
        print("LIVE RUN STARTED. Ctrl+C or spotter disconnect stops immediately.")

        pending_sample = armed_sample
        while time.monotonic() < motion_deadline:
            # The final arm already produced a current FULL/clear sample.
            # Consume it immediately instead of discarding it and waiting for
            # another frame that may be stale under GPU contention.
            if pending_sample is not None:
                sample = pending_sample
                pending_sample = None
            else:
                sample = live_observation(
                    camera, lane_detector, worker, safety, controller_config
                )
            frame_count += 1
            decision = sample["decision"]
            observation = sample["observation"]
            yolo_state = sample["yolo_state"]
            stationary_reacquisition_requested = False
            speed_scale = 0.0
            if decision["state"] != "DRIVE":
                fault_reason = decision["reason"]
                pid.reset()
                motor_output.stop(fault_reason)
                left_command = right_command = 0.0
                left_duty = right_duty = 0.0
                voltage = motor_output.read_voltage()
                terms = {
                    "output": 0.0,
                    "tracking_error": 0.0,
                    "near_offset": observation.get("near_field_offset"),
                    "lookahead_offset": observation.get("lookahead_offset"),
                    "heading_error_rad": observation.get("heading_error_rad"),
                    "feedforward": 0.0,
                    "steering": 0.0,
                }
                # Do not drive on stale geometry.  A single segmentation
                # flicker may be rechecked only after hardware OFF is
                # confirmed.  Ambiguous/invalid line, stale camera/YOLO,
                # obstacle, voltage and watchdog faults remain terminal.
                if (
                    stationary_reacquisition_count
                    < MAXIMUM_STATIONARY_REACQUISITIONS
                    and line_loss_reacquisition_allowed(
                        fault_reason, sample, controller_config
                    )
                    and time.monotonic() < motion_deadline
                ):
                    stationary_reacquisition_requested = True
                else:
                    stop_reason = fault_reason
            else:
                terms = single_line_steering(pid, observation)
                elapsed_motion = time.monotonic() - run_start
                if elapsed_motion < COMMISSIONING_BREAKAWAY_SECONDS:
                    speed_scale = COMMISSIONING_BREAKAWAY_SPEED_SCALE
                elif abs(float(terms["steering"])) >= (
                    COMMISSIONING_TURN_THRESHOLD
                ):
                    speed_scale = COMMISSIONING_TURN_SPEED_SCALE
                else:
                    speed_scale = COMMISSIONING_STRAIGHT_SPEED_SCALE
                base_command = cruise_command * speed_scale
                left_command, right_command = mix_forward(
                    base_command, terms["steering"]
                )
                if time.monotonic() >= motion_deadline:
                    break
                applied = motor_output.command(
                    left_command,
                    right_command,
                    reason="LIVE_FULL_LINE",
                    deadline_monotonic=motion_deadline,
                )
                left_duty = applied["left_duty"]
                right_duty = applied["right_duty"]
                voltage = applied["battery_voltage"]
                if max(left_duty, right_duty) > 0.0:
                    motion_frames += 1
            voltages.append(voltage)

            now = time.monotonic()
            if previous_timestamp is None:
                control_fps = 0.0
            else:
                control_fps = 1.0 / max(0.000001, now - previous_timestamp)
                fps_values.append(control_fps)
            previous_timestamp = now
            writer.writerow({
                "timestamp_utc": utc_now(),
                "frame": frame_count,
                "camera_sequence": sample["metadata"]["sequence"],
                "camera_age_ms": round(sample["camera_age"] * 1000.0, 3),
                "line_status": observation["status"],
                "line_confidence": round(float(observation["confidence"]), 4),
                "line_offset": round(float(observation["lane_offset"]), 6),
                "near_field_offset": rounded_optional(
                    observation.get("near_field_offset")
                ),
                "lookahead_offset": rounded_optional(
                    observation.get("lookahead_offset")
                ),
                "heading_error_rad": rounded_optional(
                    observation.get("heading_error_rad")
                ),
                "tracking_error": rounded_optional(
                    terms.get("tracking_error")
                ),
                "pid_output": rounded_optional(terms.get("output")),
                "curve_feedforward": rounded_optional(
                    terms.get("feedforward")
                ),
                "steering_command": rounded_optional(terms.get("steering")),
                "speed_scale": round(float(speed_scale), 4),
                "yolo_age_s": yolo_state.get("age_seconds"),
                "yolo_frame_lag": yolo_state.get("source_frame_lag"),
                "objects": format_detections(sample["detections"]),
                "obstacle_state": sample["obstacle"]["state"],
                "decision": decision["state"],
                "decision_reason": decision["reason"],
                "stationary_reacquisition_count": (
                    stationary_reacquisition_count
                ),
                "left_command": round(float(left_command), 6),
                "right_command": round(float(right_command), 6),
                "left_duty": round(float(left_duty), 6),
                "right_duty": round(float(right_duty), 6),
                "battery_voltage_v": round(float(voltage), 4),
                "control_fps": round(float(control_fps), 3),
            })
            csv_file.flush()

            if not arguments.headless:
                overlay = observation["original_overlay"]
                if yolo_state.get("has_result"):
                    draw_detections(
                        overlay,
                        sample["detections"],
                        yolo_state.get("corridor"),
                        path_polygon=observation["source_polygon"],
                    )
                cv2.putText(
                    overlay,
                    "{} {}".format(decision["state"], decision["reason"]),
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                    (0, 255, 0) if decision["state"] == "DRIVE" else (0, 0, 255),
                    2,
                )
                cv2.imshow("Guarded live outer-circle run", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    stop_reason = "OPERATOR_Q_STOP"
                    motor_output.stop(stop_reason)

            if stationary_reacquisition_requested and stop_reason is None:
                stationary_reacquisition_attempts += 1
                print(
                    "LINE LOST: outputs are OFF; attempting one stationary "
                    "reacquisition for at most {:.2f} s.".format(
                        STATIONARY_REACQUISITION_SECONDS
                    )
                )
                remaining = max(0.0, motion_deadline - time.monotonic())
                reacquisition_timeout = min(
                    STATIONARY_REACQUISITION_SECONDS, remaining
                )
                try:
                    if reacquisition_timeout <= 0.0:
                        raise RuntimeError("Motion deadline reached.")
                    _recovered_yolo, _attempts, recovered_sample = (
                        warm_live_perception(
                            camera,
                            lane_detector,
                            worker,
                            controller_config,
                            reacquisition_timeout,
                            minimum_yolo_source_frame=(
                                int(sample["source_frame"]) + 1
                            ),
                        )
                    )
                    if (
                        time.monotonic() >= motion_deadline
                        or deadline_stopper.triggered
                    ):
                        raise RuntimeError("Motion deadline reached.")
                    stationary_reacquisition_count += 1
                    stationary_reacquisition_error = None
                    pending_sample = recovered_sample
                    pid.reset()
                    previous_timestamp = None
                    print(
                        "LINE REACQUIRED while stationary; resuming at the "
                        "bounded turn-speed command."
                    )
                    continue
                except Exception as error:
                    stationary_reacquisition_error = repr(error)
                    stop_reason = "LINE_LOST_REACQUISITION_FAILED"
                    motor_output.stop(stop_reason)
                    print(
                        "LINE REACQUISITION FAILED; outputs remain OFF: {}"
                        .format(error)
                    )

            # Every terminal fault or operator request ends the run.
            if stop_reason is not None:
                break

        if stop_reason is None:
            completed_duration = True
            motor_output.stop("LIVE_DURATION_COMPLETE")
    except KeyboardInterrupt:
        stop_reason = "EMERGENCY_STOP_CTRL_C"
        if motor_output is not None:
            try:
                motor_output.stop(stop_reason)
            except Exception:
                print("SPOTTER: DISCONNECT MOTOR POWER NOW.")
        print("\nEMERGENCY STOP")
    except TerminationRequested as error:
        stop_reason = str(error)
        if motor_output is not None:
            try:
                motor_output.stop(stop_reason)
            except Exception:
                print("SPOTTER: DISCONNECT MOTOR POWER NOW.")
        print("\nEMERGENCY STOP: {}".format(stop_reason))
    except Exception as error:
        stop_reason = "LIVE_RUN_EXCEPTION: {!r}".format(error)
        if motor_output is not None:
            try:
                motor_output.stop("LIVE_RUN_EXCEPTION")
            except Exception:
                print("SPOTTER: DISCONNECT MOTOR POWER NOW.")
        print("LIVE RUN STOPPED: {}".format(error))
    finally:
        if motor_output is not None:
            try:
                motor_output.stop("LIVE_RUN_FINALIZATION")
                time.sleep(0.02)
                ending_voltage = motor_output.read_voltage()
                voltages.append(ending_voltage)
            except Exception as error:
                print("FINAL MOTOR/VOLTAGE ERROR: {}".format(error))
                if stop_reason is None:
                    stop_reason = "FINAL_MOTOR_ERROR: {!r}".format(error)
            try:
                motor_output.close()
            except Exception as error:
                print("MOTOR CLOSE ERROR: {}".format(error))
                if stop_reason is None:
                    stop_reason = "MOTOR_CLOSE_ERROR: {!r}".format(error)
        if deadline_stopper is not None:
            deadline_stopper.cancel()
            if deadline_stopper.stop_error is not None and stop_reason is None:
                stop_reason = "DEADLINE_STOP_ERROR: {}".format(
                    deadline_stopper.stop_error
                )
        measurement_end = time.monotonic()
        _EMERGENCY_MOTOR_OUTPUT = None
        try:
            camera.close()
        except Exception as error:
            print("CAMERA CLOSE ERROR: {}".format(error))
        try:
            worker.stop()
        except Exception as error:
            print("YOLO WORKER CLOSE ERROR: {}".format(error))
        csv_file.close()
        cv2.destroyAllWindows()

    elapsed = 0.0 if run_start is None else max(
        0.0, measurement_end - run_start
    )
    watchdog_fault = None if motor_output is None else motor_output.watchdog_fault
    watchdog_trips = 0 if motor_output is None else motor_output.watchdog_trip_count
    energized_duration = (
        0.0 if motor_output is None
        else float(motor_output.total_energized_duration_s)
    )
    hardware_off = bool(
        motor_output is not None
        and motor_output.hardware_off_confirmed
        and not motor_output.hardware_off_retry_required
    )
    successful = bool(
        completed_duration
        and stop_reason is None
        and frame_count > 0
        and motion_frames > 0
        and watchdog_fault is None
        and watchdog_trips == 0
        and hardware_off
        and energized_duration <= float(arguments.seconds)
        and (
            deadline_stopper is None
            or deadline_stopper.stop_error is None
        )
    )
    summary = {
        "schema_version": 2,
        "test": "GUARDED_LIVE_OUTER_CIRCLE_COMMISSIONING_V2",
        "commissioning_only": True,
        "valid_as_historical_validation_evidence": False,
        "historical_validation_gates_checked": True,
        "run_id": run_id,
        "successful": successful,
        "completed_guarded_motion_window": completed_duration,
        "stop_reason": stop_reason,
        "requested_maximum_duration_s": float(arguments.seconds),
        "effective_guarded_motion_window_s": max(
            0.0,
            float(arguments.seconds) - DEADLINE_STOP_MARGIN_SECONDS,
        ),
        "elapsed_s": elapsed,
        "frames": frame_count,
        "motion_command_frames": motion_frames,
        "initial_live_warmup_attempts": initial_attempts,
        "final_live_warmup_attempts": final_attempts,
        "detector_mode": lane_detector.config.get("detector_mode"),
        "target_line_role": lane_detector.config.get("target_line_role"),
        "controller_verification_state": controller_config[
            "verification_state"
        ],
        "controller_parameters": {
            key: controller_config[key]
            for key in ("kp", "ki", "kd", "steering_limit")
        },
        "single_line_control": {
            "near_weight": TRACKING_NEAR_WEIGHT,
            "lookahead_weight": TRACKING_LOOKAHEAD_WEIGHT,
            "curve_feedforward": TRACKING_CURVE_FEEDFORWARD,
            "heading_feedforward": TRACKING_HEADING_FEEDFORWARD,
            "turn_threshold": COMMISSIONING_TURN_THRESHOLD,
        },
        "minimum_required_line_confidence": float(
            controller_config["lane_confidence_minimum"]
        ),
        "effective_camera_stale_timeout_s": float(
            controller_config["camera_stale_timeout_s"]
        ),
        "effective_yolo_stale_timeout_s": float(
            controller_config["yolo_stale_timeout_s"]
        ),
        "effective_command_lease_s": float(
            controller_config["command_lease_s"]
        ),
        "commissioning_speed_scales": {
            "breakaway": COMMISSIONING_BREAKAWAY_SPEED_SCALE,
            "straight": COMMISSIONING_STRAIGHT_SPEED_SCALE,
            "turn": COMMISSIONING_TURN_SPEED_SCALE,
            "breakaway_seconds": COMMISSIONING_BREAKAWAY_SECONDS,
        },
        "camera_runtime": {
            "capture_width": 1280,
            "capture_height": 720,
            "output_width": CAMERA_OUTPUT_WIDTH,
            "output_height": CAMERA_OUTPUT_HEIGHT,
            "requested_framerate": CAMERA_CAPTURE_FRAMERATE,
        },
        "stationary_reacquisition_count": stationary_reacquisition_count,
        "stationary_reacquisition_attempts": (
            stationary_reacquisition_attempts
        ),
        "stationary_reacquisition_error": stationary_reacquisition_error,
        "starting_voltage_v": starting_voltage,
        "minimum_voltage_v": min(voltages) if voltages else None,
        "ending_voltage_v": ending_voltage,
        "average_control_fps": statistics.mean(fps_values) if fps_values else None,
        "watchdog_fault": watchdog_fault,
        "watchdog_trip_count": watchdog_trips,
        "actual_total_energized_duration_s": energized_duration,
        "hardware_off_confirmed": hardware_off,
        "deadline_stopper": (
            None if deadline_stopper is None else deadline_stopper.snapshot()
        ),
        "csv": csv_path,
        "completed_utc": utc_now(),
    }
    with open(json_path, "w") as summary_file:
        json.dump(summary, summary_file, indent=2, sort_keys=True)
        summary_file.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print("Motors are OFF. Live commissioning log: {}".format(json_path))
    return 0 if successful else 1


if __name__ == "__main__":
    sys.exit(main())
