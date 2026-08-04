#!/usr/bin/env python3
"""Validation-gated physical track runner with immutable telemetry logging."""

from __future__ import print_function

import argparse
import atexit
import collections
import csv
import datetime
import hashlib
import json
import math
import os
import signal
import statistics
import sys
import time

import cv2

from control_core import (
    PIDController,
    SafetySupervisor,
    cruise_normalized_command,
    load_controller_config,
    mix_forward,
)
from gst_camera_bridge import GstCamera
from ipm_lane import IPMLaneDetector
from motor_mapping import load_motor_config
from safe_motor_output import MotionDeadlineStopper, SafeMotorOutput
from validation_manager import GATES_PATH, REQUIRED_MOTION_GATES, read_json
from yolov5_runtime import (
    AsyncYoloWorker,
    YoloV5Detector,
    draw_detections,
    evaluate_obstacles,
    format_detections,
    mark_path_overlap,
)


_EMERGENCY_MOTOR_OUTPUT = None
TRACK_DEADLINE_STOP_MARGIN_SECONDS = 0.10
INITIAL_PERCEPTION_WARMUP_TIMEOUT_SECONDS = 30.0
REARM_PERCEPTION_WARMUP_TIMEOUT_SECONDS = 10.0
YOLO_WORKER_POLL_SECONDS = 0.01
MINIMUM_RESULT_DRIVE_ROWS = 5
MINIMUM_RESULT_MOTION_ROWS = 5
MINIMUM_RESULT_UNIQUE_YOLO_RESULTS = 2


class TerminationRequested(Exception):
    pass


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


def utc_now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        while True:
            block = input_file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def verify_gates():
    state = read_json(GATES_PATH)
    missing = [
        name
        for name in REQUIRED_MOTION_GATES
        if not state.get("gates", {}).get(name, {}).get("passed")
    ]
    if missing or not state.get("physical_motion_authorized"):
        raise RuntimeError(
            "Physical run locked. Unresolved validation gates: {}".format(
                ", ".join(missing) if missing else "authorization flag false"
            )
        )
    return state


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


def yolo_state_is_current(
    yolo_state,
    controller_config,
    minimum_yolo_source_frame=None,
):
    """Apply the physical-run YOLO freshness policy without relaxing it."""
    return (
        yolo_state["has_result"]
        and not yolo_state["error"]
        and 0.0 <= yolo_state["age_seconds"]
        <= float(controller_config["yolo_stale_timeout_s"])
        and yolo_state["source_frame_lag"] is not None
        and yolo_state["source_frame_lag"]
        <= int(controller_config["yolo_max_frame_lag"])
        and (
            minimum_yolo_source_frame is None
            or yolo_state["source_frame"] is not None
            and yolo_state["source_frame"] >= minimum_yolo_source_frame
        )
    )


def format_yolo_warmup_diagnostic(
    yolo_state,
    current_frame,
    accepted_submissions,
    attempted_submissions,
    phase,
):
    return (
        "phase={}; has_result={}; busy={}; error={}; age_s={}; "
        "result_age_s={}; frame_lag={}; source_frame={}; "
        "current_frame={}; inference_ms={}; detector_ms={}; "
        "accepted_submissions={}/{}"
    ).format(
        phase,
        bool(yolo_state.get("has_result")),
        bool(yolo_state.get("busy")),
        yolo_state.get("error"),
        yolo_state.get("age_seconds"),
        yolo_state.get("result_age_seconds"),
        yolo_state.get("source_frame_lag"),
        yolo_state.get("source_frame"),
        current_frame,
        yolo_state.get("inference_ms"),
        yolo_state.get("end_to_end_ms"),
        accepted_submissions,
        attempted_submissions,
    )


def wait_for_yolo_worker_idle(worker, deadline, current_frame):
    """Yield the CPU while the one-slot worker owns an inference request."""
    while True:
        state = worker.snapshot(current_frame_number=current_frame)
        # An error from an older request remains in ``latest`` while a newer
        # recovery request is busy.  Wait for that request to finish before
        # deciding whether the worker is still in error.
        if not state.get("busy"):
            return state
        remaining = float(deadline) - time.monotonic()
        if remaining <= 0.0:
            return state
        time.sleep(min(YOLO_WORKER_POLL_SECONDS, remaining))


def warm_perception(
    camera,
    lane_detector,
    worker,
    controller_config,
    minimum_yolo_source_frame=None,
    timeout_seconds=REARM_PERCEPTION_WARMUP_TIMEOUT_SECONDS,
):
    """Prove a clear, current perception state before motor I/O is opened."""
    timeout_seconds = float(timeout_seconds)
    if not 1.0 <= timeout_seconds <= 60.0:
        raise ValueError("Perception warm-up timeout must be from 1 to 60 seconds.")
    deadline = time.monotonic() + timeout_seconds
    latest_reason = "NO_FRAME"
    latest_yolo_diagnostic = "no YOLO submission completed"
    submitted_frames = 0
    accepted_submissions = 0
    rejection_counts = collections.Counter()
    last_lane_status = "NOT_OBSERVED"
    last_lane_confidence = None
    last_camera_age = None
    last_status_print = time.monotonic()
    while time.monotonic() < deadline:
        # Keep the expensive stages serial during readiness.  The Nano has
        # proven each stage independently, but running IPM and YOLO at exactly
        # the same instant can push a fresh camera observation past 250 ms.
        # This mirrors the validated asynchronous motion policy: a current
        # YOLO result may guard a newer lane frame while both timestamps remain
        # inside their original limits.
        idle_state = wait_for_yolo_worker_idle(worker, deadline, None)
        if idle_state.get("busy"):
            latest_reason = "YOLO_WORKER_BUSY_TIMEOUT"
            rejection_counts[latest_reason] += 1
            latest_yolo_diagnostic = format_yolo_warmup_diagnostic(
                idle_state,
                None,
                accepted_submissions,
                submitted_frames,
                "PRE_READ_WAIT",
            )
            continue

        # Stage 1: acquire and complete one fresh YOLO observation without
        # competing with IPM for the Nano's limited CPU/memory bandwidth.
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        success, yolo_frame, yolo_metadata = camera.read_with_metadata(
            timeout_seconds=max(0.01, min(2.0, remaining))
        )
        if not success:
            raise RuntimeError("CSI frame unavailable during pre-motion warm-up.")
        require_trustworthy_camera_metadata(yolo_metadata)
        yolo_camera_age = (
            time.monotonic() - yolo_metadata["capture_monotonic"]
        )
        if yolo_camera_age > float(
            controller_config["camera_stale_timeout_s"]
        ):
            latest_reason = "YOLO_SOURCE_CAMERA_STALE"
            rejection_counts[latest_reason] += 1
            continue

        yolo_source_frame = int(yolo_metadata["sequence"])
        submitted_frames += 1
        accepted = worker.submit(
            yolo_frame,
            yolo_source_frame,
            capture_monotonic=yolo_metadata["capture_monotonic"],
            source_pts_ns=yolo_metadata["pts_ns"],
        )
        if not accepted:
            latest_reason = "YOLO_SUBMISSION_REJECTED"
            rejection_counts[latest_reason] += 1
            yolo_state = wait_for_yolo_worker_idle(
                worker,
                deadline,
                yolo_source_frame,
            )
            latest_yolo_diagnostic = format_yolo_warmup_diagnostic(
                yolo_state,
                yolo_source_frame,
                accepted_submissions,
                submitted_frames,
                "SUBMISSION_REJECTED_WAIT",
            )
            continue
        accepted_submissions += 1

        yolo_state = wait_for_yolo_worker_idle(
            worker,
            deadline,
            yolo_source_frame,
        )
        latest_yolo_diagnostic = format_yolo_warmup_diagnostic(
            yolo_state,
            yolo_source_frame,
            accepted_submissions,
            submitted_frames,
            "SERIAL_YOLO_COMPLETE",
        )
        if not yolo_state_is_current(
            yolo_state,
            controller_config,
            minimum_yolo_source_frame=minimum_yolo_source_frame,
        ):
            if yolo_state.get("error"):
                latest_reason = "YOLO_WORKER_ERROR"
            elif yolo_state.get("busy"):
                latest_reason = "YOLO_WORKER_BUSY_TIMEOUT"
            else:
                latest_reason = "YOLO_NOT_CURRENT"
            rejection_counts[latest_reason] += 1
            continue
        if int(yolo_state["source_frame"]) != yolo_source_frame:
            latest_reason = "YOLO_RESULT_FRAME_MISMATCH"
            rejection_counts[latest_reason] += 1
            continue

        early_obstacle = evaluate_obstacles(yolo_state["detections"])
        if early_obstacle["state"] == "STOP":
            raise RuntimeError(
                "Pre-motion perception blocked by {}.".format(
                    early_obstacle["reason"]
                )
            )

        # Stage 2: pull the newest frame after YOLO has finished, then run IPM
        # alone.  The completed YOLO result is rechecked against this newer
        # frame, including its source age and frame-lag budget.
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            latest_reason = "WARMUP_DEADLINE_AFTER_YOLO"
            rejection_counts[latest_reason] += 1
            continue
        success, lane_frame, lane_metadata = camera.read_with_metadata(
            timeout_seconds=max(0.01, min(2.0, remaining))
        )
        if not success:
            raise RuntimeError("CSI frame unavailable during pre-motion warm-up.")
        require_trustworthy_camera_metadata(lane_metadata)
        lane_camera_age = (
            time.monotonic() - lane_metadata["capture_monotonic"]
        )
        if lane_camera_age > float(
            controller_config["camera_stale_timeout_s"]
        ):
            latest_reason = "LANE_SOURCE_CAMERA_STALE"
            rejection_counts[latest_reason] += 1
            continue

        lane_source_frame = int(lane_metadata["sequence"])
        if (
            lane_source_frame <= yolo_source_frame
            or lane_metadata["capture_monotonic"]
            <= yolo_metadata["capture_monotonic"]
        ):
            latest_reason = "LANE_FRAME_NOT_NEWER_THAN_YOLO_FRAME"
            rejection_counts[latest_reason] += 1
            continue
        observation = lane_detector.observe(lane_frame)
        last_lane_status = str(observation["status"])
        last_lane_confidence = float(observation["confidence"])
        yolo_state = worker.snapshot(current_frame_number=lane_source_frame)
        latest_yolo_diagnostic = format_yolo_warmup_diagnostic(
            yolo_state,
            lane_source_frame,
            accepted_submissions,
            submitted_frames,
            "SERIAL_POST_IPM",
        )
        camera_age = time.monotonic() - lane_metadata["capture_monotonic"]
        last_camera_age = camera_age
        if camera_age > float(controller_config["camera_stale_timeout_s"]):
            latest_reason = "CAMERA_STALE_AFTER_PROCESSING"
            rejection_counts[latest_reason] += 1
            continue
        if not yolo_state_is_current(
            yolo_state,
            controller_config,
            minimum_yolo_source_frame=minimum_yolo_source_frame,
        ):
            if yolo_state.get("error"):
                latest_reason = "YOLO_WORKER_ERROR_AFTER_IPM"
            elif yolo_state.get("busy"):
                latest_reason = "YOLO_WORKER_BUSY_AFTER_IPM"
            else:
                latest_reason = "YOLO_NOT_CURRENT_AFTER_IPM"
            rejection_counts[latest_reason] += 1
            continue
        if int(yolo_state["source_frame"]) != yolo_source_frame:
            latest_reason = "YOLO_RESULT_CHANGED_DURING_IPM"
            rejection_counts[latest_reason] += 1
            continue

        if time.monotonic() - last_status_print >= 2.0:
            print(
                "Warm-up status | lane {} confidence {:.3f} | "
                "camera {:.1f} ms | YOLO {:.1f} ms".format(
                    last_lane_status,
                    last_lane_confidence,
                    last_camera_age * 1000.0,
                    float(yolo_state["age_seconds"]) * 1000.0,
                )
            )
            last_status_print = time.monotonic()

        detections = mark_path_overlap(
            yolo_state["detections"], observation["source_polygon"]
        )
        obstacle = evaluate_obstacles(detections)
        if obstacle["state"] == "STOP":
            raise RuntimeError(
                "Pre-motion perception blocked by {}.".format(
                    obstacle["reason"]
                )
            )
        if observation["status"] != "FULL":
            latest_reason = "LANE_{}".format(observation["status"])
            rejection_counts[latest_reason] += 1
            continue
        if float(observation["confidence"]) < float(
            controller_config["lane_confidence_minimum"]
        ):
            latest_reason = "LANE_CONFIDENCE_LOW"
            rejection_counts[latest_reason] += 1
            continue
        if time.monotonic() >= deadline:
            latest_reason = "WARMUP_DEADLINE_AFTER_VALIDATION"
            rejection_counts[latest_reason] += 1
            continue
        # The caller uses this value as the minimum source sequence for the
        # next YOLO re-arm, so return the YOLO evidence frame rather than the
        # newer lane frame.
        return int(yolo_state["source_frame"])
    dominant_reason = (
        rejection_counts.most_common(1)[0][0]
        if rejection_counts
        else latest_reason
    )
    raise RuntimeError(
        "Perception did not become motion-ready within {:.1f} seconds: "
        "dominant_reason={}; final_phase_reason={}; "
        "lane_status={}; lane_confidence={}; camera_age_s={}; "
        "rejections={}; {}."
        .format(
            timeout_seconds,
            dominant_reason,
            latest_reason,
            last_lane_status,
            last_lane_confidence,
            last_camera_age,
            dict(rejection_counts),
            latest_yolo_diagnostic,
        )
    )


def main():
    global _EMERGENCY_MOTOR_OUTPUT
    parser = argparse.ArgumentParser(description="Short guarded autonomous track run")
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--run-label", default="track_validation")
    parser.add_argument(
        "--purpose",
        choices=("tuning", "results"),
        default="tuning",
        help="Provisional gains permit tuning runs only; results need a physically tuned config.",
    )
    arguments = parser.parse_args()
    if not arguments.enable_motors:
        print("REFUSED: this file is the physical runner.")
        print("Use integrated_control_dry_run.py until every gate passes.")
        return 1
    if not 0.5 <= arguments.seconds <= 10.0:
        print("REFUSED: --seconds must be between 0.5 and 10.0.")
        return 1

    try:
        gate_state = verify_gates()
    except Exception as error:
        print("REFUSED: {}".format(error))
        print("Run: python validation_manager.py show")
        return 1

    controller_config = load_controller_config()
    controller_is_tuned = (
        controller_config["verification_state"] == "PHYSICALLY_TUNED"
    )
    if arguments.purpose == "results" and not controller_is_tuned:
        print("REFUSED: results runs require verification_state=PHYSICALLY_TUNED.")
        print("Use --purpose tuning for the first short closed-loop trials.")
        return 1
    if not controller_is_tuned and arguments.seconds > 2.0:
        print("REFUSED: provisional PID gains are limited to 2-second tuning runs.")
        return 1
    motor_config = load_motor_config()
    lane_detector = IPMLaneDetector()
    if not lane_detector.physically_calibrated:
        print("REFUSED: IPM is not physically calibrated.")
        return 1
    pid = PIDController(controller_config)
    safety = SafetySupervisor(controller_config)
    base_command = cruise_normalized_command(motor_config)

    # Initialise and warm perception before creating any motor interface.
    detector = YoloV5Detector(
        image_size=controller_config["yolo_image_size"],
        confidence=controller_config["yolo_confidence_threshold"],
        iou=controller_config["yolo_iou_threshold"],
    )
    if detector.device.type != "cuda":
        print("REFUSED: physical track runs require CUDA YOLO inference.")
        print("CPU fallback cannot inherit the CUDA validation gate.")
        return 1
    worker = AsyncYoloWorker(detector)
    worker.start()
    camera = GstCamera(
        sensor_id=0,
        capture_width=1280,
        capture_height=720,
        output_width=640,
        output_height=360,
        framerate=21,
    )

    atexit.register(emergency_process_cleanup)
    for signal_name in ("SIGTERM", "SIGHUP", "SIGQUIT"):
        signal_value = getattr(signal, signal_name, None)
        if signal_value is not None:
            signal.signal(signal_value, termination_handler)

    try:
        print(
            "Cold-start perception warm-up (motor interface unavailable; "
            "up to {:.0f} seconds).".format(
                INITIAL_PERCEPTION_WARMUP_TIMEOUT_SECONDS
            )
        )
        warmed_source_frame = warm_perception(
            camera,
            lane_detector,
            worker,
            controller_config,
            timeout_seconds=INITIAL_PERCEPTION_WARMUP_TIMEOUT_SECONDS,
        )
    except Exception as error:
        camera.close()
        worker.stop()
        print("REFUSED: pre-motion perception check failed: {}".format(error))
        return 1

    print("=" * 68)
    print("VALIDATION-GATED PHYSICAL TRACK RUN")
    print("Duration: {:.2f} seconds maximum".format(arguments.seconds))
    print("Purpose: {} | controller: {}".format(
        arguments.purpose.upper(), controller_config["verification_state"]
    ))
    print("Use a closed, flat track with barriers and no stairs/drop-offs.")
    print("Keep people out of the vehicle path during motion.")
    print("A spotter must hold the motor-battery disconnect.")
    print("MOTOR BATTERY MUST REMAIN PHYSICALLY DISCONNECTED FOR INITIALISATION.")
    print("Camera/lane/YOLO failure, stale data, low voltage or Ctrl+C => STOP.")
    print("Software command lease is secondary protection only.")
    print("The physical battery disconnect is the independent emergency stop.")
    print("=" * 68)
    token = "TRACK-{}-{:.1f}".format(
        arguments.purpose.upper(), arguments.seconds
    )
    if input("Type {} to initialise OFF: ".format(token)).strip() != token:
        camera.close()
        worker.stop()
        print("Cancelled; motors were never opened.")
        return 0

    project_directory = os.path.dirname(os.path.abspath(__file__))
    results_directory = os.path.join(project_directory, "results")
    if not os.path.isdir(results_directory):
        os.makedirs(results_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    safe_label = "".join(
        character if character.isalnum() or character in "-_" else "_"
        for character in arguments.run_label
    )[:40]
    run_id = "{}_{}".format(safe_label or "track", stamp)
    csv_path = os.path.join(results_directory, run_id + ".csv")
    json_path = os.path.join(results_directory, run_id + ".json")

    fields = (
        "run_id", "timestamp_utc", "monotonic_s", "frame_id",
        "camera_sequence", "camera_pts_ns", "camera_age_ms", "camera_fresh",
        "camera_age_method", "lane_status", "lane_confidence", "lane_offset",
        "near_field_offset", "lookahead_offset", "heading_error_rad",
        "curvature_per_px", "candidate_count",
        "lane_width_px", "yolo_has_result", "yolo_fresh", "yolo_age_s", "yolo_result_age_s",
        "yolo_source_frame_lag", "yolo_source_frame",
        "yolo_inference_ms", "yolo_nms_ms", "yolo_detector_ms",
        "objects", "obstacle_state", "safety_state",
        "safety_reason", "speed_scale", "pid_p", "pid_i", "pid_d",
        "pid_output", "left_normalized", "right_normalized",
        "left_mapped_duty", "right_mapped_duty", "battery_pre_command_v",
        "battery_post_command_v", "battery_voltage_v", "watchdog_fault",
        "watchdog_trip_count",
        "control_fps", "physical_motor_commands",
    )
    csv_file = open(csv_path, "w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=fields)
    writer.writeheader()

    motor_output = None
    starting_voltage = None
    ending_voltage = None
    frame_number = 0
    voltages = []
    offsets = []
    fps_values = []
    inference_values = []
    seen_yolo_source_frames = set()
    yolo_input_shapes = set()
    safety_counts = collections.Counter()
    reason_counts = collections.Counter()
    motion_command_frames = 0
    completed = False
    error_text = None
    run_start = time.monotonic()
    previous_decision_timestamp = None
    source_frame_number = warmed_source_frame
    measurement_end = None
    run_deadline = None
    motion_deadline = None
    deadline_stopper = None

    try:
        # The camera warm-up and operator confirmation can take time. Recheck
        # every evidence/artifact hash immediately before opening motor I/O.
        gate_state = verify_gates()
        motor_output = SafeMotorOutput(
            motor_config=motor_config,
            controller_config=controller_config,
        )
        _EMERGENCY_MOTOR_OUTPUT = motor_output
        print("Controller initialised; hardware OFF write confirmed.")
        print("Place the robot at the marked start while motor power is OFF.")
        print("After connection, do not touch the chassis, wheels, or track.")
        if input(
            "Type MOTOR-POWER-CONNECTED only after everyone is clear: "
        ).strip() != "MOTOR-POWER-CONNECTED":
            raise RuntimeError("Motor-power connection was not confirmed.")
        if input(
            "Type TRACK-AREA-CLEAR when hands are off and the closed track "
            "is clear: "
        ).strip() != "TRACK-AREA-CLEAR":
            raise RuntimeError("Final hands-off track clearance was not confirmed.")
        # This prompt is intentionally before the final warm-up.  Perception
        # evidence collected before an unbounded operator pause is stale.
        gate_state = verify_gates()
        print(
            "Final fresh perception recheck (outputs remain OFF; up to "
            "{:.0f} seconds).".format(
                REARM_PERCEPTION_WARMUP_TIMEOUT_SECONDS
            )
        )
        warmed_source_frame = warm_perception(
            camera,
            lane_detector,
            worker,
            controller_config,
            minimum_yolo_source_frame=warmed_source_frame + 1,
            timeout_seconds=REARM_PERCEPTION_WARMUP_TIMEOUT_SECONDS,
        )
        starting_voltage = motor_output.read_voltage()
        voltages.append(starting_voltage)
        run_start = time.monotonic()
        run_deadline = run_start + arguments.seconds
        motion_deadline = run_deadline - TRACK_DEADLINE_STOP_MARGIN_SECONDS
        deadline_stopper = MotionDeadlineStopper(
            motor_output,
            motion_deadline,
            "RUN_HARD_DEADLINE",
        ).start()
        while True:
            remaining_seconds = motion_deadline - time.monotonic()
            if remaining_seconds <= 0.0:
                break
            success, frame, camera_metadata = camera.read_with_metadata(
                timeout_seconds=max(0.01, min(2.0, remaining_seconds))
            )
            if not success:
                if time.monotonic() >= motion_deadline:
                    break
                motor_output.stop("CAMERA_FRAME_UNAVAILABLE")
                raise RuntimeError("CSI frame unavailable.")
            require_trustworthy_camera_metadata(camera_metadata)
            camera_age = (
                time.monotonic() - camera_metadata["capture_monotonic"]
            )
            next_frame_number = frame_number + 1
            source_frame_number = int(camera_metadata["sequence"])
            observation = lane_detector.observe(frame)
            worker.submit(
                frame,
                source_frame_number,
                capture_monotonic=camera_metadata["capture_monotonic"],
                source_pts_ns=camera_metadata["pts_ns"],
            )
            yolo_state = worker.snapshot(
                current_frame_number=source_frame_number
            )
            yolo_fresh = (
                yolo_state["has_result"]
                and not yolo_state["error"]
                and 0.0 <= yolo_state["age_seconds"]
                and yolo_state["age_seconds"]
                <= float(controller_config["yolo_stale_timeout_s"])
                and yolo_state["source_frame_lag"] is not None
                and yolo_state["source_frame_lag"]
                <= int(controller_config["yolo_max_frame_lag"])
            )
            if yolo_fresh:
                active_detections = mark_path_overlap(
                    yolo_state["detections"], observation["source_polygon"]
                )
                obstacle = evaluate_obstacles(active_detections)
            else:
                active_detections = []
                obstacle = {"state": "DRIVE", "reason": "NO_FRESH_RESULT"}
            camera_age = (
                time.monotonic() - camera_metadata["capture_monotonic"]
            )
            decision = safety.evaluate(
                observation["status"],
                observation["confidence"],
                camera_age,
                yolo_state,
                obstacle,
            )
            if decision["state"] == "STOP":
                pid.reset()
                terms = {"p": 0.0, "i": 0.0, "d": 0.0, "output": 0.0}
                left_command, right_command = 0.0, 0.0
                motor_output.stop(decision["reason"])
                voltage = motor_output.read_voltage()
                battery_pre_command = voltage
                battery_post_command = None
                left_duty, right_duty = 0.0, 0.0
            else:
                terms = pid.update(observation["lane_offset"])
                left_command, right_command = mix_forward(
                    base_command * float(decision["speed_scale"]),
                    terms["output"],
                )
                if time.monotonic() >= motion_deadline:
                    motor_output.stop("RUN_DURATION_EXPIRED")
                    break
                try:
                    applied = motor_output.command(
                        left_command,
                        right_command,
                        reason=decision["reason"],
                        deadline_monotonic=motion_deadline,
                    )
                except RuntimeError:
                    if (
                        time.monotonic() >= motion_deadline
                        and motor_output.hardware_off_confirmed
                    ):
                        break
                    raise
                voltage = applied["battery_voltage"]
                battery_pre_command = applied["battery_pre_command_v"]
                battery_post_command = applied["battery_post_command_v"]
                left_duty = applied["left_duty"]
                right_duty = applied["right_duty"]

            voltages.append(voltage)
            if observation["status"] == "FULL":
                offsets.append(abs(float(observation["lane_offset"])))
            if (
                yolo_fresh
                and yolo_state["source_frame"] not in seen_yolo_source_frames
            ):
                seen_yolo_source_frames.add(yolo_state["source_frame"])
                inference_values.append(float(yolo_state["inference_ms"]))
                if yolo_state.get("input_tensor_shape") is not None:
                    yolo_input_shapes.add(
                        tuple(yolo_state["input_tensor_shape"])
                    )
            decision_timestamp = time.monotonic()
            if previous_decision_timestamp is None:
                fps = 0.0
            else:
                decision_period = max(
                    0.000001,
                    decision_timestamp - previous_decision_timestamp,
                )
                fps = 1.0 / decision_period
                fps_values.append(fps)
            previous_decision_timestamp = decision_timestamp
            # Commit counters only for a complete control/telemetry row. A
            # frame that crosses the motion deadline before command/STOP is
            # deliberately excluded from the run summary.
            frame_number = next_frame_number
            safety_counts[decision["state"]] += 1
            reason_counts[decision["reason"]] += 1
            writer.writerow(
                {
                    "run_id": run_id,
                    "timestamp_utc": utc_now(),
                    "monotonic_s": round(time.monotonic(), 6),
                    "frame_id": frame_number,
                    "camera_sequence": camera_metadata["sequence"],
                    "camera_pts_ns": camera_metadata["pts_ns"],
                    "camera_age_ms": round(camera_age * 1000.0, 3),
                    "camera_fresh": 1,
                    "camera_age_method": camera_metadata["source_age_method"],
                    "lane_status": observation["status"],
                    "lane_confidence": round(observation["confidence"], 4),
                    "lane_offset": round(observation["lane_offset"], 6),
                    "near_field_offset": round(
                        observation["near_field_offset"], 6
                    ),
                    "lookahead_offset": round(
                        observation["lookahead_offset"], 6
                    ),
                    "heading_error_rad": (
                        "" if observation["heading_error_rad"] is None
                        else round(observation["heading_error_rad"], 7)
                    ),
                    "curvature_per_px": (
                        "" if observation["curvature_per_px"] is None
                        else round(observation["curvature_per_px"], 9)
                    ),
                    "candidate_count": observation["candidate_count"],
                    "lane_width_px": "" if observation["lane_width_px"] is None else round(observation["lane_width_px"], 3),
                    "yolo_has_result": int(yolo_state["has_result"]),
                    "yolo_fresh": int(yolo_fresh),
                    "yolo_age_s": "" if not yolo_state["has_result"] else round(yolo_state["age_seconds"], 3),
                    "yolo_result_age_s": "" if not yolo_state["has_result"] else round(yolo_state["result_age_seconds"], 3),
                    "yolo_source_frame_lag": "" if yolo_state["source_frame_lag"] is None else yolo_state["source_frame_lag"],
                    "yolo_source_frame": yolo_state["source_frame"],
                    "yolo_inference_ms": round(yolo_state["inference_ms"], 3),
                    "yolo_nms_ms": round(yolo_state["nms_ms"], 3),
                    "yolo_detector_ms": round(yolo_state["end_to_end_ms"], 3),
                    "objects": format_detections(active_detections),
                    "obstacle_state": obstacle["state"],
                    "safety_state": decision["state"],
                    "safety_reason": decision["reason"],
                    "speed_scale": round(decision["speed_scale"], 3),
                    "pid_p": round(terms["p"], 6),
                    "pid_i": round(terms["i"], 6),
                    "pid_d": round(terms["d"], 6),
                    "pid_output": round(terms["output"], 6),
                    "left_normalized": round(left_command, 6),
                    "right_normalized": round(right_command, 6),
                    "left_mapped_duty": round(left_duty, 6),
                    "right_mapped_duty": round(right_duty, 6),
                    "battery_pre_command_v": round(battery_pre_command, 4),
                    "battery_post_command_v": "" if battery_post_command is None else round(battery_post_command, 4),
                    "battery_voltage_v": round(voltage, 4),
                    "watchdog_fault": motor_output.watchdog_fault or "",
                    "watchdog_trip_count": motor_output.watchdog_trip_count,
                    "control_fps": round(fps, 3),
                    "physical_motor_commands": int(
                        max(left_duty, right_duty) > 0.0
                    ),
                }
            )
            if max(left_duty, right_duty) > 0.0:
                motion_command_frames += 1
            csv_file.flush()
            if not arguments.headless:
                overlay = observation["original_overlay"]
                if yolo_state["has_result"]:
                    draw_detections(
                        overlay,
                        active_detections,
                        yolo_state["corridor"],
                        path_polygon=observation["source_polygon"],
                    )
                cv2.putText(
                    overlay,
                    "{} {}".format(decision["state"], decision["reason"]),
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 255, 0) if decision["state"] == "DRIVE" else (0, 0, 255),
                    2,
                )
                cv2.imshow("Guarded physical track run", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    motor_output.stop("OPERATOR_Q_STOP")
                    error_text = "OPERATOR_Q_STOP"
                    break
        completed = error_text is None
    except KeyboardInterrupt:
        error_text = "EMERGENCY_STOP_CTRL_C"
        if motor_output is not None:
            try:
                motor_output.stop(error_text)
            except Exception:
                print("SPOTTER: DISCONNECT MOTOR POWER NOW.")
        print("\nEMERGENCY STOP")
    except TerminationRequested as error:
        error_text = str(error)
        if motor_output is not None:
            try:
                motor_output.stop(error_text)
            except Exception:
                print("SPOTTER: DISCONNECT MOTOR POWER NOW.")
        print("\nEMERGENCY STOP: {}".format(error_text))
    except Exception as error:
        error_text = repr(error)
        if motor_output is not None:
            try:
                motor_output.stop("TRACK_RUN_EXCEPTION")
            except Exception:
                print("SPOTTER: DISCONNECT MOTOR POWER NOW.")
        print("TRACK RUN STOPPED: {}".format(error))
    finally:
        if motor_output is not None:
            try:
                motor_output.stop("RUN_FINALIZATION")
                time.sleep(0.02)
                ending_voltage = motor_output.read_voltage()
                voltages.append(ending_voltage)
            except Exception as final_voltage_error:
                if error_text is None:
                    error_text = "FINAL_VOLTAGE_ERROR: {!r}".format(
                        final_voltage_error
                    )
                print("FINAL VOLTAGE ERROR: {}".format(final_voltage_error))
            try:
                motor_output.close()
            except Exception as close_error:
                if error_text is None:
                    error_text = "MOTOR_CLOSE_ERROR: {!r}".format(close_error)
                print("MOTOR CLOSE ERROR: {}".format(close_error))
        if deadline_stopper is not None:
            deadline_stopper.cancel()
            if deadline_stopper.stop_error is not None and error_text is None:
                error_text = "DEADLINE_STOP_ERROR: {}".format(
                    deadline_stopper.stop_error
                )
        measurement_end = time.monotonic()
        _EMERGENCY_MOTOR_OUTPUT = None
        try:
            camera.close()
        except Exception as close_error:
            print("CAMERA CLOSE ERROR: {}".format(close_error))
        try:
            worker.stop()
        except Exception as close_error:
            print("YOLO WORKER CLOSE ERROR: {}".format(close_error))
        csv_file.close()
        cv2.destroyAllWindows()

    elapsed = max(0.0, measurement_end - run_start)
    observed_minimum_voltage = None
    voltage_candidates = list(voltages)
    if (
        motor_output is not None
        and math.isfinite(float(motor_output.minimum_voltage))
    ):
        voltage_candidates.append(float(motor_output.minimum_voltage))
    if voltage_candidates:
        observed_minimum_voltage = min(voltage_candidates)
    run_successful = bool(
        completed
        and error_text is None
        and frame_number > 0
        and safety_counts["DRIVE"] > 0
        and motion_command_frames > 0
        and motor_output is not None
        and motor_output.watchdog_fault is None
        and motor_output.watchdog_trip_count == 0
        and motor_output.hardware_off_confirmed
        and not motor_output.hardware_off_retry_required
        and float(motor_output.total_energized_duration_s)
        <= float(arguments.seconds)
        and (
            deadline_stopper is None
            or deadline_stopper.stop_error is None
        )
    )
    run_valid_for_results = bool(
        run_successful
        and arguments.purpose == "results"
        and controller_is_tuned
        and safety_counts["DRIVE"] >= MINIMUM_RESULT_DRIVE_ROWS
        and motion_command_frames >= MINIMUM_RESULT_MOTION_ROWS
        and len(seen_yolo_source_frames)
        >= MINIMUM_RESULT_UNIQUE_YOLO_RESULTS
    )
    csv_sha256 = sha256_file(csv_path)
    summary = {
        "schema_version": 1,
        "test": "PHYSICAL_TRACK_RUN",
        "run_id": run_id,
        "completed": completed,
        "run_successful": run_successful,
        "run_valid_for_results": run_valid_for_results,
        "run_purpose": arguments.purpose,
        "controller_verification_state": controller_config["verification_state"],
        "yolo_device": str(detector.device),
        "yolo_fp16": bool(detector.use_half),
        "detector_mode": lane_detector.config.get("detector_mode"),
        "target_line_role": lane_detector.config.get("target_line_role"),
        "error": error_text,
        "physical_motor_interface_opened": motor_output is not None,
        "physical_motor_commands": motion_command_frames > 0,
        "motion_command_frames": motion_command_frames,
        "duration_s": elapsed,
        "requested_maximum_duration_s": float(arguments.seconds),
        "deadline_stop_margin_s": TRACK_DEADLINE_STOP_MARGIN_SECONDS,
        "actual_total_energized_duration_s": (
            None
            if motor_output is None
            else float(motor_output.total_energized_duration_s)
        ),
        "maximum_continuous_energized_duration_s": (
            None
            if motor_output is None
            else float(motor_output.maximum_energized_duration_s)
        ),
        "deadline_stopper": (
            None if deadline_stopper is None else deadline_stopper.snapshot()
        ),
        "frames": frame_number,
        "average_control_fps": statistics.mean(fps_values) if fps_values else None,
        "mean_absolute_lane_offset": statistics.mean(offsets) if offsets else None,
        "starting_voltage_v": starting_voltage,
        "minimum_voltage_v": observed_minimum_voltage,
        "ending_voltage_v": ending_voltage,
        "largest_voltage_sag_v": (
            starting_voltage - observed_minimum_voltage
            if starting_voltage is not None
            and observed_minimum_voltage is not None
            else None
        ),
        "mean_yolo_inference_ms": statistics.mean(inference_values) if inference_values else None,
        "unique_yolo_results": len(seen_yolo_source_frames),
        "observed_yolo_input_tensor_shapes": [
            list(shape) for shape in sorted(yolo_input_shapes)
        ],
        "watchdog_fault": (
            None if motor_output is None else motor_output.watchdog_fault
        ),
        "watchdog_trip_count": (
            0 if motor_output is None else motor_output.watchdog_trip_count
        ),
        "controller_parameters": {
            key: controller_config[key]
            for key in (
                "kp", "ki", "kd", "steering_limit",
                "lane_confidence_minimum", "camera_stale_timeout_s",
                "yolo_image_size", "yolo_confidence_threshold",
                "yolo_iou_threshold", "yolo_stale_timeout_s",
                "yolo_max_frame_lag", "command_lease_s",
                "motor_voltage_stop_v", "maximum_motor_voltage_v",
            )
        },
        "motor_mapping_parameters": {
            key: motor_config[key]
            for key in (
                "left_deadband_duty", "right_deadband_duty",
                "left_cruise_duty", "right_cruise_duty",
                "left_max_duty", "right_max_duty",
            )
        },
        "safety_state_counts": dict(safety_counts),
        "safety_reason_counts": dict(reason_counts),
        "measured_distance_m": None,
        "final_lateral_error_cm": None,
        "csv": csv_path,
        "csv_sha256": csv_sha256,
        "result_acceptance_requirements": {
            "minimum_drive_rows": MINIMUM_RESULT_DRIVE_ROWS,
            "minimum_motion_rows": MINIMUM_RESULT_MOTION_ROWS,
            "minimum_unique_yolo_results": (
                MINIMUM_RESULT_UNIQUE_YOLO_RESULTS
            ),
        },
        "validation_gate_snapshot": gate_state,
        "completed_utc": utc_now(),
    }
    with open(json_path, "w") as summary_file:
        json.dump(summary, summary_file, indent=2, sort_keys=True)
        summary_file.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print("Motors are OFF. Results: {}".format(json_path))
    accepted = (
        run_valid_for_results
        if arguments.purpose == "results"
        else run_successful
    )
    return 0 if accepted else 1


if __name__ == "__main__":
    sys.exit(main())
