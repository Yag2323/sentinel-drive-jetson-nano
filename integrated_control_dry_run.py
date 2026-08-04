#!/usr/bin/env python3
"""CSI + IPM + asynchronous YOLO + PID/motor mapping, with motors absent."""

from __future__ import print_function

import argparse
import collections
import csv
import datetime
import json
import os
import statistics
import sys
import time

import cv2
import numpy as np

from control_core import (
    PIDController,
    SafetySupervisor,
    cruise_normalized_command,
    load_controller_config,
    mix_forward,
)
from gst_camera_bridge import GstCamera
from ipm_lane import DEFAULT_CONFIG_PATH, IPMLaneDetector
from motor_mapping import load_motor_config, map_duty
from validation_manager import capture_artifact_snapshot
from yolov5_runtime import (
    AsyncYoloWorker,
    YoloV5Detector,
    draw_detections,
    evaluate_obstacles,
    format_detections,
    mark_path_overlap,
)


def utc_now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def main():
    parser = argparse.ArgumentParser(
        description="Motor-disabled integrated perception/control validation"
    )
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--ipm-config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--allow-sitl-seed",
        action="store_true",
        help="Diagnostic only; results cannot pass physical-IPM validation.",
    )
    arguments = parser.parse_args()
    if not 10.0 <= arguments.seconds <= 300.0:
        print("REFUSED: --seconds must be between 10 and 300.")
        return 1

    lane_detector = IPMLaneDetector(arguments.ipm_config)
    default_ipm_config_used = (
        os.path.realpath(os.path.abspath(arguments.ipm_config))
        == os.path.realpath(os.path.abspath(DEFAULT_CONFIG_PATH))
    )
    if not lane_detector.physically_calibrated and not arguments.allow_sitl_seed:
        print("REFUSED: IPM is not physically calibrated.")
        print("Run: python calibrate_ipm.py")
        return 1

    controller_config = load_controller_config()
    motor_config = load_motor_config()
    pid = PIDController(controller_config)
    safety = SafetySupervisor(controller_config)
    base_command = cruise_normalized_command(motor_config)
    artifact_snapshot_start = capture_artifact_snapshot(
        "integrated_control_dry_run"
    )

    yolo_detector = YoloV5Detector(
        image_size=controller_config["yolo_image_size"],
        confidence=controller_config["yolo_confidence_threshold"],
        iou=controller_config["yolo_iou_threshold"],
    )
    if yolo_detector.device.type != "cuda":
        print("REFUSED: integrated dry-run requires CUDA YOLO inference.")
        print("CPU fallback is diagnostic-only and cannot satisfy this gate.")
        return 1
    yolo_worker = AsyncYoloWorker(yolo_detector)
    yolo_worker.start()
    camera = GstCamera(
        sensor_id=0,
        capture_width=1280,
        capture_height=720,
        output_width=640,
        output_height=360,
        framerate=21,
    )

    project_directory = os.path.dirname(os.path.abspath(__file__))
    evidence_directory = os.path.join(project_directory, "evidence")
    if not os.path.isdir(evidence_directory):
        os.makedirs(evidence_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    run_id = "integrated_dry_{}".format(stamp)
    csv_path = os.path.join(evidence_directory, run_id + ".csv")
    json_path = os.path.join(evidence_directory, run_id + ".json")
    image_path = os.path.join(evidence_directory, run_id + ".jpg")

    fields = (
        "run_id",
        "timestamp_utc",
        "monotonic_s",
        "frame_id",
        "mode",
        "camera_sequence",
        "camera_pts_ns",
        "camera_age_ms",
        "camera_fresh",
        "lane_status",
        "lane_confidence",
        "lane_offset",
        "near_field_offset",
        "lookahead_offset",
        "heading_error_rad",
        "curvature_per_px",
        "candidate_count",
        "lane_width_px",
        "yolo_has_result",
        "yolo_age_s",
        "yolo_result_age_s",
        "yolo_source_frame_lag",
        "yolo_source_frame",
        "yolo_inference_ms",
        "yolo_nms_ms",
        "yolo_detector_ms",
        "objects",
        "obstacle_state",
        "safety_state",
        "safety_reason",
        "speed_scale",
        "pid_p",
        "pid_i",
        "pid_d",
        "pid_output",
        "left_normalized",
        "right_normalized",
        "left_mapped_duty",
        "right_mapped_duty",
        "battery_voltage_v",
        "physical_motor_commands",
        "control_fps",
    )
    csv_file = open(csv_path, "w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=fields)
    writer.writeheader()

    frame_number = 0
    full_frames = 0
    usable_frames = 0
    yolo_fresh_frames = 0
    submitted_frames = 0
    fps_values = []
    yolo_inference_values = []
    seen_yolo_source_frames = set()
    yolo_input_shapes = set()
    safety_counts = collections.Counter()
    reason_counts = collections.Counter()
    steering_sign_mismatches = 0
    steering_saturation_frames = 0
    steering_flip_count = 0
    large_derivative_frames = 0
    mapped_duty_violations = 0
    previous_nonzero_steering = None
    controlled_frames = 0
    latest_composite = None
    error_text = None
    test_start = time.monotonic()
    previous_control_timestamp = None

    print("=" * 64)
    print("INTEGRATED CONTROL DRY RUN")
    print("NO MOTOR MODULE IS IMPORTED; PWM VALUES ARE CALCULATED ONLY.")
    print("Show a clear track, briefly hide one/both lane edges, then show an object.")
    print("IPM state: {}".format(lane_detector.config.get("calibration_state")))
    print("Base command: {:.4f} -> left {:.3f}, right {:.3f}".format(
        base_command,
        map_duty(base_command, False, motor_config),
        map_duty(base_command, True, motor_config),
    ))
    print("=" * 64)

    try:
        while time.monotonic() - test_start < arguments.seconds:
            success, frame, camera_metadata = camera.read_with_metadata(
                timeout_seconds=2.0
            )
            if not success:
                raise RuntimeError("Camera frame unavailable; fail-safe STOP.")
            if (
                camera_metadata is None
                or not camera_metadata["timestamp_valid"]
                or not camera_metadata["source_age_trustworthy"]
                or not camera_metadata["fresh"]
            ):
                raise RuntimeError("Camera timestamp is invalid or repeated.")
            camera_age = time.monotonic() - camera_metadata["capture_monotonic"]
            frame_number += 1
            observation = lane_detector.observe(frame)
            if observation["status"] == "FULL":
                full_frames += 1
                usable_frames += 1
            elif observation["status"].startswith("PARTIAL"):
                usable_frames += 1

            if yolo_worker.submit(
                frame,
                frame_number,
                capture_monotonic=camera_metadata["capture_monotonic"],
                source_pts_ns=camera_metadata["pts_ns"],
            ):
                submitted_frames += 1
            yolo_state = yolo_worker.snapshot(current_frame_number=frame_number)
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
                yolo_fresh_frames += 1
                if yolo_state["source_frame"] not in seen_yolo_source_frames:
                    seen_yolo_source_frames.add(yolo_state["source_frame"])
                    yolo_inference_values.append(float(yolo_state["inference_ms"]))
                    if yolo_state.get("input_tensor_shape") is not None:
                        yolo_input_shapes.add(
                            tuple(yolo_state["input_tensor_shape"])
                        )
                active_detections = mark_path_overlap(
                    yolo_state["detections"], observation["source_polygon"]
                )
                obstacle_action = evaluate_obstacles(active_detections)
            else:
                active_detections = []
                obstacle_action = {"state": "DRIVE", "reason": "NO_FRESH_RESULT"}

            camera_age = (
                time.monotonic() - camera_metadata["capture_monotonic"]
            )
            decision = safety.evaluate(
                lane_status=observation["status"],
                lane_confidence=observation["confidence"],
                camera_age_s=camera_age,
                yolo_state=yolo_state,
                obstacle_action=obstacle_action,
            )
            safety_counts[decision["state"]] += 1
            reason_counts[decision["reason"]] += 1

            if decision["state"] == "STOP":
                pid.reset()
                previous_nonzero_steering = None
                terms = {"p": 0.0, "i": 0.0, "d": 0.0, "output": 0.0}
                left_command, right_command = 0.0, 0.0
            else:
                terms = pid.update(observation["lane_offset"])
                left_command, right_command = mix_forward(
                    base_command * float(decision["speed_scale"]),
                    terms["output"],
                )

                controlled_frames += 1
                if (
                    abs(observation["lane_offset"]) >= 0.03
                    and terms["output"] * observation["lane_offset"] <= 0.0
                ):
                    steering_sign_mismatches += 1
                steering_limit = float(controller_config["steering_limit"])
                if abs(terms["output"]) >= 0.98 * steering_limit:
                    steering_saturation_frames += 1
                if abs(terms["d"]) >= 0.80 * steering_limit:
                    large_derivative_frames += 1
                if abs(terms["output"]) >= 0.04:
                    current_sign = 1 if terms["output"] > 0.0 else -1
                    if (
                        previous_nonzero_steering is not None
                        and current_sign != previous_nonzero_steering
                    ):
                        steering_flip_count += 1
                    previous_nonzero_steering = current_sign

            left_duty = map_duty(left_command, False, motor_config)
            right_duty = map_duty(right_command, True, motor_config)
            if left_duty > 0.0 and not (
                motor_config["left_deadband_duty"]
                <= left_duty
                <= motor_config["left_max_duty"]
            ):
                mapped_duty_violations += 1
            if right_duty > 0.0 and not (
                motor_config["right_deadband_duty"]
                <= right_duty
                <= motor_config["right_max_duty"]
            ):
                mapped_duty_violations += 1
            control_timestamp = time.monotonic()
            if previous_control_timestamp is None:
                control_fps = 0.0
            else:
                control_period = max(
                    0.000001, control_timestamp - previous_control_timestamp
                )
                control_fps = 1.0 / control_period
                fps_values.append(control_fps)
            previous_control_timestamp = control_timestamp
            writer.writerow(
                {
                    "run_id": run_id,
                    "timestamp_utc": utc_now(),
                    "monotonic_s": round(time.monotonic(), 6),
                    "frame_id": frame_number,
                    "mode": "DRY_RUN",
                    "camera_sequence": camera_metadata["sequence"],
                    "camera_pts_ns": camera_metadata["pts_ns"],
                    "camera_age_ms": round(camera_age * 1000.0, 3),
                    "camera_fresh": 1,
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
                    "yolo_age_s": "" if not yolo_state["has_result"] else round(yolo_state["age_seconds"], 3),
                    "yolo_result_age_s": "" if not yolo_state["has_result"] else round(yolo_state["result_age_seconds"], 3),
                    "yolo_source_frame_lag": "" if yolo_state["source_frame_lag"] is None else yolo_state["source_frame_lag"],
                    "yolo_source_frame": yolo_state["source_frame"],
                    "yolo_inference_ms": round(yolo_state["inference_ms"], 3),
                    "yolo_nms_ms": round(yolo_state["nms_ms"], 3),
                    "yolo_detector_ms": round(yolo_state["end_to_end_ms"], 3),
                    "objects": format_detections(active_detections),
                    "obstacle_state": obstacle_action["state"],
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
                    "battery_voltage_v": "",
                    "physical_motor_commands": 0,
                    "control_fps": round(control_fps, 3),
                }
            )
            if frame_number % 20 == 0:
                csv_file.flush()
                print(
                    "Frame {} | {} / {} | offset {:+.3f} | PWM {:.3f}/{:.3f} | {:.2f} FPS".format(
                        frame_number,
                        decision["state"],
                        decision["reason"],
                        observation["lane_offset"],
                        left_duty,
                        right_duty,
                        control_fps,
                    )
                )

            original = observation["original_overlay"]
            if yolo_state["has_result"]:
                draw_detections(
                    original,
                    active_detections,
                    yolo_state["corridor"],
                    path_polygon=observation["source_polygon"],
                )
            colour = (0, 255, 0) if decision["state"] == "DRIVE" else (
                (0, 200, 255) if decision["state"] == "CAUTION" else (0, 0, 255)
            )
            cv2.putText(
                original,
                "{} - {}".format(decision["state"], decision["reason"]),
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                colour,
                2,
            )
            cv2.putText(
                original,
                "SIMULATED PWM L {:.3f} R {:.3f}".format(left_duty, right_duty),
                (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 255),
                2,
            )
            latest_composite = np.hstack(
                [original, observation["warped_overlay"]]
            )
            if not arguments.headless:
                cv2.imshow("Integrated dry run - motors disabled", latest_composite)
                cv2.imshow("IPM mask", observation["mask"])
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    error_text = "OPERATOR_ENDED_BEFORE_FULL_WINDOW"
                    break
    except KeyboardInterrupt:
        error_text = "INTERRUPTED_BY_OPERATOR"
        print("Interrupted.")
    except Exception as error:
        error_text = repr(error)
        print("DRY RUN STOPPED: {}".format(error))
    finally:
        measurement_end = time.monotonic()
        camera.close()
        yolo_worker.stop()
        csv_file.close()
        cv2.destroyAllWindows()

    image_written = bool(
        latest_composite is not None
        and cv2.imwrite(image_path, latest_composite)
    )
    artifact_snapshot_end = capture_artifact_snapshot(
        "integrated_control_dry_run"
    )
    artifact_snapshot_stable = (
        artifact_snapshot_start == artifact_snapshot_end
    )
    total = max(1, frame_number)
    obstacle_stop_frames = sum(
        count
        for reason, count in reason_counts.items()
        if reason.startswith("OBJECT_DETECTED_")
        or reason in ("STOP_SIGN_DETECTED", "TRAFFIC_LIGHT_UNCLASSIFIED")
    )
    lane_stop_frames = sum(
        count
        for reason, count in reason_counts.items()
        if reason.startswith("LANE_")
        or reason.startswith("PARTIAL_LANE_")
    )
    average_fps = statistics.mean(fps_values) if fps_values else 0.0
    full_rate = 100.0 * full_frames / total
    usable_rate = 100.0 * usable_frames / total
    yolo_fresh_rate = 100.0 * yolo_fresh_frames / total
    saturation_rate = (
        100.0 * steering_saturation_frames / controlled_frames
        if controlled_frames
        else 100.0
    )
    derivative_spike_rate = (
        100.0 * large_derivative_frames / controlled_frames
        if controlled_frames
        else 100.0
    )
    steering_flips_per_100 = (
        100.0 * steering_flip_count / controlled_frames
        if controlled_frames
        else 100.0
    )
    measurement_window_completed = bool(
        measurement_end - test_start >= float(arguments.seconds)
    )
    passed = (
        error_text is None
        and measurement_window_completed
        and lane_detector.physically_calibrated
        and default_ipm_config_used
        and frame_number >= 50
        and full_rate >= 80.0
        and usable_rate >= 95.0
        and yolo_fresh_rate >= 90.0
        and safety_counts["DRIVE"] > 0
        and obstacle_stop_frames > 0
        and lane_stop_frames > 0
        and average_fps >= 5.0
        and controlled_frames >= 20
        and steering_sign_mismatches == 0
        and mapped_duty_violations == 0
        and saturation_rate <= 20.0
        and derivative_spike_rate <= 10.0
        and steering_flips_per_100 <= 15.0
        and image_written
        and artifact_snapshot_stable
    )
    summary = {
        "schema_version": 1,
        "test": "INTEGRATED_PERCEPTION_CONTROL_DRY_RUN",
        "passed": passed,
        "error": error_text,
        "physical_motor_commands": False,
        "requested_measurement_window_s": float(arguments.seconds),
        "measurement_window_completed": measurement_window_completed,
        "yolo_device": str(yolo_detector.device),
        "yolo_fp16": bool(yolo_detector.use_half),
        "ipm_calibration_state": lane_detector.config.get("calibration_state"),
        "detector_mode": lane_detector.config.get("detector_mode"),
        "target_line_role": lane_detector.config.get("target_line_role"),
        "default_ipm_config_used": default_ipm_config_used,
        "frames": frame_number,
        "full_lane_rate_pct": full_rate,
        "usable_lane_rate_pct": usable_rate,
        "yolo_fresh_rate_pct": yolo_fresh_rate,
        "average_control_fps": average_fps,
        "mean_yolo_inference_ms": statistics.mean(yolo_inference_values) if yolo_inference_values else None,
        "unique_yolo_results": len(seen_yolo_source_frames),
        "observed_yolo_input_tensor_shapes": [
            list(shape) for shape in sorted(yolo_input_shapes)
        ],
        "yolo_submissions": submitted_frames,
        "safety_state_counts": dict(safety_counts),
        "safety_reason_counts": dict(reason_counts),
        "obstacle_stop_frames": obstacle_stop_frames,
        "lane_stop_frames": lane_stop_frames,
        "controlled_frames": controlled_frames,
        "steering_sign_mismatches": steering_sign_mismatches,
        "mapped_duty_violations": mapped_duty_violations,
        "steering_saturation_rate_pct": saturation_rate,
        "large_derivative_rate_pct": derivative_spike_rate,
        "steering_flips_per_100_controlled_frames": steering_flips_per_100,
        "artifact_snapshot_stable": artifact_snapshot_stable,
        "artifact_sha256_at_test": artifact_snapshot_end,
        "pass_requirements": {
            "physically_calibrated_ipm": True,
            "minimum_frames": 50,
            "minimum_full_lane_rate_pct": 80.0,
            "minimum_usable_lane_rate_pct": 95.0,
            "minimum_yolo_fresh_rate_pct": 90.0,
            "minimum_average_control_fps": 5.0,
            "drive_exercised": True,
            "obstacle_stop_exercised": True,
            "lane_loss_stop_exercised": True,
            "minimum_controlled_frames": 20,
            "maximum_steering_sign_mismatches": 0,
            "maximum_mapped_duty_violations": 0,
            "maximum_steering_saturation_rate_pct": 20.0,
            "maximum_large_derivative_rate_pct": 10.0,
            "maximum_steering_flips_per_100_controlled_frames": 15.0,
        },
        "csv": csv_path,
        "evidence_image": image_path if image_written else None,
        "completed_utc": utc_now(),
    }
    with open(json_path, "w") as summary_file:
        json.dump(summary, summary_file, indent=2, sort_keys=True)
        summary_file.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print("DRY-RUN VALIDATION: {}".format("PASS" if passed else "NOT YET PASSED"))
    print("Evidence: {}".format(json_path))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
