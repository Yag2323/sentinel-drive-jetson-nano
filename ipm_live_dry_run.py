#!/usr/bin/env python3
"""Validate physical-camera IPM and lane offset without importing motor code."""

from __future__ import print_function

import argparse
import csv
import datetime
import json
import os
import statistics
import sys
import time

import cv2
import numpy as np

from gst_camera_bridge import GstCamera
from ipm_lane import DEFAULT_CONFIG_PATH, IPMLaneDetector
from validation_manager import capture_artifact_snapshot


CAMERA_STALE_LIMIT_SECONDS = 0.25
WARMUP_TIMEOUT_SECONDS = 10.0
WARMUP_REQUIRED_FRESH_STREAK = 3


def utc_now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def main():
    parser = argparse.ArgumentParser(description="Read-only live IPM validation")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--expected-centered",
        action="store_true",
        help=(
            "Confirm the stationary robot is manually positioned with the "
            "OUTER black tape directly under the camera/vehicle centre; "
            "required for a passing result."
        ),
    )
    parser.add_argument(
        "--allow-sitl-seed",
        action="store_true",
        help="Permit diagnostics with uncalibrated SITL seed geometry.",
    )
    arguments = parser.parse_args()

    if not 5.0 <= arguments.seconds <= 300.0:
        print("REFUSED: --seconds must be between 5 and 300.")
        return 1

    detector = IPMLaneDetector(arguments.config)
    single_outer_target = (
        detector.config.get("detector_mode") == "SINGLE_LINE_POLYNOMIAL_V1"
        and detector.config.get("target_line_role")
        == "OUTER_CIRCLE_CENTERLINE"
    )
    outer_tape_only_confirmed = False
    if single_outer_target:
        print("SINGLE-LINE PHYSICAL TARGET CHECK - MOTORS ARE NOT ACCESSED")
        print("Remove or completely mask the inner circle before this test.")
        print("Place the OUTER tape directly under the camera/vehicle centre.")
        response = input(
            "Type OUTER-TAPE-ONLY to confirm the inner circle is absent: "
        ).strip()
        outer_tape_only_confirmed = response == "OUTER-TAPE-ONLY"
        if not outer_tape_only_confirmed:
            print("REFUSED: outer-tape-only condition was not confirmed.")
            return 1
    default_config_used = (
        os.path.realpath(os.path.abspath(arguments.config))
        == os.path.realpath(os.path.abspath(DEFAULT_CONFIG_PATH))
    )
    if not detector.physically_calibrated and not arguments.allow_sitl_seed:
        print("REFUSED: IPM is not physically calibrated.")
        print("Run: python calibrate_ipm.py")
        return 1
    artifact_snapshot_start = capture_artifact_snapshot("ipm_live_dry_run")

    project_directory = os.path.dirname(os.path.abspath(__file__))
    evidence_directory = os.path.join(project_directory, "evidence")
    if not os.path.isdir(evidence_directory):
        os.makedirs(evidence_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    csv_path = os.path.join(evidence_directory, "ipm_dry_run_{}.csv".format(stamp))
    json_path = os.path.join(evidence_directory, "ipm_dry_run_{}.json".format(stamp))
    image_path = os.path.join(evidence_directory, "ipm_dry_run_{}.jpg".format(stamp))

    camera = GstCamera(
        sensor_id=0,
        capture_width=1280,
        capture_height=720,
        output_width=640,
        output_height=360,
        framerate=21,
    )

    rows = []
    near_field_offsets = []
    steering_offsets = []
    confidences = []
    fps_values = []
    full_frames = 0
    usable_frames = 0
    latest_composite = None
    error_text = None
    start = None
    warmup_attempts = 0
    warmup_observations = 0
    warmup_fresh_streak = 0
    warmup_maximum_age = 0.0
    warmup_stabilized = False

    csv_file = open(csv_path, "w", newline="")
    fields = (
        "timestamp_utc",
        "monotonic_s",
        "frame",
        "camera_sequence",
        "camera_pts_ns",
        "camera_age_ms",
        "camera_fresh",
        "status",
        "lane_offset",
        "near_field_offset",
        "lookahead_offset",
        "heading_error_rad",
        "curvature_per_px",
        "candidate_count",
        "confidence",
        "lane_width_px",
        "line_count",
        "processing_ms",
        "loop_fps",
    )
    writer = csv.DictWriter(csv_file, fieldnames=fields)
    writer.writeheader()

    print("IPM LIVE DRY RUN - NO MOTOR MODULES IMPORTED")
    print("Calibration state: {}".format(detector.config.get("calibration_state")))

    frame_number = 0
    try:
        print(
            "Stabilising camera/detector for up to {:.1f} seconds; "
            "the {:.3f} s freshness limit is unchanged.".format(
                WARMUP_TIMEOUT_SECONDS,
                CAMERA_STALE_LIMIT_SECONDS,
            )
        )
        warmup_deadline = time.monotonic() + WARMUP_TIMEOUT_SECONDS
        while (
            time.monotonic() < warmup_deadline
            and warmup_fresh_streak < WARMUP_REQUIRED_FRESH_STREAK
        ):
            success, frame, camera_metadata = camera.read_with_metadata(
                timeout_seconds=2.0
            )
            warmup_attempts += 1
            if (
                not success
                or camera_metadata is None
                or not camera_metadata["timestamp_valid"]
                or not camera_metadata["source_age_trustworthy"]
                or not camera_metadata["fresh"]
            ):
                warmup_fresh_streak = 0
                continue
            camera_age = (
                time.monotonic() - camera_metadata["capture_monotonic"]
            )
            if camera_age > CAMERA_STALE_LIMIT_SECONDS:
                warmup_fresh_streak = 0
                warmup_maximum_age = max(warmup_maximum_age, camera_age)
                continue
            detector.observe(frame)
            warmup_observations += 1
            camera_age = (
                time.monotonic() - camera_metadata["capture_monotonic"]
            )
            warmup_maximum_age = max(warmup_maximum_age, camera_age)
            if camera_age <= CAMERA_STALE_LIMIT_SECONDS:
                warmup_fresh_streak += 1
            else:
                warmup_fresh_streak = 0
        warmup_stabilized = (
            warmup_fresh_streak >= WARMUP_REQUIRED_FRESH_STREAK
        )
        if not warmup_stabilized:
            raise RuntimeError(
                "Camera/detector did not stabilise below the unchanged "
                "{:.3f} s freshness limit.".format(
                    CAMERA_STALE_LIMIT_SECONDS
                )
            )
        print(
            "Warm-up complete after {} attempts; starting the evidence "
            "window now.".format(warmup_attempts)
        )
        start = time.monotonic()
        while time.monotonic() - start < arguments.seconds:
            loop_start = time.monotonic()
            success, frame, camera_metadata = camera.read_with_metadata(
                timeout_seconds=2.0
            )
            if not success:
                raise RuntimeError("Camera frame unavailable; fail-safe stop.")
            if (
                camera_metadata is None
                or not camera_metadata["timestamp_valid"]
                or not camera_metadata["source_age_trustworthy"]
                or not camera_metadata["fresh"]
            ):
                raise RuntimeError("Camera frame timestamp is invalid or repeated.")
            camera_age = time.monotonic() - camera_metadata["capture_monotonic"]
            if camera_age > CAMERA_STALE_LIMIT_SECONDS:
                raise RuntimeError(
                    "Camera frame is stale: {:.3f} seconds.".format(camera_age)
                )

            process_start = time.monotonic()
            observation = detector.observe(frame)
            processing_ms = (time.monotonic() - process_start) * 1000.0
            camera_age = time.monotonic() - camera_metadata["capture_monotonic"]
            if camera_age > CAMERA_STALE_LIMIT_SECONDS:
                raise RuntimeError(
                    "Camera observation became stale: {:.3f} seconds."
                    .format(camera_age)
                )
            frame_number += 1
            status = observation["status"]
            confidences.append(float(observation["confidence"]))
            if status == "FULL":
                full_frames += 1
            if status == "FULL" or status.startswith("PARTIAL"):
                usable_frames += 1
                near_field_offsets.append(
                    abs(float(observation["near_field_offset"]))
                )
                steering_offsets.append(abs(float(observation["lane_offset"])))

            loop_time = time.monotonic() - loop_start
            loop_fps = 1.0 / loop_time if loop_time > 0 else 0.0
            fps_values.append(loop_fps)
            row = {
                "timestamp_utc": utc_now(),
                "monotonic_s": round(time.monotonic(), 6),
                "frame": frame_number,
                "camera_sequence": camera_metadata["sequence"],
                "camera_pts_ns": camera_metadata["pts_ns"],
                "camera_age_ms": round(camera_age * 1000.0, 3),
                "camera_fresh": 1,
                "status": status,
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
                "confidence": round(observation["confidence"], 4),
                "lane_width_px": "" if observation["lane_width_px"] is None else round(observation["lane_width_px"], 3),
                "line_count": observation["line_count"],
                "processing_ms": round(processing_ms, 3),
                "loop_fps": round(loop_fps, 3),
            }
            rows.append(row)
            writer.writerow(row)
            if frame_number % 10 == 0:
                csv_file.flush()

            original = observation["original_overlay"]
            warped = observation["warped_overlay"]
            cv2.putText(
                original,
                "{} offset {:+.3f} confidence {:.2f}".format(
                    status, observation["lane_offset"], observation["confidence"]
                ),
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 255),
                2,
            )
            latest_composite = np.hstack([original, warped])
            if not arguments.headless:
                cv2.imshow("IPM dry run - original / bird's-eye", latest_composite)
                cv2.imshow("IPM black-tape mask", observation["mask"])
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    error_text = "OPERATOR_ENDED_BEFORE_FULL_WINDOW"
                    break

    except KeyboardInterrupt:
        error_text = "INTERRUPTED_BY_OPERATOR"
        print("Interrupted.")
    except Exception as error:
        error_text = repr(error)
        print("IPM DRY RUN STOPPED: {}".format(error))
    finally:
        measurement_end = time.monotonic()
        camera.close()
        csv_file.close()
        cv2.destroyAllWindows()

    image_written = bool(
        latest_composite is not None
        and cv2.imwrite(image_path, latest_composite)
    )
    artifact_snapshot_end = capture_artifact_snapshot("ipm_live_dry_run")
    artifact_snapshot_stable = (
        artifact_snapshot_start == artifact_snapshot_end
    )

    processed = max(1, frame_number)
    full_rate = 100.0 * full_frames / processed
    usable_rate = 100.0 * usable_frames / processed
    measurement_elapsed = (
        measurement_end - start if start is not None else 0.0
    )
    average_fps = (
        float(frame_number) / measurement_elapsed
        if measurement_elapsed > 0.0
        else 0.0
    )
    mean_instantaneous_fps = (
        statistics.mean(fps_values) if fps_values else 0.0
    )
    measurement_window_completed = bool(
        start is not None
        and measurement_elapsed >= float(arguments.seconds)
    )
    passed = (
        error_text is None
        and measurement_window_completed
        and detector.physically_calibrated
        and default_config_used
        and frame_number >= 50
        and full_rate >= 90.0
        and usable_rate >= 95.0
        and average_fps >= 5.0
        and image_written
        and artifact_snapshot_stable
        and warmup_stabilized
        and (not single_outer_target or outer_tape_only_confirmed)
    )
    mean_offset = (
        statistics.mean(near_field_offsets) if near_field_offsets else None
    )
    mean_steering_offset = (
        statistics.mean(steering_offsets) if steering_offsets else None
    )
    mean_confidence = statistics.mean(confidences) if confidences else 0.0
    passed = (
        passed
        and arguments.expected_centered
        and mean_offset is not None
        and mean_offset <= 0.08
        and mean_confidence >= 0.55
    )
    summary = {
        "schema_version": 1,
        "test": "PHYSICAL_CSI_IPM_DRY_RUN",
        "passed": passed,
        "error": error_text,
        "motor_commands": False,
        "calibration_state": detector.config.get("calibration_state"),
        "detector_mode": detector.config.get("detector_mode"),
        "target_line_role": detector.config.get("target_line_role"),
        "outer_tape_only_operator_confirmed": bool(
            outer_tape_only_confirmed
        ),
        "default_ipm_config_used": default_config_used,
        "frames": frame_number,
        "requested_measurement_window_s": float(arguments.seconds),
        "measurement_elapsed_s": measurement_elapsed,
        "measurement_window_completed": measurement_window_completed,
        "warmup_timeout_s": WARMUP_TIMEOUT_SECONDS,
        "warmup_required_fresh_streak": WARMUP_REQUIRED_FRESH_STREAK,
        "warmup_attempts": warmup_attempts,
        "warmup_detector_observations": warmup_observations,
        "warmup_final_fresh_streak": warmup_fresh_streak,
        "warmup_maximum_camera_age_s": warmup_maximum_age,
        "warmup_stabilized": warmup_stabilized,
        "full_lane_rate_pct": full_rate,
        "usable_lane_rate_pct": usable_rate,
        "expected_centered_reference": bool(arguments.expected_centered),
        "mean_absolute_offset": mean_offset,
        "centred_reference_metric": "near_field_offset",
        "mean_absolute_steering_offset": mean_steering_offset,
        "mean_lane_confidence": mean_confidence,
        "average_loop_fps": average_fps,
        "mean_instantaneous_loop_fps": mean_instantaneous_fps,
        "artifact_snapshot_stable": artifact_snapshot_stable,
        "artifact_sha256_at_test": artifact_snapshot_end,
        "pass_requirements": {
            "physically_calibrated_ipm": True,
            "minimum_frames": 50,
            "minimum_full_lane_rate_pct": 90.0,
            "minimum_usable_lane_rate_pct": 95.0,
            "minimum_average_loop_fps": 5.0,
            "warmup_stabilized_below_camera_age_s": (
                CAMERA_STALE_LIMIT_SECONDS
            ),
            "expected_centered_reference": True,
            "maximum_mean_absolute_offset": 0.08,
            "minimum_mean_lane_confidence": 0.55,
            "outer_tape_only_operator_confirmation": bool(
                single_outer_target
            ),
        },
        "csv": csv_path,
        "evidence_image": image_path if image_written else None,
        "completed_utc": utc_now(),
    }
    with open(json_path, "w") as summary_file:
        json.dump(summary, summary_file, indent=2, sort_keys=True)
        summary_file.write("\n")

    print(json.dumps(summary, indent=2, sort_keys=True))
    print("IPM VALIDATION: {}".format("PASS" if passed else "NOT YET PASSED"))
    print("Evidence: {}".format(json_path))
    if frame_number == 0:
        return 1
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
