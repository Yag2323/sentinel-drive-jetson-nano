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
            "Confirm the stationary robot centreline is manually aligned to "
            "the lane centre; required for a passing calibration result."
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
    offsets = []
    confidences = []
    fps_values = []
    full_frames = 0
    usable_frames = 0
    latest_composite = None
    error_text = None
    start = time.monotonic()

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
            if camera_age > 0.25:
                raise RuntimeError(
                    "Camera frame is stale: {:.3f} seconds.".format(camera_age)
                )

            process_start = time.monotonic()
            observation = detector.observe(frame)
            processing_ms = (time.monotonic() - process_start) * 1000.0
            camera_age = time.monotonic() - camera_metadata["capture_monotonic"]
            if camera_age > 0.25:
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
                offsets.append(abs(observation["lane_offset"]))

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
    average_fps = statistics.mean(fps_values) if fps_values else 0.0
    measurement_window_completed = bool(
        measurement_end - start >= float(arguments.seconds)
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
    )
    mean_offset = statistics.mean(offsets) if offsets else None
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
        "default_ipm_config_used": default_config_used,
        "frames": frame_number,
        "requested_measurement_window_s": float(arguments.seconds),
        "measurement_window_completed": measurement_window_completed,
        "full_lane_rate_pct": full_rate,
        "usable_lane_rate_pct": usable_rate,
        "expected_centered_reference": bool(arguments.expected_centered),
        "mean_absolute_offset": mean_offset,
        "mean_lane_confidence": mean_confidence,
        "average_loop_fps": average_fps,
        "artifact_snapshot_stable": artifact_snapshot_stable,
        "artifact_sha256_at_test": artifact_snapshot_end,
        "pass_requirements": {
            "physically_calibrated_ipm": True,
            "minimum_frames": 50,
            "minimum_full_lane_rate_pct": 90.0,
            "minimum_usable_lane_rate_pct": 95.0,
            "minimum_average_loop_fps": 5.0,
            "expected_centered_reference": True,
            "maximum_mean_absolute_offset": 0.08,
            "minimum_mean_lane_confidence": 0.55,
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
