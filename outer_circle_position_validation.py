#!/usr/bin/env python3
"""Motor-free validation of four representative outer-circle positions.

The operator repositions the stationary vehicle by hand.  This program never
imports the motor stack and never exposes a motor command.  Every captured
sample must independently satisfy the live-motion perception contract.

Compatible with Python 3.6.
"""

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
from single_line_lane import SINGLE_LINE_DETECTOR_MODE
from validation_manager import capture_artifact_snapshot


TEST_NAME = "OUTER_CIRCLE_REPRESENTATIVE_POSITIONS"
TARGET_LINE_ROLE = "OUTER_CIRCLE_CENTERLINE"
CAMERA_STALE_LIMIT_SECONDS = 0.25
MINIMUM_CONFIDENCE = 0.55
STRICT_NEAR_FIELD_LIMIT = 0.25
PREFLIGHT_TIMEOUT_SECONDS = 10.0
PREFLIGHT_REQUIRED_FRESH_STREAK = 3
POSITIONS = (
    (
        "STRAIGHT",
        "Place the car tangent to a straight section with the OUTER tape "
        "under the vehicle/camera centre.",
    ),
    (
        "CURVE_ENTRY",
        "Place the car tangent where the straight first begins to bend; "
        "keep the OUTER tape under the vehicle/camera centre.",
    ),
    (
        "CURVE_APEX",
        "Place the car tangent at the roundest part of the curve with the "
        "OUTER tape under the vehicle/camera centre.",
    ),
    (
        "CURVE_EXIT",
        "Place the car tangent where the curve becomes straight again; "
        "keep the OUTER tape under the vehicle/camera centre.",
    ),
)
FORBIDDEN_MOTOR_MODULES = (
    "manual_motor_control",
    "motor_mapping",
    "safe_motor_output",
)


def utc_now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def forbidden_motor_modules_loaded():
    return sorted(
        name for name in FORBIDDEN_MOTOR_MODULES if name in sys.modules
    )


def metadata_is_trustworthy(metadata):
    return bool(
        metadata is not None
        and metadata.get("timestamp_valid")
        and metadata.get("source_age_trustworthy")
        and metadata.get("fresh")
        and metadata.get("capture_monotonic") is not None
    )


def make_composite(observation, position_name, sample_passed, camera_age):
    original = observation["original_overlay"].copy()
    warped = observation["warped_overlay"].copy()
    colour = (40, 220, 70) if sample_passed else (40, 40, 255)
    text = (
        "{} | {} | conf {:.3f} | near {:+.3f} | candidates {} | age {:.1f} ms"
    ).format(
        position_name,
        observation["status"],
        float(observation["confidence"]),
        float(observation.get("near_field_offset", 0.0)),
        int(observation.get("candidate_count", 0)),
        camera_age * 1000.0,
    )
    for image in (original, warped):
        cv2.rectangle(image, (0, 0), (image.shape[1] - 1, 38), (4, 8, 14), -1)
        cv2.putText(
            image,
            text,
            (8, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            colour,
            1,
        )
    return np.hstack([original, warped])


def sample_reasons(trustworthy, camera_age, observation, near_limit):
    reasons = []
    if not trustworthy:
        reasons.append("UNTRUSTWORTHY_CAMERA_TIMESTAMP")
    if camera_age is None or camera_age > CAMERA_STALE_LIMIT_SECONDS:
        reasons.append("STALE_CAMERA_OBSERVATION")
    if observation is None:
        reasons.append("NO_LANE_OBSERVATION")
        return reasons
    if observation.get("status") != "FULL":
        reasons.append("STATUS_NOT_FULL")
    if float(observation.get("confidence", 0.0)) < MINIMUM_CONFIDENCE:
        reasons.append("LOW_CONFIDENCE")
    if int(observation.get("candidate_count", 0)) != 1:
        reasons.append("CANDIDATE_COUNT_NOT_ONE")
    near_offset = observation.get("near_field_offset")
    if near_offset is None or abs(float(near_offset)) > near_limit:
        reasons.append("NEAR_FIELD_OFFSET_OUT_OF_RANGE")
    return reasons


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Motor-free four-position validation for the outer-circle "
            "single-line detector"
        )
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--seconds-per-position", type=float, default=5.0)
    parser.add_argument("--minimum-samples", type=int, default=10)
    parser.add_argument("--headless", action="store_true")
    arguments = parser.parse_args()

    if not 2.0 <= arguments.seconds_per_position <= 20.0:
        print("REFUSED: --seconds-per-position must be between 2 and 20.")
        return 1
    if not 10 <= arguments.minimum_samples <= 100:
        print("REFUSED: --minimum-samples must be between 10 and 100.")
        return 1
    loaded_motor_modules = forbidden_motor_modules_loaded()
    if loaded_motor_modules:
        print("REFUSED: motor modules were imported: {}".format(
            ", ".join(loaded_motor_modules)
        ))
        return 1

    detector = IPMLaneDetector(arguments.config)
    default_config_used = (
        os.path.realpath(os.path.abspath(arguments.config))
        == os.path.realpath(os.path.abspath(DEFAULT_CONFIG_PATH))
    )
    if not default_config_used:
        print("REFUSED: gate evidence requires the default ipm_config.json.")
        return 1
    if not detector.physically_calibrated:
        print("REFUSED: IPM configuration is not physically calibrated.")
        return 1
    if detector.config.get("detector_mode") != SINGLE_LINE_DETECTOR_MODE:
        print("REFUSED: the active detector is not the single-line build.")
        return 1
    if detector.config.get("target_line_role") != TARGET_LINE_ROLE:
        print("REFUSED: target_line_role is not OUTER_CIRCLE_CENTERLINE.")
        return 1

    configured_near_limit = float(
        detector.config.get(
            "single_line_maximum_near_field_offset",
            STRICT_NEAR_FIELD_LIMIT,
        )
    )
    near_limit = min(STRICT_NEAR_FIELD_LIMIT, configured_near_limit)

    print("=" * 72)
    print("OUTER-CIRCLE REPRESENTATIVE-POSITION VALIDATION")
    print("MOTOR MODULES ARE NOT IMPORTED; MOTOR BATTERY MUST BE DISCONNECTED.")
    print("Remove the inner circle or mask every part of it completely.")
    print("Each captured sample must be FULL, fresh, unambiguous and centred.")
    print("Near-field limit: +/-{:.3f}; confidence minimum: {:.2f}.".format(
        near_limit, MINIMUM_CONFIDENCE
    ))
    print("=" * 72)
    motor_battery_response = input(
        "Type MOTOR-BATTERY-DISCONNECTED after physically disconnecting it: "
    ).strip()
    motor_battery_disconnected_confirmed = (
        motor_battery_response == "MOTOR-BATTERY-DISCONNECTED"
    )
    if not motor_battery_disconnected_confirmed:
        print("REFUSED: disconnected motor battery was not confirmed.")
        return 1
    response = input(
        "Type OUTER-TAPE-ONLY after removing/masking the inner circle: "
    ).strip()
    outer_tape_only_confirmed = response == "OUTER-TAPE-ONLY"
    if not outer_tape_only_confirmed:
        print("REFUSED: outer-tape-only condition was not confirmed.")
        return 1

    project_directory = os.path.dirname(os.path.abspath(__file__))
    evidence_directory = os.path.join(project_directory, "evidence")
    if not os.path.isdir(evidence_directory):
        os.makedirs(evidence_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    prefix = os.path.join(
        evidence_directory, "outer_circle_positions_{}".format(stamp)
    )
    csv_path = prefix + ".csv"
    json_path = prefix + ".json"

    artifact_snapshot_start = capture_artifact_snapshot(
        "outer_circle_positions"
    )
    fields = (
        "timestamp_utc",
        "position",
        "sample",
        "camera_sequence",
        "camera_age_ms",
        "camera_trustworthy",
        "status",
        "confidence",
        "candidate_count",
        "near_field_offset",
        "lookahead_offset",
        "heading_error_rad",
        "curvature_per_px",
        "sample_passed",
        "failure_reasons",
    )
    csv_file = open(csv_path, "w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=fields)
    writer.writeheader()

    camera = None
    position_results = []
    image_paths = []
    run_error = None
    try:
        camera = GstCamera(
            sensor_id=0,
            capture_width=1280,
            capture_height=720,
            output_width=640,
            output_height=360,
            framerate=21,
        )
        for position_name, instruction in POSITIONS:
            print("\n{}: {}".format(position_name, instruction))
            print("Disconnect motor power before repositioning; do not push "
                  "the powered vehicle.")
            expected_confirmation = "READY-{}".format(position_name)
            if input("Type {} to measure: ".format(
                expected_confirmation
            )).strip() != expected_confirmation:
                raise RuntimeError(
                    "{}_CONFIRMATION_REFUSED".format(position_name)
                )

            print(
                "{}: Stabilising for up to {:.1f} seconds; no preflight "
                "sample is counted as evidence.".format(
                    position_name, PREFLIGHT_TIMEOUT_SECONDS
                )
            )
            preflight_deadline = time.monotonic() + PREFLIGHT_TIMEOUT_SECONDS
            preflight_attempts = 0
            preflight_observations = 0
            preflight_fresh_streak = 0
            preflight_maximum_age = 0.0
            preflight_last_reasons = []
            while (
                time.monotonic() < preflight_deadline
                and preflight_fresh_streak
                < PREFLIGHT_REQUIRED_FRESH_STREAK
            ):
                success, frame, metadata = camera.read_with_metadata(
                    timeout_seconds=2.0
                )
                preflight_attempts += 1
                trustworthy = bool(
                    success and metadata_is_trustworthy(metadata)
                )
                observation = None
                camera_age = None
                if trustworthy:
                    observation = detector.observe(frame)
                    preflight_observations += 1
                    camera_age = (
                        time.monotonic() - metadata["capture_monotonic"]
                    )
                    preflight_maximum_age = max(
                        preflight_maximum_age, camera_age
                    )
                preflight_last_reasons = sample_reasons(
                    trustworthy, camera_age, observation, near_limit
                )
                if preflight_last_reasons:
                    preflight_fresh_streak = 0
                else:
                    preflight_fresh_streak += 1
            preflight_stabilized = bool(
                preflight_fresh_streak
                >= PREFLIGHT_REQUIRED_FRESH_STREAK
            )
            if not preflight_stabilized:
                raise RuntimeError(
                    "{} did not produce {} consecutive valid observations "
                    "within {:.1f} seconds. Last reasons: {}".format(
                        position_name,
                        PREFLIGHT_REQUIRED_FRESH_STREAK,
                        PREFLIGHT_TIMEOUT_SECONDS,
                        "|".join(preflight_last_reasons) or "UNKNOWN",
                    )
                )
            print(
                "{}: Preflight ready after {} attempts; starting the "
                "evidence window now.".format(
                    position_name, preflight_attempts
                )
            )
            deadline = time.monotonic() + arguments.seconds_per_position
            sample_count = 0
            trustworthy_count = 0
            full_count = 0
            stale_count = 0
            failed_count = 0
            confidences = []
            near_offsets = []
            maximum_age = 0.0
            latest_composite = None
            position_interrupted = False
            while time.monotonic() < deadline:
                success, frame, metadata = camera.read_with_metadata(
                    timeout_seconds=2.0
                )
                sample_count += 1
                observation = None
                trustworthy = bool(success and metadata_is_trustworthy(metadata))
                if trustworthy:
                    trustworthy_count += 1
                    observation = detector.observe(frame)
                    camera_age = (
                        time.monotonic() - metadata["capture_monotonic"]
                    )
                    maximum_age = max(maximum_age, camera_age)
                else:
                    camera_age = None

                reasons = sample_reasons(
                    trustworthy, camera_age, observation, near_limit
                )
                sample_passed = not reasons
                if not sample_passed:
                    failed_count += 1
                if camera_age is None or camera_age > CAMERA_STALE_LIMIT_SECONDS:
                    stale_count += 1
                if observation is not None:
                    if observation.get("status") == "FULL":
                        full_count += 1
                    confidences.append(float(observation["confidence"]))
                    near_offsets.append(
                        abs(float(observation["near_field_offset"]))
                    )
                    latest_composite = make_composite(
                        observation,
                        position_name,
                        sample_passed,
                        camera_age,
                    )
                    if not arguments.headless:
                        cv2.imshow(
                            "Outer-circle position validation",
                            latest_composite,
                        )
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            position_interrupted = True
                            break

                row = {
                    "timestamp_utc": utc_now(),
                    "position": position_name,
                    "sample": sample_count,
                    "camera_sequence": (
                        "" if metadata is None else metadata.get("sequence", "")
                    ),
                    "camera_age_ms": (
                        "" if camera_age is None
                        else round(camera_age * 1000.0, 3)
                    ),
                    "camera_trustworthy": int(trustworthy),
                    "status": (
                        "NO_OBSERVATION" if observation is None
                        else observation["status"]
                    ),
                    "confidence": (
                        "" if observation is None
                        else round(float(observation["confidence"]), 6)
                    ),
                    "candidate_count": (
                        "" if observation is None
                        else int(observation.get("candidate_count", 0))
                    ),
                    "near_field_offset": (
                        "" if observation is None
                        else round(float(observation["near_field_offset"]), 7)
                    ),
                    "lookahead_offset": (
                        "" if observation is None
                        else round(float(observation["lookahead_offset"]), 7)
                    ),
                    "heading_error_rad": (
                        "" if observation is None
                        or observation.get("heading_error_rad") is None
                        else round(float(observation["heading_error_rad"]), 8)
                    ),
                    "curvature_per_px": (
                        "" if observation is None
                        or observation.get("curvature_per_px") is None
                        else round(float(observation["curvature_per_px"]), 10)
                    ),
                    "sample_passed": int(sample_passed),
                    "failure_reasons": "|".join(reasons),
                }
                writer.writerow(row)
                csv_file.flush()

            image_path = prefix + "_{}.jpg".format(position_name.lower())
            image_written = bool(
                latest_composite is not None
                and cv2.imwrite(image_path, latest_composite)
            )
            if image_written:
                image_paths.append(image_path)
            completed_window = bool(
                not position_interrupted
                and time.monotonic() >= deadline
            )
            position_passed = bool(
                completed_window
                and sample_count >= arguments.minimum_samples
                and trustworthy_count == sample_count
                and full_count == sample_count
                and stale_count == 0
                and failed_count == 0
                and image_written
            )
            result = {
                "position": position_name,
                "passed": position_passed,
                "measurement_window_completed": completed_window,
                "samples": sample_count,
                "trustworthy_samples": trustworthy_count,
                "full_samples": full_count,
                "failed_samples": failed_count,
                "stale_samples": stale_count,
                "minimum_confidence": (
                    min(confidences) if confidences else None
                ),
                "maximum_absolute_near_field_offset": (
                    max(near_offsets) if near_offsets else None
                ),
                "maximum_camera_age_s": (
                    maximum_age if trustworthy_count else None
                ),
                "preflight_stabilized": preflight_stabilized,
                "preflight_timeout_s": PREFLIGHT_TIMEOUT_SECONDS,
                "preflight_required_fresh_streak": (
                    PREFLIGHT_REQUIRED_FRESH_STREAK
                ),
                "preflight_attempts": preflight_attempts,
                "preflight_detector_observations": preflight_observations,
                "preflight_final_fresh_streak": preflight_fresh_streak,
                "preflight_maximum_camera_age_s": preflight_maximum_age,
                "evidence_image": image_path if image_written else None,
            }
            position_results.append(result)
            print("{}: {} ({} samples, {} failures)".format(
                position_name,
                "PASS" if position_passed else "FAIL",
                sample_count,
                failed_count,
            ))
    except KeyboardInterrupt:
        run_error = "INTERRUPTED_BY_OPERATOR"
    except Exception as error:
        run_error = repr(error)
        print("VALIDATION STOPPED: {}".format(error))
    finally:
        if camera is not None:
            camera.close()
        csv_file.close()
        cv2.destroyAllWindows()

    loaded_motor_modules = forbidden_motor_modules_loaded()
    artifact_snapshot_end = capture_artifact_snapshot(
        "outer_circle_positions"
    )
    artifact_snapshot_stable = (
        artifact_snapshot_start == artifact_snapshot_end
    )
    expected_positions = [item[0] for item in POSITIONS]
    passed = bool(
        run_error is None
        and not loaded_motor_modules
        and motor_battery_disconnected_confirmed
        and outer_tape_only_confirmed
        and [result["position"] for result in position_results]
        == expected_positions
        and all(result["passed"] for result in position_results)
        and artifact_snapshot_stable
    )
    summary = {
        "schema_version": 1,
        "test": TEST_NAME,
        "passed": passed,
        "error": run_error,
        "physical_motor_commands": False,
        "motor_commands": False,
        "motor_modules_imported": bool(loaded_motor_modules),
        "loaded_motor_modules": loaded_motor_modules,
        "operator_confirmed_motor_battery_disconnected": bool(
            motor_battery_disconnected_confirmed
        ),
        "outer_tape_only_operator_confirmed": outer_tape_only_confirmed,
        "default_ipm_config_used": default_config_used,
        "calibration_state": detector.config.get("calibration_state"),
        "detector_mode": detector.config.get("detector_mode"),
        "target_line_role": detector.config.get("target_line_role"),
        "positions_expected": expected_positions,
        "positions": position_results,
        "seconds_per_position": arguments.seconds_per_position,
        "minimum_samples_per_position": arguments.minimum_samples,
        "minimum_confidence": MINIMUM_CONFIDENCE,
        "maximum_camera_age_s": CAMERA_STALE_LIMIT_SECONDS,
        "maximum_absolute_near_field_offset": near_limit,
        "all_positions_passed": bool(
            len(position_results) == len(POSITIONS)
            and all(result["passed"] for result in position_results)
        ),
        "artifact_snapshot_stable": artifact_snapshot_stable,
        "artifact_sha256_at_test": artifact_snapshot_end,
        "csv": csv_path,
        "evidence_images": image_paths,
        "completed_utc": utc_now(),
    }
    with open(json_path, "w") as output_file:
        json.dump(summary, output_file, indent=2, sort_keys=True)
        output_file.write("\n")

    print("\n" + "=" * 72)
    print("OUTER-CIRCLE POSITION VALIDATION: {}".format(
        "PASS" if passed else "NOT YET PASSED"
    ))
    print("Evidence: {}".format(json_path))
    print("CSV: {}".format(csv_path))
    print("=" * 72)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
