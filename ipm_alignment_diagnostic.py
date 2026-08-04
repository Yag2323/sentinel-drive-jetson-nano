#!/usr/bin/env python3
"""Motor-free CSI/IPM alignment diagnostic.

This utility deliberately does not import the motor stack or validation manager.
It tolerates cold-start frames for diagnosis, but it does not relax or replace
the 250 ms freshness requirement used by the validated motion path.
"""

from __future__ import print_function

import argparse
import collections
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


CAMERA_STALE_LIMIT_SECONDS = 0.25
LANE_CONFIDENCE_MINIMUM = 0.55


def utc_now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def metadata_is_trustworthy(metadata):
    return bool(
        metadata is not None
        and metadata.get("timestamp_valid")
        and metadata.get("source_age_trustworthy")
        and metadata.get("fresh")
        and metadata.get("capture_monotonic") is not None
    )


def make_composite(observation, post_age_seconds, processing_ms, phase):
    original = observation["original_overlay"].copy()
    warped = observation["warped_overlay"].copy()
    mask_bgr = cv2.cvtColor(observation["mask"], cv2.COLOR_GRAY2BGR)
    edges_bgr = cv2.cvtColor(observation["edges"], cv2.COLOR_GRAY2BGR)

    status = observation["status"]
    status_colour = (40, 220, 70) if status == "FULL" else (0, 190, 255)
    if status in (
        "LOST",
        "INVALID_WIDTH",
        "INVALID_GEOMETRY",
        "AMBIGUOUS",
        "RUN_OVERFLOW",
    ):
        status_colour = (50, 50, 255)

    line_one = "{} | {} | conf {:.3f} | steer {:+.3f}".format(
        phase,
        status,
        float(observation["confidence"]),
        float(observation["lane_offset"]),
    )
    line_two = "support {} | near {:+.3f} | candidates {} | {:.1f} ms / age {:.1f} ms".format(
        int(observation["line_count"]),
        float(observation.get("near_field_offset", 0.0)),
        int(observation.get("candidate_count", 0)),
        float(processing_ms),
        float(post_age_seconds) * 1000.0,
    )
    for image in (original, warped):
        cv2.rectangle(image, (0, 0), (image.shape[1] - 1, 58), (4, 8, 14), -1)
        cv2.putText(
            image,
            line_one,
            (10, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.54,
            status_colour,
            2,
        )
        cv2.putText(
            image,
            line_two,
            (10, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (220, 220, 220),
            1,
        )

    cv2.putText(
        mask_bgr,
        "BLACK-TAPE MASK",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        edges_bgr,
        "HOUGH EDGE INPUT",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )

    top = np.hstack([original, warped])
    bottom = np.hstack([mask_bgr, edges_bgr])
    return np.vstack([top, bottom])


def write_image(path, image):
    return bool(image is not None and cv2.imwrite(path, image))


def main():
    parser = argparse.ArgumentParser(
        description="Motor-free CSI/IPM alignment diagnostic"
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--warmup-seconds", type=float, default=4.0)
    parser.add_argument("--headless", action="store_true")
    arguments = parser.parse_args()

    if not 3.0 <= arguments.seconds <= 60.0:
        print("REFUSED: --seconds must be between 3 and 60.")
        return 1
    if not 1.0 <= arguments.warmup_seconds <= 15.0:
        print("REFUSED: --warmup-seconds must be between 1 and 15.")
        return 1

    detector = IPMLaneDetector(arguments.config)
    if not detector.physically_calibrated:
        print("REFUSED: IPM configuration is not physically calibrated.")
        return 1

    project_directory = os.path.dirname(os.path.abspath(__file__))
    output_directory = os.path.join(project_directory, "diagnostics")
    if not os.path.isdir(output_directory):
        os.makedirs(output_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    prefix = os.path.join(output_directory, "ipm_alignment_{}".format(stamp))
    best_path = prefix + "_best.jpg"
    last_path = prefix + "_last.jpg"
    mask_path = prefix + "_last_mask.png"
    summary_path = prefix + ".json"

    print("=" * 68)
    print("IPM ALIGNMENT DIAGNOSTIC - MOTOR MODULES ARE NOT IMPORTED")
    print("DIAGNOSTIC ONLY: this cannot satisfy a validation gate.")
    print("The live-motion freshness limit remains {:.0f} ms.".format(
        CAMERA_STALE_LIMIT_SECONDS * 1000.0
    ))
    print("Remove any blanket or camera cover before continuing.")
    print("Remove or fully mask the inner circle for single-line validation.")
    print("Place the OUTER tape under the camera/vehicle centre.")
    print("Keep the motor battery physically disconnected.")
    print("=" * 68)

    camera = None
    statuses = collections.Counter()
    processing_values = []
    post_age_values = []
    trustworthy_frames = 0
    observations = 0
    lane_camera_ready_frames = 0
    stale_after_processing = 0
    best_score = None
    best_composite = None
    last_composite = None
    last_mask = None
    last_observation = None
    warmup_observations = 0
    error_text = None

    try:
        camera = GstCamera(
            sensor_id=0,
            capture_width=1280,
            capture_height=720,
            output_width=640,
            output_height=360,
            framerate=21,
        )

        print("Cold-start warm-up for {:.1f} seconds...".format(
            arguments.warmup_seconds
        ))
        warmup_deadline = time.monotonic() + arguments.warmup_seconds
        while time.monotonic() < warmup_deadline:
            success, frame, metadata = camera.read_with_metadata(
                timeout_seconds=2.0
            )
            if not success:
                continue
            if not metadata_is_trustworthy(metadata):
                continue
            process_start = time.monotonic()
            observation = detector.observe(frame)
            processing_ms = (time.monotonic() - process_start) * 1000.0
            post_age = time.monotonic() - metadata["capture_monotonic"]
            warmup_observations += 1
            last_composite = make_composite(
                observation, post_age, processing_ms, "WARM-UP"
            )
            last_mask = observation["mask"].copy()
            last_observation = observation

        print("Warm-up observations: {}".format(warmup_observations))
        print("Measuring alignment for {:.1f} seconds...".format(
            arguments.seconds
        ))
        measurement_start = time.monotonic()
        measurement_deadline = measurement_start + arguments.seconds
        while time.monotonic() < measurement_deadline:
            success, frame, metadata = camera.read_with_metadata(
                timeout_seconds=2.0
            )
            if not success:
                continue
            trustworthy = metadata_is_trustworthy(metadata)
            if trustworthy:
                trustworthy_frames += 1

            process_start = time.monotonic()
            observation = detector.observe(frame)
            processing_ms = (time.monotonic() - process_start) * 1000.0
            post_age = (
                time.monotonic() - metadata["capture_monotonic"]
                if trustworthy
                else float("inf")
            )

            observations += 1
            status = observation["status"]
            confidence = float(observation["confidence"])
            statuses[status] += 1
            processing_values.append(processing_ms)
            if post_age != float("inf"):
                post_age_values.append(post_age)
            if post_age > CAMERA_STALE_LIMIT_SECONDS:
                stale_after_processing += 1

            lane_camera_ready = bool(
                trustworthy
                and post_age <= CAMERA_STALE_LIMIT_SECONDS
                and status == "FULL"
                and confidence >= LANE_CONFIDENCE_MINIMUM
            )
            if lane_camera_ready:
                lane_camera_ready_frames += 1

            composite = make_composite(
                observation, post_age, processing_ms, "MEASUREMENT"
            )
            last_composite = composite
            last_mask = observation["mask"].copy()
            last_observation = observation
            score = (
                1 if lane_camera_ready else 0,
                1 if status == "FULL" else 0,
                confidence,
                int(observation["line_count"]),
                -post_age,
            )
            if best_score is None or score > best_score:
                best_score = score
                best_composite = composite.copy()

            if not arguments.headless:
                cv2.imshow("IPM alignment diagnostic", composite)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        error_text = "INTERRUPTED_BY_OPERATOR"
    except Exception as error:
        error_text = repr(error)
        print("DIAGNOSTIC STOPPED: {}".format(error))
    finally:
        if camera is not None:
            camera.close()
        cv2.destroyAllWindows()

    best_written = write_image(best_path, best_composite)
    last_written = write_image(last_path, last_composite)
    mask_written = write_image(mask_path, last_mask)
    lane_camera_ready_rate = (
        100.0 * lane_camera_ready_frames / observations if observations else 0.0
    )
    stale_rate = (
        100.0 * stale_after_processing / observations if observations else 0.0
    )
    summary = {
        "schema_version": 1,
        "test": "IPM_ALIGNMENT_DIAGNOSTIC_ONLY",
        "validation_evidence": False,
        "motor_modules_imported": False,
        "completed_utc": utc_now(),
        "error": error_text,
        "config_path": os.path.abspath(arguments.config),
        "calibration_state": detector.config.get("calibration_state"),
        "detector_mode": detector.config.get("detector_mode"),
        "target_line_role": detector.config.get("target_line_role"),
        "camera_stale_limit_s_unchanged": CAMERA_STALE_LIMIT_SECONDS,
        "lane_confidence_minimum_unchanged": LANE_CONFIDENCE_MINIMUM,
        "warmup_seconds": arguments.warmup_seconds,
        "warmup_observations": warmup_observations,
        "measurement_seconds_requested": arguments.seconds,
        "observations": observations,
        "trustworthy_frames": trustworthy_frames,
        "statuses": dict(statuses),
        "lane_camera_ready_frames": lane_camera_ready_frames,
        "lane_camera_ready_rate_pct": lane_camera_ready_rate,
        "stale_after_processing_frames": stale_after_processing,
        "stale_after_processing_rate_pct": stale_rate,
        "mean_processing_ms": (
            statistics.mean(processing_values) if processing_values else None
        ),
        "maximum_processing_ms": (
            max(processing_values) if processing_values else None
        ),
        "mean_post_processing_age_ms": (
            statistics.mean(post_age_values) * 1000.0
            if post_age_values else None
        ),
        "last_status": (
            last_observation["status"] if last_observation is not None else None
        ),
        "last_confidence": (
            float(last_observation["confidence"])
            if last_observation is not None else None
        ),
        "last_offset": (
            float(last_observation["lane_offset"])
            if last_observation is not None else None
        ),
        "last_near_field_offset": (
            float(last_observation["near_field_offset"])
            if last_observation is not None else None
        ),
        "last_candidate_count": (
            int(last_observation["candidate_count"])
            if last_observation is not None else None
        ),
        "best_image": best_path if best_written else None,
        "last_image": last_path if last_written else None,
        "last_mask": mask_path if mask_written else None,
    }
    with open(summary_path, "w") as summary_file:
        json.dump(summary, summary_file, indent=2, sort_keys=True)
        summary_file.write("\n")

    print("\n" + "=" * 68)
    print("DIAGNOSTIC COMPLETE")
    print("=" * 68)
    print("Observations:              {}".format(observations))
    print("Lane states:               {}".format(dict(statuses)))
    print("Lane/camera-ready frames:  {} ({:.1f}%)".format(
        lane_camera_ready_frames, lane_camera_ready_rate
    ))
    print("Stale after processing:    {} ({:.1f}%)".format(
        stale_after_processing, stale_rate
    ))
    if processing_values:
        print("Mean / max processing:     {:.1f} / {:.1f} ms".format(
            statistics.mean(processing_values), max(processing_values)
        ))
    print("Best alignment image:      {}".format(
        best_path if best_written else "not written"
    ))
    print("Last alignment image:      {}".format(
        last_path if last_written else "not written"
    ))
    print("Last black-tape mask:      {}".format(
        mask_path if mask_written else "not written"
    ))
    print("Diagnostic summary:        {}".format(summary_path))
    print("This output is diagnostic only; do not record it as a gate.")

    if error_text is not None or observations == 0:
        print("RESULT: CAMERA/DIAGNOSTIC ERROR")
        return 1
    if lane_camera_ready_frames > 0:
        print("RESULT: LANE/CAMERA CHECK BECAME READY DURING DIAGNOSIS")
    elif statuses.get("FULL", 0) > 0:
        print("RESULT: FULL LANE SEEN, BUT CONFIDENCE OR FRESHNESS BLOCKED IT")
    else:
        print("RESULT: LANE NOT FULL - inspect the saved overlay and mask")
    return 0


if __name__ == "__main__":
    sys.exit(main())
