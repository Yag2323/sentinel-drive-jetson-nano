#!/usr/bin/env python3
"""Interactively calibrate the real-camera IPM quadrilateral; no motor access."""

from __future__ import print_function

import argparse
import datetime
import json
import os
import shutil
import sys

import cv2
import numpy as np

from gst_camera_bridge import GstCamera
from ipm_lane import DEFAULT_CONFIG_PATH, load_ipm_config
from validation_manager import capture_artifact_snapshot


POINT_LABELS = (
    "NEAR LEFT RAIL (BOTTOM LEFT)",
    "NEAR RIGHT RAIL (BOTTOM RIGHT)",
    "FAR RIGHT RAIL (TOP RIGHT)",
    "FAR LEFT RAIL (TOP LEFT)",
)

DISPLAY_LABELS = (
    "1: NEAR LEFT RAIL",
    "2: NEAR RIGHT RAIL",
    "3: FAR RIGHT RAIL",
    "4: FAR LEFT RAIL",
)


def main():
    parser = argparse.ArgumentParser(description="Calibrate physical-camera IPM")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--sensor-id", type=int, default=0)
    arguments = parser.parse_args()

    config_path = os.path.abspath(arguments.config)
    if os.path.realpath(config_path) != os.path.realpath(DEFAULT_CONFIG_PATH):
        print(
            "REFUSED: physical gate calibration must write the project's "
            "default ipm_config.json."
        )
        return 1
    code_snapshot_start = capture_artifact_snapshot(
        "ipm_physical_calibration",
        excluded_paths=("ipm_config.json",),
    )
    config = load_ipm_config(config_path)
    width = int(config.get("frame_width", 640))
    height = int(config.get("frame_height", 480))
    points = []

    print("IPM CALIBRATION - MOTORS ARE NOT ACCESSED")
    print("Park the robot centred on a STRAIGHT portion of the oval lane.")
    print("Both open left/right tape rails must be visible ahead of the camera.")
    print("Select the CENTRE of each tape rail in this exact order:")
    for index, label in enumerate(POINT_LABELS, 1):
        print("  {}. {}".format(index, label))
    print("The four points must outline the open lane corridor ahead.")
    print("DO NOT select a crossbar, end closure, corner, or curved section.")
    print("After four points: S saves, R resets, Q cancels.")

    camera = GstCamera(
        sensor_id=arguments.sensor_id,
        capture_width=1280,
        capture_height=720,
        output_width=width,
        output_height=height,
        framerate=21,
    )

    try:
        frame = None
        for _unused_index in range(20):
            success, candidate, metadata = camera.read_with_metadata(
                timeout_seconds=3.0
            )
            if (
                not success
                or metadata is None
                or not metadata.get("fresh")
                or not metadata.get("source_age_trustworthy")
            ):
                print("ERROR: no fresh timestamped CSI frame received.")
                return 1
            frame = candidate
        if frame is None:
            print("ERROR: no CSI frame received.")
            return 1
    finally:
        camera.close()

    original = frame.copy()
    window_name = "Oval IPM - straight rails only - BL, BR, TR, TL"

    def on_mouse(event, x, y, flags, parameter):
        del flags, parameter
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((int(x), int(y)))

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse)

    saved = False
    while True:
        display = original.copy()
        for index, point in enumerate(points):
            cv2.circle(display, point, 6, (0, 255, 255), -1)
            cv2.putText(
                display,
                str(index + 1),
                (point[0] + 8, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
        if len(points) > 1:
            cv2.polylines(display, [np.int32(points)], len(points) == 4, (0, 200, 255), 2)

        next_label = (
            DISPLAY_LABELS[len(points)]
            if len(points) < 4
            else "PRESS S TO SAVE"
        )
        cv2.putText(
            display,
            "OVAL STRAIGHT: NO ENDS / CROSSBARS / CORNERS",
            (12, height - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (0, 0, 255),
            2,
        )
        cv2.putText(
            display,
            next_label,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )
        cv2.imshow(window_name, display)
        key = cv2.waitKey(20) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("r"):
            points[:] = []
        if key == ord("s") and len(points) == 4:
            normalized = [
                [
                    float(point[0]) / max(1, width - 1),
                    float(point[1]) / max(1, height - 1),
                ]
                for point in points
            ]
            candidate_config = dict(config)
            candidate_config["source_points_normalized"] = normalized
            candidate_config["calibration_state"] = "PHYSICALLY_CALIBRATED"
            candidate_config["calibration_timestamp_utc"] = (
                datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"
            )

            timestamp = datetime.datetime.utcnow().strftime(
                "%Y%m%dT%H%M%S%fZ"
            )
            evidence_directory = os.path.join(
                os.path.dirname(config_path), "evidence"
            )
            if not os.path.isdir(evidence_directory):
                os.makedirs(evidence_directory)
            evidence_path = os.path.abspath(os.path.join(
                evidence_directory, "ipm_calibration_{}.jpg".format(timestamp)
            ))
            evidence_json_path = os.path.abspath(os.path.join(
                evidence_directory, "ipm_calibration_{}.json".format(timestamp)
            ))
            candidate_config["calibration_evidence_image"] = evidence_path
            candidate_config["calibration_evidence_json"] = evidence_json_path
            backup = config_path + ".before-physical-{}.bak".format(timestamp)
            shutil.copy2(config_path, backup)
            config_temporary = config_path + ".calibration-{}.tmp".format(
                timestamp
            )
            image_temporary = os.path.join(
                evidence_directory,
                ".ipm_calibration_{}.tmp.jpg".format(timestamp),
            )
            evidence_temporary = evidence_json_path + ".tmp"
            restore_temporary = config_path + ".rollback-{}.tmp".format(
                timestamp
            )
            config_committed = False
            image_committed = False
            try:
                with open(config_temporary, "w") as config_file:
                    json.dump(candidate_config, config_file, indent=2)
                    config_file.write("\n")
                    config_file.flush()
                    os.fsync(config_file.fileno())
                # Refuse wrong ordering, crossed/concave geometry, and
                # out-of-range points before changing the live config.
                validated_candidate = load_ipm_config(config_temporary)

                if not cv2.imwrite(image_temporary, display):
                    raise RuntimeError(
                        "Could not write the calibration evidence image."
                    )
                code_snapshot_end = capture_artifact_snapshot(
                    "ipm_physical_calibration",
                    excluded_paths=("ipm_config.json",),
                )
                artifact_stable = code_snapshot_start == code_snapshot_end
                if not artifact_stable:
                    raise RuntimeError(
                        "Calibration code changed during the run."
                    )

                os.replace(image_temporary, evidence_path)
                image_committed = True
                os.replace(config_temporary, config_path)
                config_committed = True

                full_snapshot = capture_artifact_snapshot(
                    "ipm_physical_calibration"
                )
                destination = validated_candidate[
                    "destination_points_normalized"
                ]
                source_pixels = np.float32(points)
                destination_pixels = np.float32(
                    [
                        [
                            float(point[0]) * max(1, width - 1),
                            float(point[1]) * max(1, height - 1),
                        ]
                        for point in destination
                    ]
                )
                homography = cv2.getPerspectiveTransform(
                    source_pixels, destination_pixels
                )
                evidence = {
                    "schema_version": 1,
                    "test": "PHYSICAL_IPM_CALIBRATION",
                    "passed": True,
                    "motor_commands": False,
                    "calibration_state": "PHYSICALLY_CALIBRATED",
                    "config_path": config_path,
                    "evidence_image": evidence_path,
                    "point_order": list(POINT_LABELS),
                    "source_points_normalized": normalized,
                    "destination_points_normalized": destination,
                    "homography_matrix": homography.tolist(),
                    "detector_mode": validated_candidate["detector_mode"],
                    "code_artifact_sha256_start": code_snapshot_start,
                    "code_artifact_sha256_end": code_snapshot_end,
                    "artifact_snapshot_stable": True,
                    "artifact_sha256_at_test": full_snapshot,
                    "completed_utc": candidate_config[
                        "calibration_timestamp_utc"
                    ],
                }
                with open(evidence_temporary, "w") as evidence_file:
                    json.dump(evidence, evidence_file, indent=2, sort_keys=True)
                    evidence_file.write("\n")
                    evidence_file.flush()
                    os.fsync(evidence_file.fileno())
                os.replace(evidence_temporary, evidence_json_path)
                saved = True
            except Exception as error:
                if config_committed:
                    try:
                        shutil.copy2(backup, restore_temporary)
                        os.replace(restore_temporary, config_path)
                    except Exception as restore_error:
                        print(
                            "CRITICAL: calibration rollback failed: {}".format(
                                restore_error
                            )
                        )
                if image_committed:
                    try:
                        os.unlink(evidence_path)
                    except OSError:
                        pass
                for temporary_path in (
                    config_temporary,
                    image_temporary,
                    evidence_temporary,
                    restore_temporary,
                ):
                    try:
                        os.unlink(temporary_path)
                    except OSError:
                        pass
                print("CALIBRATION SAVE FAILED: {}".format(error))
                saved = False

            if saved:
                print("Saved configuration: {}".format(config_path))
                print("Backup: {}".format(backup))
                print("Evidence image: {}".format(evidence_path))
                print("Evidence JSON: {}".format(evidence_json_path))
            break

    cv2.destroyAllWindows()
    if not saved:
        print("Calibration cancelled; configuration was not changed.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
