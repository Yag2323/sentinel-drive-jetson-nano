#!/usr/bin/env python3
"""Safely install the paired-boundary oval detector profile.

This migration deliberately preserves the physical IPM source and destination
points.  It invalidates the old physical calibration because changing the lane
detector profile requires fresh calibration and validation evidence before any
motion can be authorised.
"""

from __future__ import print_function

import datetime
import hashlib
import json
import os
import shutil
import sys
import tempfile

from ipm_lane import (
    DEFAULT_CONFIG_PATH,
    DETECTOR_IMPLEMENTATION_VERSION,
    PAIRED_ROW_DEFAULTS,
    load_ipm_config,
)


PROFILE_VERSION = "PAIRED_ROW_POLYNOMIAL_V2"
REVALIDATION_STATE = "OVAL_PROFILE_REQUIRES_PHYSICAL_CALIBRATION"

OVAL_PROFILE = {
    "detector_mode": PROFILE_VERSION,
    "scan_top_fraction": 0.28,
    "scan_bottom_fraction": 0.94,
    "scan_row_step_px": 6,
    "scan_band_height_px": 5,
    "minimum_column_fill_fraction": 0.40,
    "minimum_tape_run_width_px": 2,
    "maximum_tape_run_width_fraction": 0.08,
    "maximum_boundary_step_fraction": 0.04,
    "maximum_missing_scan_rows": 3,
    "maximum_row_runs": 24,
    "maximum_run_overflow_row_fraction": 0.10,
    "pair_width_tolerance_fraction": 0.35,
    "analysis_margin_lane_width_fraction": 0.35,
    "minimum_paired_row_fraction": 0.40,
    "minimum_vertical_coverage_fraction": 0.45,
    "fit_residual_threshold_px": 12.0,
    "minimum_fit_inlier_fraction": 0.70,
    "maximum_lane_width_variation_fraction": 0.25,
    "lane_width_evaluation_window_fraction": 0.30,
    "valid_roi_erode_px": 5,
    "lookahead_y_fraction": 0.62,
    "close_curve_minimum_pairs": 8,
    "close_curve_minimum_paired_row_fraction": 0.20,
    "close_curve_minimum_vertical_coverage_fraction": 0.30,
    "close_curve_maximum_fit_residual_px": 3.0,
    "close_curve_maximum_lane_width_variation_fraction": 0.15,
    "close_curve_minimum_pair_quality": 0.75,
    "close_curve_maximum_width_error_fraction": 0.35,
    "close_curve_maximum_raw_row_runs": 8,
}


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        while True:
            block = input_file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def utc_stamp():
    return datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%S%fZ")


def main():
    if DETECTOR_IMPLEMENTATION_VERSION != PROFILE_VERSION:
        print("REFUSED: paired-row detector implementation is not installed.")
        return 1
    expected_defaults = dict(PAIRED_ROW_DEFAULTS)
    expected_defaults["lookahead_y_fraction"] = 0.62
    mismatched = sorted(
        key
        for key in set(OVAL_PROFILE).union(expected_defaults)
        if OVAL_PROFILE.get(key) != expected_defaults.get(key)
    )
    if OVAL_PROFILE != expected_defaults:
        print(
            "REFUSED: installer/profile mismatch: {}".format(
                ", ".join(sorted(mismatched))
            )
        )
        return 1

    config_path = os.path.abspath(DEFAULT_CONFIG_PATH)
    if not os.path.isfile(config_path):
        print("REFUSED: missing configuration: {}".format(config_path))
        return 1

    with open(config_path, "r") as config_file:
        config = json.load(config_file)

    source_before = json.dumps(
        config.get("source_points_normalized"),
        separators=(",", ":"),
        sort_keys=True,
    )
    destination_before = json.dumps(
        config.get("destination_points_normalized"),
        separators=(",", ":"),
        sort_keys=True,
    )

    if (
        config.get("calibration_state") == REVALIDATION_STATE
        and all(config.get(key) == value for key, value in OVAL_PROFILE.items())
    ):
        print("Oval profile is already installed; configuration was not changed.")
        print("Calibration state: {}".format(REVALIDATION_STATE))
        print("Next: python calibrate_ipm.py")
        return 0

    timestamp = utc_stamp()
    backup_path = config_path + ".before-oval-profile-{}.bak".format(timestamp)
    shutil.copy2(config_path, backup_path)
    if sha256_file(config_path) != sha256_file(backup_path):
        print("REFUSED: backup verification failed; configuration was not changed.")
        return 1

    for key, value in OVAL_PROFILE.items():
        config[key] = value

    config["calibration_state"] = REVALIDATION_STATE
    config["calibration_timestamp_utc"] = None
    config["oval_profile_installed_utc"] = (
        datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"
    )
    # Old evidence describes the old detector/calibration pairing and must not
    # be mistaken for evidence supporting this profile.
    config.pop("calibration_evidence_image", None)
    config.pop("calibration_evidence_json", None)

    source_after = json.dumps(
        config.get("source_points_normalized"),
        separators=(",", ":"),
        sort_keys=True,
    )
    destination_after = json.dumps(
        config.get("destination_points_normalized"),
        separators=(",", ":"),
        sort_keys=True,
    )
    if source_after != source_before or destination_after != destination_before:
        print("REFUSED: migration attempted to change physical IPM points.")
        return 1

    directory = os.path.dirname(config_path)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=".ipm_config.oval-profile-",
        suffix=".tmp",
        dir=directory,
    )
    try:
        with os.fdopen(descriptor, "w") as temporary_file:
            json.dump(config, temporary_file, indent=2, sort_keys=True)
            temporary_file.write("\n")
            temporary_file.flush()
            os.fsync(temporary_file.fileno())

        # Use the detector's current validator, including its oval-profile
        # constraints, before the atomic replacement.  The existing file may
        # legitimately identify the previous V1 detector, so it cannot be
        # passed to the V2-only validator before this migration is applied.
        validated = load_ipm_config(temporary_path)
        if validated.get("calibration_state") == "PHYSICALLY_CALIBRATED":
            raise RuntimeError("Migration failed to invalidate calibration.")
        if validated.get("detector_mode") != PROFILE_VERSION:
            raise RuntimeError("Oval detector profile was not validated.")
        with open(temporary_path, "r") as written_file:
            written_config = json.load(written_file)
        if json.dumps(
            written_config.get("source_points_normalized"),
            separators=(",", ":"),
            sort_keys=True,
        ) != source_before:
            raise RuntimeError("Written migration changed physical source points.")
        if json.dumps(
            written_config.get("destination_points_normalized"),
            separators=(",", ":"),
            sort_keys=True,
        ) != destination_before:
            raise RuntimeError(
                "Written migration changed physical destination points."
            )

        os.replace(temporary_path, config_path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            try:
                os.unlink(temporary_path)
            except OSError:
                pass

    print("OVAL LANE PROFILE INSTALLED")
    print("Profile: {}".format(PROFILE_VERSION))
    print("Configuration: {}".format(config_path))
    print("Verified backup: {}".format(backup_path))
    print("Physical source and destination points were preserved exactly.")
    print("Calibration state: {}".format(REVALIDATION_STATE))
    print("Physical motion must remain unauthorised until all stale gates pass.")
    print("Next: python calibrate_ipm.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
