#!/usr/bin/env python3
"""Hardware-free fail-safe regression tests for single-line following.

The fixtures represent the outer black-tape centreline after inverse-
perspective warping.  No camera, I2C, motor, network or GUI resource is used.
The script deliberately runs without pytest so it remains usable with the
Jetson Nano Python 3.6/OpenCV environment.
"""

from __future__ import print_function

import json
import math
import os
import shutil
import sys
import tempfile

import cv2
import numpy as np

from ipm_lane import IPMLaneDetector
from single_line_lane import (
    SINGLE_LINE_DEFAULTS,
    SINGLE_LINE_DETECTOR_MODE,
)


WIDTH = 640
HEIGHT = 360
FLOOR_COLOUR = 228
TAPE_COLOUR = (16, 16, 16)
TAPE_THICKNESS = 11


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def write_test_config(directory):
    """Write a strict single-line config with an identity-like homography."""
    path = os.path.join(directory, "ipm_config.single_line.synthetic.json")
    config = {
        "schema_version": 1,
        "calibration_state": "PHYSICALLY_CALIBRATED",
        "calibration_target": "SYNTHETIC_IDENTITY_GROUND_PLANE",
        "target_line_role": "OUTER_CIRCLE_CENTERLINE",
        "frame_width": WIDTH,
        "frame_height": HEIGHT,
        "source_points_normalized": [
            [0.05, 0.98],
            [0.95, 0.98],
            [0.95, 0.02],
            [0.05, 0.02],
        ],
        "destination_points_normalized": [
            [0.05, 0.98],
            [0.95, 0.98],
            [0.95, 0.02],
            [0.05, 0.02],
        ],
        "black_threshold": 83,
        "close_kernel_size": 5,
        # Retained common/legacy values are required by the shared config and
        # mask pipeline.  They do not weaken the single-line proof.
        "hough_votes": 20,
        "minimum_line_length": 25,
        "maximum_line_gap": 45,
        "minimum_vertical_ratio": 1.0,
        "lookahead_y_fraction": 0.62,
        "minimum_lane_width_fraction": 0.18,
        "maximum_lane_width_fraction": 0.62,
        "scan_top_fraction": 0.25,
        "scan_bottom_fraction": 0.96,
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
        "analysis_margin_lane_width_fraction": 0.50,
        "minimum_paired_row_fraction": 0.40,
        "minimum_vertical_coverage_fraction": 0.45,
        "fit_residual_threshold_px": 12.0,
        "minimum_fit_inlier_fraction": 0.70,
        "maximum_lane_width_variation_fraction": 0.25,
        "lane_width_evaluation_window_fraction": 0.30,
        "valid_roi_erode_px": 3,
        "close_curve_minimum_pairs": 8,
        "close_curve_minimum_paired_row_fraction": 0.20,
        "close_curve_minimum_vertical_coverage_fraction": 0.30,
        "close_curve_maximum_fit_residual_px": 3.0,
        "close_curve_maximum_lane_width_variation_fraction": 0.15,
        "close_curve_minimum_pair_quality": 0.75,
        "close_curve_maximum_width_error_fraction": 0.35,
        "close_curve_maximum_raw_row_runs": 8,
    }
    config.update(SINGLE_LINE_DEFAULTS)
    config["detector_mode"] = SINGLE_LINE_DETECTOR_MODE
    with open(path, "w") as config_file:
        json.dump(config, config_file, indent=2, sort_keys=True)
        config_file.write("\n")
    return path


def clean_floor(value=FLOOR_COLOUR):
    return np.full((HEIGHT, WIDTH, 3), int(value), dtype=np.uint8)


def centreline_points(x_function, y_start=0, y_stop=HEIGHT - 1):
    points = []
    for y_value in range(int(y_start), int(y_stop) + 1, 2):
        x_value = int(round(x_function(float(y_value))))
        points.append([x_value, y_value])
    if points[-1][1] != int(y_stop):
        points.append([int(round(x_function(float(y_stop)))), int(y_stop)])
    return np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))


def draw_centreline(x_function, thickness=TAPE_THICKNESS):
    frame = clean_floor()
    cv2.polylines(
        frame,
        [centreline_points(x_function)],
        False,
        TAPE_COLOUR,
        int(thickness),
        cv2.LINE_8,
    )
    return frame


def straight_frame(x_value=WIDTH / 2.0):
    return draw_centreline(lambda _y: float(x_value))


def curve_frame(direction):
    """Return a centreline under the car that bends left/right ahead."""
    scan_top = HEIGHT * 0.25
    scan_bottom = HEIGHT * 0.96
    span = scan_bottom - scan_top

    def x_function(y_value):
        forward = max(0.0, min(1.0, (scan_bottom - y_value) / span))
        return WIDTH / 2.0 + float(direction) * 130.0 * forward * forward

    return draw_centreline(x_function)


def close_perspective_tape_frame():
    """Model the physically observed close tape after IPM warping.

    The raised camera makes the nearby tape approximately 10% of the warped
    frame width.  It is still a single elongated, well-supported component,
    not a broad obstacle or floor blob.
    """
    frame = draw_centreline(
        lambda y_value: 345.0
        + 0.0008 * (HEIGHT - float(y_value)) ** 2,
        thickness=57,
    )
    # Model a local 73 px perspective/morphology bulge like the physical
    # diagnostic.  The majority width remains valid 1.8 cm tape; accepting
    # this must not imply accepting a uniformly oversized stripe.
    bulge_y = 190
    bulge_x = int(round(
        345.0 + 0.0008 * (HEIGHT - float(bulge_y)) ** 2
    ))
    cv2.circle(frame, (bulge_x, bulge_y), 36, TAPE_COLOUR, -1)
    return frame


def oversized_stripe_frame():
    """A full-height dark stripe wider than the measured tape is not tape."""
    return draw_centreline(lambda _y: WIDTH / 2.0, thickness=78)


def fragmented_line_frame():
    """Leave a large gap spanning look-ahead so neither fragment is valid."""
    frame = clean_floor()
    x_value = WIDTH // 2
    cv2.line(
        frame, (x_value, 75), (x_value, 182), TAPE_COLOUR,
        TAPE_THICKNESS, cv2.LINE_8,
    )
    cv2.line(
        frame, (x_value, 262), (x_value, HEIGHT - 1), TAPE_COLOUR,
        TAPE_THICKNESS, cv2.LINE_8,
    )
    return frame


def broad_blob_frame():
    frame = clean_floor()
    cv2.rectangle(frame, (260, 70), (380, 350), TAPE_COLOUR, -1)
    return frame


def ambiguous_parallel_lines_frame():
    frame = clean_floor()
    for x_value in (250, 390):
        cv2.line(
            frame,
            (x_value, 0),
            (x_value, HEIGHT - 1),
            TAPE_COLOUR,
            TAPE_THICKNESS,
            cv2.LINE_8,
        )
    return frame


def unequal_parallel_lines_frame():
    """Two valid candidates remain ambiguous even with unequal fit scores."""
    frame = clean_floor()
    cv2.line(
        frame,
        (250, 0),
        (250, HEIGHT - 1),
        TAPE_COLOUR,
        TAPE_THICKNESS,
        cv2.LINE_8,
    )
    points = []
    for y_value in range(0, HEIGHT, 2):
        points.append([
            int(round(390.0 + 4.0 * math.sin(y_value / 18.0))),
            y_value,
        ])
    cv2.polylines(
        frame,
        [np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))],
        False,
        TAPE_COLOUR,
        TAPE_THICKNESS,
        cv2.LINE_8,
    )
    return frame


def weak_outer_strong_inner_frame():
    """Model a weak intended segment beside a strong wrong-circle line."""
    frame = clean_floor()
    cv2.line(
        frame,
        (WIDTH // 2, 0),
        (WIDTH // 2, HEIGHT - 1),
        TAPE_COLOUR,
        TAPE_THICKNESS,
        cv2.LINE_8,
    )
    cv2.line(
        frame,
        (240, 240),
        (240, HEIGHT - 1),
        TAPE_COLOUR,
        TAPE_THICKNESS,
        cv2.LINE_8,
    )
    return frame


def dense_noise_frame():
    frame = clean_floor()
    random_state = np.random.RandomState(2062)
    # Many independent tape-like fragments must not be promoted to a line.
    for _unused in range(900):
        x_value = int(random_state.randint(25, WIDTH - 25))
        y_value = int(random_state.randint(20, HEIGHT - 10))
        radius = int(random_state.randint(1, 4))
        cv2.circle(frame, (x_value, y_value), radius, TAPE_COLOUR, -1)
    return frame


def observe_new(config_path, frame):
    return IPMLaneDetector(config_path).observe(frame)


def require_full(observation, label):
    require(
        observation["status"] == "FULL",
        "{} must be FULL, got {}".format(label, observation["status"]),
    )
    require(
        float(observation["confidence"]) >= 0.55,
        "{} confidence must satisfy the 0.55 gate, got {:.3f}".format(
            label, float(observation["confidence"])
        ),
    )
    require(
        observation.get("detector_mode") == SINGLE_LINE_DETECTOR_MODE,
        "{} used unexpected detector mode {}".format(
            label, observation.get("detector_mode")
        ),
    )
    return observation


def test_centered_straight(config_path):
    observation = require_full(
        observe_new(config_path, straight_frame()), "centred straight"
    )
    require(
        abs(float(observation["lane_offset"])) <= 0.025,
        "centred straight look-ahead offset was {:+.4f}".format(
            float(observation["lane_offset"])
        ),
    )
    require(
        abs(float(observation["near_field_offset"])) <= 0.025,
        "centred straight near-field offset was {:+.4f}".format(
            float(observation["near_field_offset"])
        ),
    )
    return observation


def test_shift_signs(config_path):
    left = require_full(
        observe_new(config_path, straight_frame(250.0)), "left-shifted line"
    )
    right = require_full(
        observe_new(config_path, straight_frame(390.0)), "right-shifted line"
    )
    require(
        float(left["lane_offset"]) < -0.10,
        "left-shifted line must have negative offset, got {:+.4f}".format(
            float(left["lane_offset"])
        ),
    )
    require(
        float(right["lane_offset"]) > 0.10,
        "right-shifted line must have positive offset, got {:+.4f}".format(
            float(right["lane_offset"])
        ),
    )
    return left, right


def test_curve_signs(config_path):
    left = require_full(
        observe_new(config_path, curve_frame(-1.0)), "left curve"
    )
    right = require_full(
        observe_new(config_path, curve_frame(1.0)), "right curve"
    )
    require(
        abs(float(left["near_field_offset"])) <= 0.04,
        "left curve must remain near-centred under the car, got {:+.4f}".format(
            float(left["near_field_offset"])
        ),
    )
    require(
        abs(float(right["near_field_offset"])) <= 0.04,
        "right curve must remain near-centred under the car, got {:+.4f}".format(
            float(right["near_field_offset"])
        ),
    )
    require(
        float(left["lane_offset"]) < -0.05,
        "left curve must request a negative correction, got {:+.4f}".format(
            float(left["lane_offset"])
        ),
    )
    require(
        float(right["lane_offset"]) > 0.05,
        "right curve must request a positive correction, got {:+.4f}".format(
            float(right["lane_offset"])
        ),
    )
    require(
        float(left["heading_error_rad"]) < 0.0
        and float(right["heading_error_rad"]) > 0.0,
        "mirrored curves must produce mirrored heading signs",
    )
    return left, right


def test_close_perspective_tape_accepted(config_path):
    observation = require_full(
        observe_new(config_path, close_perspective_tape_frame()),
        "close perspective tape",
    )
    require(
        float(observation["tape_width_px"]) > WIDTH * 0.08,
        "close-tape fixture must exercise the former 8% rejection limit",
    )
    require(
        int(observation.get("candidate_count", 0)) == 1,
        "close tape must remain exactly one candidate",
    )
    return observation


def test_oversized_stripe_rejected(config_path):
    observation = observe_new(config_path, oversized_stripe_frame())
    require(
        observation["status"] != "FULL",
        "an oversized full-height dark stripe must not be accepted as tape",
    )
    return observation


def test_blank_after_good_immediately_lost(config_path):
    detector = IPMLaneDetector(config_path)
    initial = detector.observe(straight_frame())
    require_full(initial, "blank-memory seed")
    blank = detector.observe(clean_floor())
    require(
        blank["status"] == "LOST",
        "blank frame after FULL must immediately be LOST, got {}".format(
            blank["status"]
        ),
    )
    require(
        float(blank["confidence"]) == 0.0,
        "blank frame must have zero confidence",
    )
    return blank


def test_fragmented_rejected(config_path):
    observation = observe_new(config_path, fragmented_line_frame())
    require(
        observation["status"] != "FULL",
        "a line fragmented across look-ahead must not be FULL",
    )
    return observation


def test_broad_blob_rejected(config_path):
    observation = observe_new(config_path, broad_blob_frame())
    require(
        observation["status"] != "FULL",
        "a broad filled dark object must not be accepted as tape",
    )
    return observation


def test_two_lines_ambiguous(config_path):
    observation = observe_new(config_path, ambiguous_parallel_lines_frame())
    require(
        observation["status"] == "AMBIGUOUS",
        "two equally plausible lines must be AMBIGUOUS, got {}".format(
            observation["status"]
        ),
    )
    require(
        int(observation.get("candidate_count", 0)) >= 2,
        "ambiguous frame must report at least two candidates",
    )
    require(
        float(observation["confidence"]) == 0.0,
        "ambiguous frame must have zero confidence",
    )
    return observation


def test_unequal_lines_still_ambiguous(config_path):
    observation = observe_new(config_path, unequal_parallel_lines_frame())
    require(
        observation["status"] == "AMBIGUOUS",
        "two plausible lines must remain AMBIGUOUS even with unequal scores, "
        "got {}".format(observation["status"]),
    )
    require(
        int(observation.get("candidate_count", 0)) >= 2,
        "unequal ambiguous frame must report at least two candidates",
    )
    require(
        float(observation["confidence"]) == 0.0,
        "unequal ambiguous frame must have zero confidence",
    )
    return observation


def test_weak_outer_cannot_promote_inner(config_path):
    observation = observe_new(config_path, weak_outer_strong_inner_frame())
    require(
        observation["status"] == "AMBIGUOUS",
        "a strong wrong-circle line beside weak target evidence must be "
        "AMBIGUOUS, got {}".format(observation["status"]),
    )
    require(
        int(observation.get("line_like_component_count", 0)) >= 2,
        "wrong-target takeover fixture must retain secondary line evidence",
    )
    require(
        float(observation["confidence"]) == 0.0,
        "wrong-target takeover fixture must have zero confidence",
    )
    return observation


def test_dense_noise_rejected(config_path):
    observation = observe_new(config_path, dense_noise_frame())
    require(
        observation["status"] != "FULL",
        "dense floor noise must not be classified FULL",
    )
    return observation


def print_result(name, observation):
    print(
        "PASS {:28s} status={:12s} conf={:.3f} offset={:+.4f}".format(
            name,
            str(observation["status"]),
            float(observation["confidence"]),
            float(observation.get("lane_offset", 0.0)),
        )
    )


def main():
    temporary_directory = tempfile.mkdtemp(prefix="sentinel_single_line_test_")
    try:
        config_path = write_test_config(temporary_directory)
        centred = test_centered_straight(config_path)
        print_result("centered_straight", centred)

        shifted_left, shifted_right = test_shift_signs(config_path)
        print_result("shifted_left", shifted_left)
        print_result("shifted_right", shifted_right)

        curve_left, curve_right = test_curve_signs(config_path)
        print_result("curve_left", curve_left)
        print_result("curve_right", curve_right)

        close_tape = test_close_perspective_tape_accepted(config_path)
        print_result("close_perspective_tape", close_tape)

        oversized = test_oversized_stripe_rejected(config_path)
        print_result("oversized_stripe", oversized)

        blank = test_blank_after_good_immediately_lost(config_path)
        print_result("blank_after_good", blank)
        fragmented = test_fragmented_rejected(config_path)
        print_result("fragmented_gap", fragmented)
        broad = test_broad_blob_rejected(config_path)
        print_result("broad_blob", broad)
        ambiguous = test_two_lines_ambiguous(config_path)
        print_result("parallel_ambiguity", ambiguous)
        unequal_ambiguous = test_unequal_lines_still_ambiguous(config_path)
        print_result("unequal_ambiguity", unequal_ambiguous)
        takeover = test_weak_outer_cannot_promote_inner(config_path)
        print_result("wrong_target_takeover", takeover)
        noise = test_dense_noise_rejected(config_path)
        print_result("dense_noise", noise)

        print("SINGLE-LINE SYNTHETIC REGRESSION: PASS")
        return 0
    except Exception as error:
        print("SINGLE-LINE SYNTHETIC REGRESSION: FAIL")
        print("{}: {}".format(type(error).__name__, error))
        return 1
    finally:
        shutil.rmtree(temporary_directory, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
