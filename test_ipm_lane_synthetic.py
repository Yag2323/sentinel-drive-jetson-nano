#!/usr/bin/env python3
"""Hardware-free regression tests for the physical IPM lane detector.

The generated frames exercise the geometry needed by the oval two-boundary
track.  No camera, I2C device, motor module, or network resource is imported.
This script is intentionally runnable without pytest on the Jetson Nano's
Python 3.6/OpenCV 3.2 environment.
"""

from __future__ import print_function

import json
import os
import shutil
import sys
import tempfile

import cv2
import numpy as np

from control_core import SafetySupervisor, load_controller_config
from ipm_lane import IPMLaneDetector, normalized_points_to_pixels


WIDTH = 640
HEIGHT = 360
TAPE_COLOUR = (18, 18, 18)
TAPE_THICKNESS = 11


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def write_test_config(directory):
    """Write an identity-homography config with an oval-analysis margin."""
    path = os.path.join(directory, "ipm_config.synthetic.json")
    config = {
        "schema_version": 1,
        "calibration_state": "PHYSICALLY_CALIBRATED",
        "frame_width": WIDTH,
        "frame_height": HEIGHT,
        # Equal source/destination quads give an identity homography while
        # preserving the nominal lane width used by the physical detector.
        "source_points_normalized": [
            [0.25, 1.0],
            [0.75, 1.0],
            [0.75, 0.0],
            [0.25, 0.0],
        ],
        "destination_points_normalized": [
            [0.25, 1.0],
            [0.75, 1.0],
            [0.75, 0.0],
            [0.25, 0.0],
        ],
        "black_threshold": 83,
        "close_kernel_size": 5,
        "hough_votes": 20,
        "minimum_line_length": 25,
        "maximum_line_gap": 45,
        "minimum_vertical_ratio": 1.0,
        "lookahead_y_fraction": 0.62,
        "minimum_lane_width_fraction": 0.18,
        "maximum_lane_width_fraction": 0.62,
        # New paired-row detector options. Older implementations safely
        # ignore these extra JSON keys, so the test also diagnoses whether
        # the intended detector has actually landed.
        "detector_mode": "PAIRED_ROW_POLYNOMIAL_V2",
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
        "close_curve_minimum_pairs": 8,
        "close_curve_minimum_paired_row_fraction": 0.20,
        "close_curve_minimum_vertical_coverage_fraction": 0.30,
        "close_curve_maximum_fit_residual_px": 3.0,
        "close_curve_maximum_lane_width_variation_fraction": 0.15,
        "close_curve_minimum_pair_quality": 0.75,
        "close_curve_maximum_width_error_fraction": 0.35,
        "close_curve_maximum_raw_row_runs": 8,
    }
    with open(path, "w") as config_file:
        json.dump(config, config_file, indent=2, sort_keys=True)
        config_file.write("\n")
    return path


def write_config_variant(config_path, filename, updates):
    """Write a test-only config variant beside the base temporary config."""
    with open(config_path, "r") as config_file:
        config = json.load(config_file)
    config.update(updates)
    path = os.path.join(os.path.dirname(config_path), filename)
    with open(path, "w") as config_file:
        json.dump(config, config_file, indent=2, sort_keys=True)
        config_file.write("\n")
    return path


def clean_floor(value=228):
    return np.full((HEIGHT, WIDTH, 3), int(value), dtype=np.uint8)


def points_for_x(x_function):
    points = []
    for y_value in range(0, HEIGHT, 3):
        x_value = int(round(x_function(float(y_value))))
        points.append([x_value, y_value])
    if points[-1][1] != HEIGHT - 1:
        points.append([int(round(x_function(float(HEIGHT - 1)))), HEIGHT - 1])
    return np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))


def draw_boundaries(left_function, right_function, crossbar_y=None):
    frame = clean_floor()
    cv2.polylines(
        frame,
        [points_for_x(left_function)],
        False,
        TAPE_COLOUR,
        TAPE_THICKNESS,
        cv2.LINE_8,
    )
    cv2.polylines(
        frame,
        [points_for_x(right_function)],
        False,
        TAPE_COLOUR,
        TAPE_THICKNESS,
        cv2.LINE_8,
    )
    if crossbar_y is not None:
        y_value = int(crossbar_y)
        left_x = int(round(left_function(float(y_value))))
        right_x = int(round(right_function(float(y_value))))
        cv2.line(
            frame,
            (max(0, left_x - 24), y_value),
            (min(WIDTH - 1, right_x + 24), y_value),
            TAPE_COLOUR,
            TAPE_THICKNESS,
            cv2.LINE_8,
        )
    return frame


def straight_frame():
    return draw_boundaries(lambda _y: 180.0, lambda _y: 460.0)


def same_half_curve_frame():
    # Both rails remain to the left of x=WIDTH/2 at the 62% look-ahead row.
    def centre(y_value):
        fraction = y_value / float(HEIGHT - 1)
        relative = fraction - 0.62
        return 175.0 + 25.0 * relative + 5.0 * relative * relative

    return draw_boundaries(
        lambda y_value: centre(y_value) - 105.0,
        lambda y_value: centre(y_value) + 105.0,
    )


def same_half_right_curve_frame():
    """Mirror the left-bend case so both rails sit right of image centre."""
    return cv2.flip(same_half_curve_frame(), 1)


def concentric_arc_frame():
    """Model the changing horizontal rail gap on a real oval bend."""
    frame = clean_floor()
    scan_bottom = int(round(HEIGHT * 0.94))
    y_values = np.arange(int(round(HEIGHT * 0.28)), scan_bottom + 1)
    forward = (scan_bottom - y_values).astype(np.float64)
    circle_centre_x = 820.0
    outer_radius = 660.0
    inner_radius = 340.0
    left_x = circle_centre_x - np.sqrt(
        np.maximum(0.0, outer_radius ** 2 - forward ** 2)
    )
    right_x = circle_centre_x - np.sqrt(
        np.maximum(0.0, inner_radius ** 2 - forward ** 2)
    )
    for x_values in (left_x, right_x):
        cv2.polylines(
            frame,
            [np.int32(np.column_stack([x_values, y_values]))],
            False,
            TAPE_COLOUR,
            TAPE_THICKNESS,
            cv2.LINE_8,
        )
    return frame


def close_curve_short_support_frame():
    """Model two continuous close-bend components near look-ahead."""
    frame = clean_floor()
    y_values = np.arange(158, 237)
    relative = (y_values.astype(np.float64) - 222.0) / 64.0
    left_x = 104.0 + 6.0 * relative + 2.5 * relative * relative
    right_x = 344.0 + 17.0 * relative + 5.0 * relative * relative
    for x_values in (left_x, right_x):
        cv2.polylines(
            frame,
            [np.int32(np.column_stack([x_values, y_values]))],
            False,
            TAPE_COLOUR,
            TAPE_THICKNESS,
            cv2.LINE_8,
        )
    return frame


def close_curve_wide_row_frame():
    """A tape-like bend whose right rail is too wide for row-run parsing."""
    frame = clean_floor()
    left_points = np.int32([
        [108, 153],
        [103, 175],
        [100, 198],
        [101, 220],
        [106, 241],
    ])
    right_points = np.int32([
        [510, 158],
        [470, 165],
        [430, 178],
        [395, 195],
        [365, 215],
        [345, 241],
    ])
    for points in (left_points, right_points):
        cv2.polylines(
            frame,
            [points],
            False,
            TAPE_COLOUR,
            18,
            cv2.LINE_8,
        )
    return frame


def broad_blob_false_pair_frame():
    """A real rail plus a broad dark object must never form a lane."""
    frame = clean_floor()
    cv2.rectangle(frame, (90, 170), (105, 270), TAPE_COLOUR, -1)
    cv2.rectangle(frame, (385, 170), (565, 270), TAPE_COLOUR, -1)
    return frame


def textured_noise_frame():
    random_state = np.random.RandomState(2062)
    gray = random_state.normal(220.0, 8.0, (HEIGHT, WIDTH))
    gray = np.clip(gray, 150.0, 245.0).astype(np.uint8)
    frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    # Isolated dark floor marks must not form a synthetic pair of rails.
    for _unused in range(420):
        x_value = int(random_state.randint(0, WIDTH))
        y_value = int(random_state.randint(0, HEIGHT))
        radius = int(random_state.randint(1, 3))
        cv2.circle(frame, (x_value, y_value), radius, TAPE_COLOUR, -1)
    return frame


def outside_roi_false_pair_frame():
    """Draw a plausible-width pair with one rail outside the calibrated ROI."""
    frame = clean_floor()
    for x_value in (12, 332):
        cv2.line(
            frame,
            (x_value, 0),
            (x_value, HEIGHT - 1),
            TAPE_COLOUR,
            TAPE_THICKNESS,
            cv2.LINE_8,
        )
    return frame


def excessive_runs_frame():
    """Create more per-row candidates than the bounded detector accepts.

    Keep the synthetic stripes far enough apart that Gaussian filtering and
    the 5 px close kernel cannot join neighbouring stripes on OpenCV 3.2.
    The former 3 px stripes at 9 px centres were distinct on newer OpenCV,
    but could merge into one broad rejected blob on the Jetson Nano image.
    Thirty-one 5 px stripes at 17 px centres remain separate while still
    exceeding the configured 24-run safety cap on every scanned row.
    """
    frame = clean_floor()
    for x_value in range(65, 578, 17):
        cv2.line(
            frame,
            (x_value, 0),
            (x_value, HEIGHT - 1),
            TAPE_COLOUR,
            5,
            cv2.LINE_8,
        )
    return frame


def fragmented_pair_frame():
    """Two separated rail fragments must not be reconnected as one lane."""
    frame = clean_floor()
    for y_start, y_stop in ((102, 150), (270, 337)):
        for x_value in (180, 460):
            cv2.line(
                frame,
                (x_value, y_start),
                (x_value, y_stop),
                TAPE_COLOUR,
                TAPE_THICKNESS,
                cv2.LINE_8,
            )
    return frame


def distant_supported_pair_frame():
    """Provide enough total rail coverage, but none near the look-ahead row."""
    frame = clean_floor()
    for x_value in (180, 460):
        cv2.line(
            frame,
            (x_value, 101),
            (x_value, 210),
            TAPE_COLOUR,
            TAPE_THICKNESS,
            cv2.LINE_8,
        )
    return frame


def asymmetric_spurious_valid_pair_frame():
    """Add a competing left-side rail without changing the true lane pair."""
    frame = straight_frame()
    cv2.line(
        frame,
        (70, 105),
        (70, 337),
        TAPE_COLOUR,
        TAPE_THICKNESS,
        cv2.LINE_8,
    )
    return frame


def asymmetric_spurious_only_frame():
    """One true rail plus discontinuous outliers must not form a lane."""
    frame = clean_floor()
    cv2.line(
        frame,
        (180, 0),
        (180, HEIGHT - 1),
        TAPE_COLOUR,
        TAPE_THICKNESS,
        cv2.LINE_8,
    )
    for y_start, y_stop, x_value in (
        (106, 130, 460),
        (190, 208, 500),
        (300, 320, 460),
    ):
        cv2.line(
            frame,
            (x_value, y_start),
            (x_value, y_stop),
            TAPE_COLOUR,
            TAPE_THICKNESS,
            cv2.LINE_8,
        )
    return frame


def observe_new(config_path, frame):
    return IPMLaneDetector(config_path).observe(frame)


def test_straight_full(config_path):
    observation = observe_new(config_path, straight_frame())
    require(
        observation["status"] == "FULL",
        "straight lane must be FULL, got {}".format(observation["status"]),
    )
    require(
        observation["confidence"] >= 0.55,
        "straight FULL confidence must satisfy the 0.55 safety gate, got {:.3f}".format(
            observation["confidence"]
        ),
    )
    require(
        abs(observation["lane_offset"]) <= 0.03,
        "straight centred offset must be within 0.03, got {:.4f}".format(
            observation["lane_offset"]
        ),
    )
    return observation


def test_curve_both_rails_same_half(config_path):
    observation = observe_new(config_path, same_half_curve_frame())
    require(
        observation["status"] == "FULL",
        "same-half curved rails must be paired as FULL, got {}".format(
            observation["status"]
        ),
    )
    require(
        observation["confidence"] >= 0.55,
        "curved FULL confidence must satisfy the 0.55 safety gate, got {:.3f}".format(
            observation["confidence"]
        ),
    )
    require(
        observation["left_x"] < WIDTH / 2.0
        and observation["right_x"] < WIDTH / 2.0,
        "test geometry error: both detected rails must be left of image centre",
    )
    require(
        observation["lane_offset"] < -0.05,
        "left-hand curve must retain a negative offset, got {:.4f}".format(
            observation["lane_offset"]
        ),
    )
    return observation


def test_curve_both_rails_same_right_half(config_path):
    observation = observe_new(config_path, same_half_right_curve_frame())
    require(
        observation["status"] == "FULL",
        "same-right-half curved rails must be paired as FULL, got {}".format(
            observation["status"]
        ),
    )
    require(
        observation["confidence"] >= 0.55,
        "right curve confidence must satisfy the 0.55 safety gate, got {:.3f}".format(
            observation["confidence"]
        ),
    )
    require(
        observation["left_x"] > WIDTH / 2.0
        and observation["right_x"] > WIDTH / 2.0,
        "test geometry error: both detected rails must be right of image centre",
    )
    require(
        observation["lane_offset"] > 0.05,
        "right-hand curve must retain a positive offset, got {:.4f}".format(
            observation["lane_offset"]
        ),
    )
    return observation


def test_concentric_oval_arc(config_path):
    observation = observe_new(config_path, concentric_arc_frame())
    require(
        observation["status"] == "FULL",
        "concentric oval rails with a changing horizontal gap must be FULL, got {}".format(
            observation["status"]
        ),
    )
    require(
        observation["confidence"] >= 0.55,
        "concentric oval confidence must satisfy the safety gate, got {:.3f}".format(
            observation["confidence"]
        ),
    )
    require(
        observation["lane_offset"] > 0.03,
        "concentric right bend must request a positive correction, got {:.4f}".format(
            observation["lane_offset"]
        ),
    )
    return observation


def test_guarded_close_curve(config_path):
    observation = observe_new(config_path, close_curve_short_support_frame())
    require(
        observation["status"] == "FULL",
        "guarded close curve must be FULL, got {}".format(
            observation["status"]
        ),
    )
    require(
        observation["support_mode"] == "COMPONENT_CURVE",
        "short-support curve must use COMPONENT_CURVE, got {}".format(
            observation["support_mode"]
        ),
    )
    require(
        observation["confidence"] >= 0.55,
        "guarded close-curve confidence must satisfy the safety gate, got "
        "{:.3f}".format(observation["confidence"]),
    )
    require(
        observation["paired_row_fraction"] < 0.40
        or observation["vertical_coverage_fraction"] < 0.45,
        "close-curve fixture accidentally passed the standard support gate",
    )
    return observation


def test_wide_row_component_curve(config_path):
    observation = observe_new(config_path, close_curve_wide_row_frame())
    maximum_active_run = 0
    for row in observation["mask"] > 0:
        active_x = np.where(row)[0]
        if active_x.size == 0:
            continue
        split_positions = np.where(np.diff(active_x) > 1)[0] + 1
        for run in np.split(active_x, split_positions):
            maximum_active_run = max(maximum_active_run, int(run.size))
    configured_row_limit = WIDTH * float(
        IPMLaneDetector(config_path).config[
            "maximum_tape_run_width_fraction"
        ]
    )
    require(
        maximum_active_run > configured_row_limit,
        "wide-row fixture did not exceed the horizontal run limit",
    )
    require(
        observation["status"] == "FULL",
        "near-horizontal tape curve must be FULL, got {}".format(
            observation["status"]
        ),
    )
    require(
        observation["support_mode"] == "COMPONENT_CURVE",
        "wide-row curve must use COMPONENT_CURVE, got {}".format(
            observation["support_mode"]
        ),
    )
    require(
        observation["confidence"] >= 0.55,
        "wide-row curve confidence must satisfy the safety gate, got "
        "{:.3f}".format(observation["confidence"]),
    )
    return observation


def test_broad_blob_not_rail(config_path):
    observation = observe_new(config_path, broad_blob_false_pair_frame())
    require(
        observation["status"] != "FULL",
        "a broad filled dark object must not be accepted as a rail",
    )
    return observation


def test_crossbar_ignored(config_path):
    left_function = lambda _y: 180.0
    right_function = lambda _y: 460.0
    detector = IPMLaneDetector(config_path)
    baseline = detector.observe(draw_boundaries(left_function, right_function))
    with_crossbar = detector.observe(
        draw_boundaries(left_function, right_function, crossbar_y=222)
    )
    require(baseline["status"] == "FULL", "crossbar baseline must be FULL")
    require(
        with_crossbar["status"] == "FULL",
        "horizontal closure must be ignored, got {}".format(
            with_crossbar["status"]
        ),
    )
    require(
        abs(with_crossbar["lane_offset"] - baseline["lane_offset"]) <= 0.02,
        "crossbar changed lane offset by more than 0.02",
    )
    return with_crossbar


def test_invalid_width_not_full(config_path):
    frame = draw_boundaries(lambda _y: 292.0, lambda _y: 348.0)
    observation = observe_new(config_path, frame)
    require(
        observation["status"] != "FULL",
        "implausibly narrow rails must never be FULL",
    )
    return observation


def test_single_rail_after_full_not_full(config_path):
    detector = IPMLaneDetector(config_path)
    initial = detector.observe(straight_frame())
    require(initial["status"] == "FULL", "single-rail test needs a FULL seed")
    single = clean_floor()
    cv2.polylines(
        single,
        [points_for_x(lambda _y: 180.0)],
        False,
        TAPE_COLOUR,
        TAPE_THICKNESS,
        cv2.LINE_8,
    )
    observation = detector.observe(single)
    require(
        observation["status"] != "FULL",
        "remembered width must not promote one current rail to FULL",
    )
    return observation


def test_blank_after_full_is_lost(config_path):
    detector = IPMLaneDetector(config_path)
    initial = detector.observe(straight_frame())
    require(initial["status"] == "FULL", "blank-memory test needs a FULL seed")
    observation = detector.observe(clean_floor())
    require(
        observation["status"] == "LOST",
        "blank frame after FULL must be LOST, got {}".format(
            observation["status"]
        ),
    )
    return observation


def test_texture_noise_not_full(config_path):
    observation = observe_new(config_path, textured_noise_frame())
    require(
        observation["status"] != "FULL",
        "floor texture/noise must not be classified FULL",
    )
    return observation


def test_outside_roi_pair_not_full(config_path):
    observation = observe_new(config_path, outside_roi_false_pair_frame())
    require(
        observation["status"] != "FULL",
        "a camera-edge false rail outside the calibrated ROI must not be FULL",
    )
    target_y = int(observation["lookahead_y"])
    roi = observation["analysis_roi_mask"]
    require(
        int(roi[target_y, 12]) == 0 and int(roi[target_y, 332]) != 0,
        "outside-ROI test did not isolate exactly one of its two rails",
    )
    return observation


def test_excessive_runs_fail_safe(config_path):
    observation = observe_new(config_path, excessive_runs_frame())
    require(
        observation["status"] == "RUN_OVERFLOW",
        "excessive row candidates must fail as RUN_OVERFLOW, got {}".format(
            observation["status"]
        ),
    )
    require(
        observation["run_overflow_row_fraction"] > 0.10,
        "overflow-row fraction must exceed the configured refusal threshold",
    )
    require(
        observation["maximum_raw_row_runs"] > 24,
        "synthetic overload did not exceed the 24-run candidate cap",
    )
    return observation


def test_fragmented_pair_not_full(config_path):
    observation = observe_new(config_path, fragmented_pair_frame())
    require(
        observation["status"] != "FULL",
        "rail fragments separated by more than the gap limit must not be FULL",
    )
    require(
        observation["paired_row_fraction"] < 0.40
        or observation["vertical_coverage_fraction"] < 0.45,
        "fragmented rails did not fail the configured support/coverage gate",
    )
    return observation


def test_distant_coverage_not_full(config_path):
    observation = observe_new(config_path, distant_supported_pair_frame())
    require(
        observation["status"] != "FULL",
        "rails with no trusted support near look-ahead must not be FULL",
    )
    require(
        observation["paired_row_fraction"] >= 0.40
        and observation["vertical_coverage_fraction"] >= 0.45,
        "distant-support test did not independently clear coverage gates",
    )
    require(
        observation["lookahead_support_distance_px"] is not None
        and observation["lookahead_support_distance_px"] > 8.0,
        "distant-support test accidentally observed the look-ahead row",
    )
    return observation


def test_non_identity_perspective_full(config_path):
    source_points = [
        [0.10, 0.96],
        [0.90, 0.96],
        [0.68, 0.22],
        [0.32, 0.22],
    ]
    destination_points = [
        [0.25, 1.0],
        [0.75, 1.0],
        [0.75, 0.0],
        [0.25, 0.0],
    ]
    perspective_path = write_config_variant(
        config_path,
        "ipm_config.synthetic_perspective.json",
        {
            "source_points_normalized": source_points,
            "destination_points_normalized": destination_points,
        },
    )
    source = normalized_points_to_pixels(source_points, WIDTH, HEIGHT)
    destination = normalized_points_to_pixels(
        destination_points, WIDTH, HEIGHT
    )
    camera_to_ipm = cv2.getPerspectiveTransform(source, destination)
    ipm_to_camera = np.linalg.inv(camera_to_ipm)
    require(
        np.max(np.abs(camera_to_ipm - np.eye(3))) > 0.10,
        "perspective regression accidentally used an identity homography",
    )
    camera_frame = cv2.warpPerspective(
        straight_frame(),
        ipm_to_camera,
        (WIDTH, HEIGHT),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(255, 255, 255),
    )
    observation = observe_new(perspective_path, camera_frame)
    require(
        observation["status"] == "FULL",
        "inverse-warped perspective lane must be FULL, got {}".format(
            observation["status"]
        ),
    )
    require(
        observation["confidence"] >= 0.55,
        "perspective lane confidence must satisfy the safety gate, got {:.3f}".format(
            observation["confidence"]
        ),
    )
    require(
        abs(observation["lane_offset"]) <= 0.03,
        "perspective lane offset must remain centred, got {:.4f}".format(
            observation["lane_offset"]
        ),
    )
    return observation


def test_asymmetric_spurious_does_not_corrupt_pair(config_path):
    baseline = observe_new(config_path, straight_frame())
    observation = observe_new(
        config_path, asymmetric_spurious_valid_pair_frame()
    )
    require(baseline["status"] == "FULL", "spurious baseline must be FULL")
    require(
        observation["maximum_raw_row_runs"] >= 3,
        "spurious rail did not enter the row-candidate set",
    )
    require(
        observation["status"] == "FULL",
        "asymmetric outlier corrupted the valid pair: {}".format(
            observation["status"]
        ),
    )
    require(
        abs(observation["lane_offset"] - baseline["lane_offset"]) <= 0.02,
        "asymmetric outlier changed lane offset by more than 0.02",
    )
    require(
        abs(observation["lane_width_px"] - baseline["lane_width_px"]) <= 4.0,
        "asymmetric outlier changed lane width by more than 4 pixels",
    )
    return observation


def test_asymmetric_spurious_not_false_full(config_path):
    observation = observe_new(config_path, asymmetric_spurious_only_frame())
    require(
        observation["maximum_raw_row_runs"] >= 2,
        "asymmetric fragments did not enter the row-candidate set",
    )
    require(
        observation["status"] != "FULL",
        "one rail plus asymmetric fragments created a false FULL lane",
    )
    require(
        observation["paired_row_fraction"]
        < float(IPMLaneDetector(config_path).config["minimum_paired_row_fraction"]),
        "asymmetric fragments unexpectedly met paired-row support",
    )
    return observation


def expect_config_rejected(config_path, filename, updates, fragments):
    invalid_path = write_config_variant(
        config_path, filename, updates
    )
    try:
        IPMLaneDetector(invalid_path)
    except ValueError as error:
        message = str(error)
        for fragment in fragments:
            require(
                fragment.lower() in message.lower(),
                "config rejection did not identify {}: {}".format(
                    fragment, message
                ),
            )
        return
    raise AssertionError(
        "unsafe config {} was accepted".format(filename)
    )


def test_lookahead_outside_scan_rejected(config_path):
    for label, lookahead in (("below", 0.20), ("above", 0.95)):
        expect_config_rejected(
            config_path,
            "invalid_lookahead_{}.json".format(label),
            {"lookahead_y_fraction": lookahead},
            ("lookahead_y_fraction", "scan"),
        )


def test_semantically_wrong_quad_order_rejected(config_path):
    with open(config_path, "r") as config_file:
        config = json.load(config_file)
    for key in (
        "source_points_normalized",
        "destination_points_normalized",
    ):
        correct = config[key]
        # A cyclic rotation remains convex, so this specifically exercises
        # BL, BR, TR, TL semantics rather than convexity alone.
        rotated = [correct[1], correct[2], correct[3], correct[0]]
        expect_config_rejected(
            config_path,
            "invalid_{}_order.json".format(key),
            {key: rotated},
            (key, "BL"),
        )


def test_config_validation_rejects_unsafe_values(directory, config_path):
    with open(config_path, "r") as config_file:
        base = json.load(config_file)

    cases = (
        ("maximum_row_runs", 3),
        ("detector_mode", "LEGACY_OR_UNKNOWN"),
        ("maximum_missing_scan_rows", 21),
    )
    for key, value in cases:
        invalid = dict(base)
        invalid[key] = value
        invalid_path = os.path.join(
            directory, "invalid_{}.json".format(key)
        )
        with open(invalid_path, "w") as config_file:
            json.dump(invalid, config_file)
            config_file.write("\n")
        rejected = False
        try:
            IPMLaneDetector(invalid_path)
        except ValueError as error:
            rejected = key in str(error)
        require(rejected, "unsafe config value {} was accepted".format(key))

    missing_mode = dict(base)
    del missing_mode["detector_mode"]
    missing_mode_path = os.path.join(directory, "missing_detector_mode.json")
    with open(missing_mode_path, "w") as config_file:
        json.dump(missing_mode, config_file)
        config_file.write("\n")
    rejected = False
    try:
        IPMLaneDetector(missing_mode_path)
    except ValueError as error:
        rejected = "detector_mode" in str(error)
    require(rejected, "a mode-less legacy config was silently accepted")


def test_safety_stops_non_full(non_full_observations):
    config = load_controller_config()
    config = dict(config)
    config["allow_partial_lane_motion"] = False
    supervisor = SafetySupervisor(config)
    valid_yolo = {
        "error": None,
        "has_result": True,
        "age_seconds": 0.01,
        "source_frame_lag": 0,
    }
    clear_obstacle = {"state": "CLEAR", "reason": "NO_OBSTACLE"}
    for name, observation in non_full_observations:
        decision = supervisor.evaluate(
            observation["status"],
            observation["confidence"],
            0.01,
            valid_yolo,
            clear_obstacle,
            now=1.0,
        )
        require(
            decision["state"] == "STOP",
            "SafetySupervisor must STOP for {} status {}, got {}".format(
                name, observation["status"], decision["state"]
            ),
        )


def test_safety_stops_stale_full(full_observation):
    config = dict(load_controller_config())
    supervisor = SafetySupervisor(config)
    clear_obstacle = {"state": "CLEAR", "reason": "NO_OBSTACLE"}
    valid_yolo = {
        "error": None,
        "has_result": True,
        "age_seconds": 0.01,
        "source_frame_lag": 0,
    }

    stale_camera = supervisor.evaluate(
        full_observation["status"],
        full_observation["confidence"],
        float(config["camera_stale_timeout_s"]) + 0.01,
        valid_yolo,
        clear_obstacle,
        now=1.0,
    )
    require(
        stale_camera["state"] == "STOP"
        and stale_camera["reason"] == "CAMERA_STALE",
        "stale camera data must produce CAMERA_STALE STOP",
    )

    stale_yolo = dict(valid_yolo)
    stale_yolo["age_seconds"] = float(config["yolo_stale_timeout_s"]) + 0.01
    stale_detector = supervisor.evaluate(
        full_observation["status"],
        full_observation["confidence"],
        0.01,
        stale_yolo,
        clear_obstacle,
        now=2.0,
    )
    require(
        stale_detector["state"] == "STOP"
        and stale_detector["reason"] == "YOLO_STALE",
        "stale YOLO data must produce YOLO_STALE STOP",
    )


def main():
    temporary_directory = tempfile.mkdtemp(prefix="sentinel_ipm_test_")
    try:
        config_path = write_test_config(temporary_directory)
        results = []

        tests = (
            ("straight_full", test_straight_full),
            ("curve_same_left_full", test_curve_both_rails_same_half),
            ("curve_same_right_full", test_curve_both_rails_same_right_half),
            ("concentric_oval_full", test_concentric_oval_arc),
            ("guarded_close_curve_full", test_guarded_close_curve),
            ("wide_row_curve_full", test_wide_row_component_curve),
            ("broad_blob_not_full", test_broad_blob_not_rail),
            ("crossbar_ignored", test_crossbar_ignored),
            ("invalid_width_not_full", test_invalid_width_not_full),
            ("single_rail_not_full", test_single_rail_after_full_not_full),
            ("blank_after_full_lost", test_blank_after_full_is_lost),
            ("texture_noise_not_full", test_texture_noise_not_full),
            ("outside_roi_not_full", test_outside_roi_pair_not_full),
            ("run_overflow_stop", test_excessive_runs_fail_safe),
            ("fragmented_not_full", test_fragmented_pair_not_full),
            ("distant_support_stop", test_distant_coverage_not_full),
            ("perspective_warp_full", test_non_identity_perspective_full),
            (
                "spurious_pair_preserved",
                test_asymmetric_spurious_does_not_corrupt_pair,
            ),
            (
                "spurious_not_false_full",
                test_asymmetric_spurious_not_false_full,
            ),
        )
        for name, test_function in tests:
            observation = test_function(config_path)
            results.append((name, observation))
            print(
                "PASS {:25s} status={:14s} conf={:.3f} offset={:+.4f}".format(
                    name,
                    observation["status"],
                    float(observation["confidence"]),
                    float(observation["lane_offset"]),
                )
            )

        non_full = [
            item
            for item in results
            if item[0]
            in (
                "invalid_width_not_full",
                "single_rail_not_full",
                "blank_after_full_lost",
                "texture_noise_not_full",
                "outside_roi_not_full",
                "run_overflow_stop",
                "broad_blob_not_full",
                "fragmented_not_full",
                "distant_support_stop",
                "spurious_not_false_full",
            )
        ]
        test_safety_stops_non_full(non_full)
        print("PASS safety_non_full_stop      {} cases".format(len(non_full)))
        straight_observation = dict(results)["straight_full"]
        test_safety_stops_stale_full(straight_observation)
        print("PASS safety_stale_full_stop    camera and YOLO")
        test_config_validation_rejects_unsafe_values(
            temporary_directory, config_path
        )
        print("PASS unsafe_config_rejected    4 cases")
        test_lookahead_outside_scan_rejected(config_path)
        print("PASS lookahead_scan_rejected   below and above")
        test_semantically_wrong_quad_order_rejected(config_path)
        print("PASS semantic_order_rejected   source/destination BL/BR/TR/TL")
        print("SYNTHETIC IPM REGRESSION: PASS")
        return 0
    except Exception as error:
        print("SYNTHETIC IPM REGRESSION: FAIL")
        print("{}: {}".format(type(error).__name__, error))
        return 1
    finally:
        shutil.rmtree(temporary_directory, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
