#!/usr/bin/env python3
"""Configurable inverse-perspective lane observation for the physical CSI view.

The default quadrilateral is a normalized port of the original WSL/SITL
``src/lane/lane_module.py`` geometry.  It is deliberately marked as an
unverified seed and must be calibrated against the real track before any
physical motion uses its output.
"""

from __future__ import print_function

import json
import math
import os
import warnings

import cv2
import numpy as np

from single_line_lane import (
    SINGLE_LINE_DEFAULTS,
    SINGLE_LINE_DETECTOR_MODE,
    detect_single_line,
    validate_single_line_config,
)


DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "ipm_config.json"
)
DETECTOR_IMPLEMENTATION_VERSION = "PAIRED_ROW_POLYNOMIAL_V2"
SUPPORTED_DETECTOR_MODES = (
    DETECTOR_IMPLEMENTATION_VERSION,
    SINGLE_LINE_DETECTOR_MODE,
)


# These defaults keep existing, physically calibrated schema-v1 configuration
# files readable while making the detector geometry explicit.  The deployment
# helper writes the same values into ipm_config.json and deliberately requires
# the physical IPM gates to be revalidated before motion can be authorised.
PAIRED_ROW_DEFAULTS = {
    "detector_mode": DETECTOR_IMPLEMENTATION_VERSION,
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
    # A close bend can make one tape rail nearly horizontal.  Horizontal
    # scan-run pairing then rejects that rail because its apparent run is too
    # wide.  V2 has a guarded connected-component fallback: both rails must be
    # individually continuous, overlap around look-ahead, retain a plausible
    # normal separation, and remain unambiguous.  These limits never relax the
    # normal detector; they only govern that second, current-frame proof.
    "close_curve_minimum_pairs": 8,
    "close_curve_minimum_paired_row_fraction": 0.20,
    "close_curve_minimum_vertical_coverage_fraction": 0.30,
    "close_curve_maximum_fit_residual_px": 3.0,
    "close_curve_maximum_lane_width_variation_fraction": 0.15,
    "close_curve_minimum_pair_quality": 0.75,
    "close_curve_maximum_width_error_fraction": 0.35,
    "close_curve_maximum_raw_row_runs": 8,
}


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _strict_number(config, key):
    value = config[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a JSON number.".format(key))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite.".format(key))
    return value


def _validate_quad(points, key):
    if not isinstance(points, list) or len(points) != 4:
        raise ValueError("{} must contain exactly four points.".format(key))
    parsed = []
    for point in points:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError("{} contains an invalid point.".format(key))
        coordinates = []
        for value in point:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("{} point coordinates must be numbers.".format(key))
            value = float(value)
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError("{} points must be finite and normalized.".format(key))
            coordinates.append(value)
        parsed.append(coordinates)

    bottom_left, bottom_right, top_right, top_left = parsed
    if not (
        bottom_left[0] < bottom_right[0]
        and top_left[0] < top_right[0]
    ):
        raise ValueError(
            "{} must use BL, BR, TR, TL order with left points left of "
            "right points.".format(key)
        )
    minimum_vertical_separation = 0.02
    if not (
        bottom_left[1] >= top_left[1] + minimum_vertical_separation
        and bottom_right[1] >= top_right[1] + minimum_vertical_separation
    ):
        raise ValueError(
            "{} must use BL, BR, TR, TL order with each bottom point "
            "below its corresponding top point by at least 0.02.".format(key)
        )

    twice_area = 0.0
    cross_signs = []
    for index in range(4):
        x1, y1 = parsed[index]
        x2, y2 = parsed[(index + 1) % 4]
        x3, y3 = parsed[(index + 2) % 4]
        twice_area += x1 * y2 - x2 * y1
        cross = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)
        if abs(cross) < 0.000001:
            raise ValueError("{} must be non-degenerate and convex.".format(key))
        cross_signs.append(1 if cross > 0.0 else -1)
    if abs(twice_area) < 0.04 or len(set(cross_signs)) != 1:
        raise ValueError("{} must be a non-degenerate convex quadrilateral.".format(key))
    return parsed


def load_ipm_config(path=DEFAULT_CONFIG_PATH):
    with open(path, "r") as config_file:
        config = json.load(config_file)

    required = (
        "detector_mode",
        "source_points_normalized",
        "destination_points_normalized",
        "black_threshold",
        "close_kernel_size",
        "hough_votes",
        "minimum_line_length",
        "maximum_line_gap",
        "minimum_vertical_ratio",
        "lookahead_y_fraction",
        "minimum_lane_width_fraction",
        "maximum_lane_width_fraction",
    )
    missing = [key for key in required if key not in config]
    if missing:
        raise ValueError("IPM config missing: {}".format(", ".join(missing)))

    for key, value in PAIRED_ROW_DEFAULTS.items():
        config.setdefault(key, value)
    if config.get("detector_mode") == SINGLE_LINE_DETECTOR_MODE:
        for key, value in SINGLE_LINE_DEFAULTS.items():
            config.setdefault(key, value)

    for key in ("source_points_normalized", "destination_points_normalized"):
        config[key] = _validate_quad(config[key], key)

    integer_keys = (
        "black_threshold",
        "close_kernel_size",
        "hough_votes",
        "minimum_line_length",
        "maximum_line_gap",
        "scan_row_step_px",
        "scan_band_height_px",
        "minimum_tape_run_width_px",
        "valid_roi_erode_px",
        "maximum_missing_scan_rows",
        "maximum_row_runs",
        "close_curve_minimum_pairs",
        "close_curve_maximum_raw_row_runs",
    )
    for key in integer_keys:
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("{} must be a JSON integer.".format(key))

    if not 1 <= config["black_threshold"] <= 254:
        raise ValueError("black_threshold must be in 1..254.")
    kernel_size = config["close_kernel_size"]
    if kernel_size < 1 or kernel_size > 31 or kernel_size % 2 == 0:
        raise ValueError("close_kernel_size must be a positive odd integer.")
    if not 5 <= config["hough_votes"] <= 300:
        raise ValueError("hough_votes must be in 5..300.")
    if not 5 <= config["minimum_line_length"] <= 1000:
        raise ValueError("minimum_line_length must be in 5..1000.")
    if not 0 <= config["maximum_line_gap"] <= 500:
        raise ValueError("maximum_line_gap must be in 0..500.")
    if not 2 <= config["scan_row_step_px"] <= 40:
        raise ValueError("scan_row_step_px must be in 2..40.")
    if (
        not 1 <= config["scan_band_height_px"] <= 31
        or config["scan_band_height_px"] % 2 == 0
    ):
        raise ValueError("scan_band_height_px must be an odd integer in 1..31.")
    if not 1 <= config["minimum_tape_run_width_px"] <= 30:
        raise ValueError("minimum_tape_run_width_px must be in 1..30.")
    if not 0 <= config["valid_roi_erode_px"] <= 31:
        raise ValueError("valid_roi_erode_px must be in 0..31.")
    if not 0 <= config["maximum_missing_scan_rows"] <= 20:
        raise ValueError("maximum_missing_scan_rows must be in 0..20.")
    if not 4 <= config["maximum_row_runs"] <= 128:
        raise ValueError("maximum_row_runs must be in 4..128.")
    if not 4 <= config["close_curve_minimum_pairs"] <= 40:
        raise ValueError("close_curve_minimum_pairs must be in 4..40.")
    if not (
        2 <= config["close_curve_maximum_raw_row_runs"]
        <= config["maximum_row_runs"]
    ):
        raise ValueError(
            "close_curve_maximum_raw_row_runs must be in "
            "2..maximum_row_runs."
        )

    if config["detector_mode"] not in SUPPORTED_DETECTOR_MODES:
        raise ValueError(
            "detector_mode must be one of: {}.".format(
                ", ".join(SUPPORTED_DETECTOR_MODES)
            )
        )
    if config["detector_mode"] == SINGLE_LINE_DETECTOR_MODE:
        if config.get("target_line_role") != "OUTER_CIRCLE_CENTERLINE":
            raise ValueError(
                "Single-line mode requires target_line_role="
                "OUTER_CIRCLE_CENTERLINE."
            )
        config = validate_single_line_config(config)

    vertical_ratio = _strict_number(config, "minimum_vertical_ratio")
    lookahead = _strict_number(config, "lookahead_y_fraction")
    minimum_width = _strict_number(config, "minimum_lane_width_fraction")
    maximum_width = _strict_number(config, "maximum_lane_width_fraction")
    scan_top = _strict_number(config, "scan_top_fraction")
    scan_bottom = _strict_number(config, "scan_bottom_fraction")
    column_fill = _strict_number(config, "minimum_column_fill_fraction")
    maximum_run_width = _strict_number(
        config, "maximum_tape_run_width_fraction"
    )
    maximum_step = _strict_number(config, "maximum_boundary_step_fraction")
    overflow_fraction = _strict_number(
        config, "maximum_run_overflow_row_fraction"
    )
    pair_width_tolerance = _strict_number(
        config, "pair_width_tolerance_fraction"
    )
    analysis_margin = _strict_number(
        config, "analysis_margin_lane_width_fraction"
    )
    paired_fraction = _strict_number(config, "minimum_paired_row_fraction")
    vertical_coverage = _strict_number(
        config, "minimum_vertical_coverage_fraction"
    )
    fit_residual = _strict_number(config, "fit_residual_threshold_px")
    fit_inliers = _strict_number(config, "minimum_fit_inlier_fraction")
    width_variation = _strict_number(
        config, "maximum_lane_width_variation_fraction"
    )
    width_window = _strict_number(
        config, "lane_width_evaluation_window_fraction"
    )
    close_curve_paired = _strict_number(
        config, "close_curve_minimum_paired_row_fraction"
    )
    close_curve_coverage = _strict_number(
        config, "close_curve_minimum_vertical_coverage_fraction"
    )
    close_curve_residual = _strict_number(
        config, "close_curve_maximum_fit_residual_px"
    )
    close_curve_width_variation = _strict_number(
        config, "close_curve_maximum_lane_width_variation_fraction"
    )
    close_curve_pair_quality = _strict_number(
        config, "close_curve_minimum_pair_quality"
    )
    close_curve_width_error = _strict_number(
        config, "close_curve_maximum_width_error_fraction"
    )
    if not 1.0 <= vertical_ratio <= 20.0:
        raise ValueError("minimum_vertical_ratio must be in 1..20.")
    if not 0.10 <= lookahead <= 0.95:
        raise ValueError("lookahead_y_fraction must be in 0.10..0.95.")
    if not 0.05 <= minimum_width < maximum_width <= 0.95:
        raise ValueError(
            "Expected 0.05 <= minimum_lane_width_fraction < "
            "maximum_lane_width_fraction <= 0.95."
        )
    if not 0.05 <= scan_top < scan_bottom <= 0.99:
        raise ValueError(
            "Expected 0.05 <= scan_top_fraction < scan_bottom_fraction <= 0.99."
        )
    if not scan_top <= lookahead <= scan_bottom:
        raise ValueError(
            "lookahead_y_fraction must lie inside scan_top_fraction.."
            "scan_bottom_fraction."
        )
    if not 0.10 <= column_fill <= 1.0:
        raise ValueError("minimum_column_fill_fraction must be in 0.10..1.0.")
    if not 0.01 <= maximum_run_width <= 0.25:
        raise ValueError(
            "maximum_tape_run_width_fraction must be in 0.01..0.25."
        )
    if not 0.01 <= maximum_step <= 0.30:
        raise ValueError("maximum_boundary_step_fraction must be in 0.01..0.30.")
    if not 0.0 <= overflow_fraction <= 1.0:
        raise ValueError(
            "maximum_run_overflow_row_fraction must be in 0..1."
        )
    if not 0.10 <= pair_width_tolerance <= 1.0:
        raise ValueError("pair_width_tolerance_fraction must be in 0.10..1.0.")
    if not 0.0 <= analysis_margin <= 1.0:
        raise ValueError(
            "analysis_margin_lane_width_fraction must be in 0..1."
        )
    if not 0.10 <= paired_fraction <= 1.0:
        raise ValueError("minimum_paired_row_fraction must be in 0.10..1.0.")
    if not 0.10 <= vertical_coverage <= 1.0:
        raise ValueError(
            "minimum_vertical_coverage_fraction must be in 0.10..1.0."
        )
    if not 1.0 <= fit_residual <= 100.0:
        raise ValueError("fit_residual_threshold_px must be in 1..100.")
    if not 0.50 <= fit_inliers <= 1.0:
        raise ValueError("minimum_fit_inlier_fraction must be in 0.50..1.0.")
    if not 0.05 <= width_variation <= 1.0:
        raise ValueError(
            "maximum_lane_width_variation_fraction must be in 0.05..1.0."
        )
    if not 0.10 <= width_window <= 1.0:
        raise ValueError(
            "lane_width_evaluation_window_fraction must be in 0.10..1.0."
        )
    if not 0.10 <= close_curve_paired <= paired_fraction:
        raise ValueError(
            "close_curve_minimum_paired_row_fraction must be in "
            "0.10..minimum_paired_row_fraction."
        )
    if not 0.10 <= close_curve_coverage <= vertical_coverage:
        raise ValueError(
            "close_curve_minimum_vertical_coverage_fraction must be in "
            "0.10..minimum_vertical_coverage_fraction."
        )
    if not 0.5 <= close_curve_residual <= fit_residual:
        raise ValueError(
            "close_curve_maximum_fit_residual_px must be in "
            "0.5..fit_residual_threshold_px."
        )
    if not 0.05 <= close_curve_width_variation <= width_variation:
        raise ValueError(
            "close_curve_maximum_lane_width_variation_fraction must be in "
            "0.05..maximum_lane_width_variation_fraction."
        )
    if not 0.50 <= close_curve_pair_quality <= 1.0:
        raise ValueError(
            "close_curve_minimum_pair_quality must be in 0.50..1.0."
        )
    if not 0.10 <= close_curve_width_error <= pair_width_tolerance:
        raise ValueError(
            "close_curve_maximum_width_error_fraction must be in "
            "0.10..pair_width_tolerance_fraction."
        )
    return config


def normalized_points_to_pixels(points, width, height):
    return np.float32(
        [
            [
                float(point[0]) * max(1, width - 1),
                float(point[1]) * max(1, height - 1),
            ]
            for point in points
        ]
    )


class IPMLaneDetector(object):
    """Warp a camera frame and estimate the black-tape lane centre."""

    def __init__(self, config_path=DEFAULT_CONFIG_PATH):
        self.config_path = os.path.abspath(config_path)
        self.config = load_ipm_config(self.config_path)
        self.remembered_lane_width = None
        self.remembered_lane_centre = None
        # Camera geometry and the analysis masks are invariant for a given
        # calibrated detector and frame size.  Building and warping these
        # full-frame masks on every observation wastes a material part of the
        # Jetson Nano's 250 ms freshness budget, so prepare them once and
        # reuse them without changing any detector thresholds or semantics.
        self._geometry_cache = {}
        close_kernel_size = int(self.config["close_kernel_size"])
        self._close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (close_kernel_size, close_kernel_size)
        )
        self._opening_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (3, 3)
        )

    @property
    def physically_calibrated(self):
        return self.config.get("calibration_state") == "PHYSICALLY_CALIBRATED"

    def _perspective(self, width, height):
        source = normalized_points_to_pixels(
            self.config["source_points_normalized"], width, height
        )
        destination = normalized_points_to_pixels(
            self.config["destination_points_normalized"], width, height
        )
        matrix = cv2.getPerspectiveTransform(source, destination)
        return source, destination, matrix

    def _prepared_geometry(self, width, height):
        """Return immutable, cached geometry for one camera frame size."""
        key = (int(width), int(height))
        prepared = self._geometry_cache.get(key)
        if prepared is None:
            source, destination, matrix = self._perspective(width, height)
            frame_shape = (int(height), int(width), 3)
            valid_warp_mask = self._valid_warp_mask(
                frame_shape,
                matrix,
                int(self.config["valid_roi_erode_px"]),
            )
            analysis_roi_mask = self._analysis_roi_mask(
                frame_shape, destination
            )
            analysis_mask = cv2.bitwise_and(
                valid_warp_mask, analysis_roi_mask
            )
            prepared = {
                "source": source,
                "destination": destination,
                "matrix": matrix,
                "valid_warp_mask": valid_warp_mask,
                "analysis_roi_mask": analysis_roi_mask,
                "analysis_mask": analysis_mask,
            }
            self._geometry_cache[key] = prepared
        return prepared

    def warp(self, frame):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("Expected a non-empty BGR frame.")
        height, width = frame.shape[:2]
        prepared = self._prepared_geometry(width, height)
        source = prepared["source"]
        destination = prepared["destination"]
        matrix = prepared["matrix"]
        # White is deliberately used outside the valid source image.  A black
        # border would otherwise look exactly like the black track tape.
        warped = cv2.warpPerspective(
            frame,
            matrix,
            (width, height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(255, 255, 255),
        )
        return warped, source, destination, matrix

    @staticmethod
    def _valid_warp_mask(frame_shape, matrix, erode_px):
        height, width = frame_shape[:2]
        source_valid = np.full((height, width), 255, dtype=np.uint8)
        valid = cv2.warpPerspective(
            source_valid,
            matrix,
            (width, height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        if erode_px > 0:
            size = 2 * int(erode_px) + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))
            valid = cv2.erode(valid, kernel)
        return valid

    def _analysis_roi_mask(self, frame_shape, destination):
        """Return an expanded track ROI that still excludes image extremes."""
        height, width = frame_shape[:2]
        if self.config["detector_mode"] == SINGLE_LINE_DETECTOR_MODE:
            x_start = max(0, int(math.floor(
                width
                * float(self.config["single_line_roi_left_fraction"])
            )))
            x_stop = min(width - 1, int(math.ceil(
                width
                * float(self.config["single_line_roi_right_fraction"])
            )))
            y_start = max(0, int(math.floor(
                height
                * float(self.config["single_line_scan_top_fraction"])
            )))
            y_stop = min(height - 1, int(math.ceil(
                height
                * float(self.config["single_line_scan_bottom_fraction"])
            )))
            roi = np.zeros((height, width), dtype=np.uint8)
            cv2.rectangle(
                roi, (x_start, y_start), (x_stop, y_stop), 255, -1
            )
            return roi

        left_edge = min(float(destination[0][0]), float(destination[3][0]))
        right_edge = max(float(destination[1][0]), float(destination[2][0]))
        bottom_width = abs(float(destination[1][0]) - float(destination[0][0]))
        top_width = abs(float(destination[2][0]) - float(destination[3][0]))
        nominal_width = max(1.0, (bottom_width + top_width) / 2.0)
        margin = nominal_width * float(
            self.config["analysis_margin_lane_width_fraction"]
        )
        x_start = max(0, int(math.floor(left_edge - margin)))
        x_stop = min(width - 1, int(math.ceil(right_edge + margin)))
        band_half = int(self.config["scan_band_height_px"]) // 2
        y_start = max(
            0,
            int(round(height * float(self.config["scan_top_fraction"])))
            - band_half,
        )
        y_stop = min(
            height - 1,
            int(round(height * float(self.config["scan_bottom_fraction"])))
            + band_half,
        )
        roi = np.zeros((height, width), dtype=np.uint8)
        cv2.rectangle(roi, (x_start, y_start), (x_stop, y_stop), 255, -1)
        return roi

    @staticmethod
    def _row_runs(mask, y_value, band_height, minimum_fill, minimum_width,
                  maximum_width):
        """Extract narrow dark-tape runs from one horizontal image band."""
        height, width = mask.shape[:2]
        half = int(band_height) // 2
        y_start = max(0, int(y_value) - half)
        y_stop = min(height, int(y_value) + half + 1)
        if y_stop <= y_start:
            return []

        profile = np.mean(mask[y_start:y_stop] > 0, axis=0)
        active = profile >= float(minimum_fill)
        runs = []
        start = None
        for x_value in range(width + 1):
            is_active = x_value < width and bool(active[x_value])
            if is_active and start is None:
                start = x_value
            elif not is_active and start is not None:
                stop = x_value
                run_width = stop - start
                if int(minimum_width) <= run_width <= int(maximum_width):
                    weights = profile[start:stop]
                    total = float(np.sum(weights))
                    if total > 0.0:
                        positions = np.arange(start, stop, dtype=np.float64)
                        centre = float(np.sum(positions * weights) / total)
                        runs.append(
                            {
                                "x": centre,
                                "width": int(run_width),
                                "quality": float(np.mean(weights)),
                            }
                        )
                start = None
        return runs

    @staticmethod
    def _pair_candidates(runs, minimum_width, maximum_width, expected_width,
                         tolerance_fraction):
        pairs = []
        ordered = sorted(runs, key=lambda item: item["x"])
        tolerated_minimum = max(
            minimum_width,
            expected_width * (1.0 - tolerance_fraction),
        )
        tolerated_maximum = min(
            maximum_width,
            expected_width * (1.0 + tolerance_fraction),
        )
        for left_index in range(len(ordered)):
            for right_index in range(left_index + 1, len(ordered)):
                left = ordered[left_index]
                right = ordered[right_index]
                width = float(right["x"] - left["x"])
                if tolerated_minimum <= width <= tolerated_maximum:
                    pairs.append(
                        {
                            "left": left,
                            "right": right,
                            "width": width,
                            "centre": (left["x"] + right["x"]) / 2.0,
                            "quality": (
                                float(left["quality"])
                                + float(right["quality"])
                            )
                            / 2.0,
                        }
                    )
        return pairs

    @staticmethod
    def _pair_score(pair, expected_width, reference_centre):
        width_error = abs(float(pair["width"]) - expected_width) / max(
            1.0, expected_width
        )
        centre_error = abs(float(pair["centre"]) - reference_centre) / max(
            1.0, expected_width
        )
        return width_error + 0.25 * centre_error - 0.15 * pair["quality"]

    def _trace_pairs(self, rows, target_y, expected_width, reference_centre,
                     maximum_step):
        available = [index for index, row in enumerate(rows) if row["pairs"]]
        if not available:
            return []

        seed_index = min(
            available,
            key=lambda index: (
                abs(rows[index]["y"] - target_y),
                min(
                    self._pair_score(
                        pair, expected_width, reference_centre
                    )
                    for pair in rows[index]["pairs"]
                ),
            ),
        )
        seed = min(
            rows[seed_index]["pairs"],
            key=lambda pair: self._pair_score(
                pair, expected_width, reference_centre
            ),
        )
        selected = {seed_index: seed}
        row_step = max(1, int(self.config["scan_row_step_px"]))

        for direction in (-1, 1):
            last_index = seed_index
            last_pair = seed
            missing_rows = 0
            index = seed_index + direction
            while 0 <= index < len(rows):
                candidates = []
                if rows[index]["pairs"]:
                    row_distance = abs(rows[index]["y"] - rows[last_index]["y"])
                    allowed_step = maximum_step * max(
                        1.0, float(row_distance) / float(row_step)
                    )
                    for pair in rows[index]["pairs"]:
                        left_step = abs(
                            pair["left"]["x"] - last_pair["left"]["x"]
                        )
                        right_step = abs(
                            pair["right"]["x"] - last_pair["right"]["x"]
                        )
                        if left_step <= allowed_step and right_step <= allowed_step:
                            motion_cost = (left_step + right_step) / max(
                                1.0, 2.0 * allowed_step
                            )
                            width_cost = abs(pair["width"] - expected_width) / max(
                                1.0, expected_width
                            )
                            candidates.append(
                                (
                                    motion_cost
                                    + 0.40 * width_cost
                                    - 0.10 * pair["quality"],
                                    pair,
                                )
                            )
                if candidates:
                    candidates.sort(key=lambda item: item[0])
                    selected[index] = candidates[0][1]
                    last_index = index
                    last_pair = candidates[0][1]
                    missing_rows = 0
                else:
                    missing_rows += 1
                    if missing_rows > int(
                        self.config["maximum_missing_scan_rows"]
                    ):
                        break
                index += direction

        return [
            (rows[index]["y"], selected[index])
            for index in sorted(selected)
        ]

    @staticmethod
    def _robust_quadratic(points, residual_threshold, minimum_inlier_fraction):
        if len(points) < 3:
            return None
        y_values = np.float64([point[0] for point in points])
        x_values = np.float64([point[1] for point in points])
        rank_warning = getattr(np, "RankWarning", None)
        if rank_warning is None:
            rank_warning = getattr(
                getattr(np, "exceptions", None), "RankWarning", Warning
            )
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("error", rank_warning)
                coefficients = np.polyfit(y_values, x_values, 2)
        except (ValueError, rank_warning, np.linalg.LinAlgError):
            return None

        residuals = np.abs(np.polyval(coefficients, y_values) - x_values)
        inliers = residuals <= float(residual_threshold)
        inlier_fraction = float(np.mean(inliers))
        if np.count_nonzero(inliers) < 3 or inlier_fraction < minimum_inlier_fraction:
            return None

        try:
            with warnings.catch_warnings():
                warnings.simplefilter("error", rank_warning)
                coefficients = np.polyfit(y_values[inliers], x_values[inliers], 2)
        except (ValueError, rank_warning, np.linalg.LinAlgError):
            return None
        if not np.all(np.isfinite(coefficients)):
            return None

        final_all_residuals = np.abs(
            np.polyval(coefficients, y_values) - x_values
        )
        final_inliers = final_all_residuals <= float(residual_threshold)
        final_inlier_fraction = float(np.mean(final_inliers))
        if (
            np.count_nonzero(final_inliers) < 3
            or final_inlier_fraction < minimum_inlier_fraction
        ):
            return None
        final_residuals = final_all_residuals[final_inliers]
        return {
            "coefficients": coefficients,
            "mean_residual": float(np.mean(final_residuals)),
            "inlier_fraction": final_inlier_fraction,
            "inlier_count": int(np.count_nonzero(final_inliers)),
            "inlier_indices": [
                int(index)
                for index, accepted in enumerate(final_inliers.tolist())
                if accepted
            ],
        }

    @staticmethod
    def _shared_quadratic_fits(traced, preliminary_left, preliminary_right,
                               residual_threshold):
        """Refit both rails using one identical, jointly trusted row set."""
        shared_indices = sorted(
            set(preliminary_left["inlier_indices"]).intersection(
                preliminary_right["inlier_indices"]
            )
        )
        rank_warning = getattr(np, "RankWarning", None)
        if rank_warning is None:
            rank_warning = getattr(
                getattr(np, "exceptions", None), "RankWarning", Warning
            )

        final_left = None
        final_right = None
        while len(shared_indices) >= 3:
            y_values = np.float64(
                [traced[index][0] for index in shared_indices]
            )
            left_values = np.float64(
                [traced[index][1]["left"]["x"] for index in shared_indices]
            )
            right_values = np.float64(
                [traced[index][1]["right"]["x"] for index in shared_indices]
            )
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("error", rank_warning)
                    left_coefficients = np.polyfit(y_values, left_values, 2)
                    right_coefficients = np.polyfit(y_values, right_values, 2)
            except (ValueError, rank_warning, np.linalg.LinAlgError):
                return None, None, []
            if not (
                np.all(np.isfinite(left_coefficients))
                and np.all(np.isfinite(right_coefficients))
            ):
                return None, None, []

            left_residuals = np.abs(
                np.polyval(left_coefficients, y_values) - left_values
            )
            right_residuals = np.abs(
                np.polyval(right_coefficients, y_values) - right_values
            )
            accepted_positions = [
                position
                for position in range(len(shared_indices))
                if left_residuals[position] <= float(residual_threshold)
                and right_residuals[position] <= float(residual_threshold)
            ]
            reduced_indices = [
                shared_indices[position] for position in accepted_positions
            ]
            if reduced_indices == shared_indices:
                total = max(1.0, float(len(traced)))
                shared_fraction = float(len(shared_indices)) / total
                final_left = {
                    "coefficients": left_coefficients,
                    "mean_residual": float(np.mean(left_residuals)),
                    "inlier_fraction": shared_fraction,
                    "inlier_count": len(shared_indices),
                    "inlier_indices": list(shared_indices),
                }
                final_right = {
                    "coefficients": right_coefficients,
                    "mean_residual": float(np.mean(right_residuals)),
                    "inlier_fraction": shared_fraction,
                    "inlier_count": len(shared_indices),
                    "inlier_indices": list(shared_indices),
                }
                break
            shared_indices = reduced_indices

        if final_left is None or final_right is None:
            return None, None, []
        return final_left, final_right, shared_indices

    @staticmethod
    def _component_edge_fit(points, degree=3):
        """Fit one continuous component edge without extrapolating support."""
        if len(points) < degree + 1:
            return None
        y_values = np.float64([point[0] for point in points])
        x_values = np.float64([point[1] for point in points])
        rank_warning = getattr(np, "RankWarning", None)
        if rank_warning is None:
            rank_warning = getattr(
                getattr(np, "exceptions", None), "RankWarning", Warning
            )
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("error", rank_warning)
                coefficients = np.polyfit(y_values, x_values, degree)
        except (ValueError, rank_warning, np.linalg.LinAlgError):
            return None
        if not np.all(np.isfinite(coefficients)):
            return None
        residuals = np.abs(np.polyval(coefficients, y_values) - x_values)
        return {
            "coefficients": coefficients,
            "mean_residual": float(np.mean(residuals)),
            "inlier_fraction": 1.0,
            "inlier_count": int(len(points)),
            "inlier_indices": list(range(len(points))),
        }

    def _component_curve_fallback(
        self,
        black_mask,
        rows,
        scan_top,
        scan_bottom,
        target_y,
        expected_width,
        minimum_width,
        maximum_width,
    ):
        """Prove a close curve from two continuous current-frame rail blobs.

        This deliberately does not bridge fragments or infer a missing rail.
        It accepts only one unambiguous pair of connected components which
        both reach look-ahead and whose nearest-boundary distance is stable.
        """
        height, width = black_mask.shape[:2]
        scan_span = max(1.0, float(scan_bottom - scan_top))
        step = int(self.config["scan_row_step_px"])
        target_tolerance = float(
            step + int(self.config["scan_band_height_px"]) // 2
        )
        minimum_component_span = (
            scan_span
            * float(self.config[
                "close_curve_minimum_vertical_coverage_fraction"
            ])
        )
        minimum_component_area = max(100, int(round(width * height * 0.0035)))

        binary = np.uint8(black_mask > 0)
        component_count, labels, stats, centroids = (
            cv2.connectedComponentsWithStats(binary, 8)
        )
        candidates = []
        component_masks = {}
        maximum_tape_diameter = max(
            float(self.config["minimum_tape_run_width_px"]),
            width
            * float(self.config["maximum_tape_run_width_fraction"]),
        )
        for label in range(1, component_count):
            component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
            area = int(stats[label, cv2.CC_STAT_AREA])
            top = int(stats[label, cv2.CC_STAT_TOP])
            bottom = top + component_height - 1
            if area < minimum_component_area:
                continue
            if component_height < minimum_component_span:
                continue
            if (
                target_y < top - target_tolerance
                or target_y > bottom + target_tolerance
            ):
                continue
            component_mask = np.uint8(labels == label)
            # A rail may be almost horizontal and therefore wide in an image
            # row, but it must remain physically tape-like in its local normal
            # direction.  Twice the maximum interior distance is the diameter
            # of the largest circle that fits inside the component.  Filled
            # dark objects/blobs exceed the configured maximum tape width and
            # cannot act as either rail.
            interior_distance = cv2.distanceTransform(
                component_mask, cv2.DIST_L2, 5
            )
            maximum_inscribed_diameter = 2.0 * float(
                np.max(interior_distance)
            )
            if maximum_inscribed_diameter > maximum_tape_diameter:
                continue
            component_masks[label] = component_mask
            candidates.append(label)

        if len(candidates) < 2:
            return None

        scan_y_values = [int(row["y"]) for row in rows]
        minimum_rows = max(
            int(self.config["close_curve_minimum_pairs"]),
            int(math.ceil(
                len(rows)
                * float(self.config[
                    "close_curve_minimum_paired_row_fraction"
                ])
            )),
        )
        maximum_raw_runs = max(
            [int(row["raw_run_count"]) for row in rows] or [0]
        )
        if maximum_raw_runs > int(
            self.config["close_curve_maximum_raw_row_runs"]
        ):
            return None

        accepted_pairs = []
        for left_label in candidates:
            for right_label in candidates:
                if left_label == right_label:
                    continue
                if not (
                    float(centroids[left_label][0])
                    < float(centroids[right_label][0])
                ):
                    continue

                shared = []
                shared_presence_rows = 0
                for y_value in scan_y_values:
                    left_pixels = np.where(labels[y_value] == left_label)[0]
                    right_pixels = np.where(labels[y_value] == right_label)[0]
                    if left_pixels.size == 0 or right_pixels.size == 0:
                        continue
                    shared_presence_rows += 1
                    # Inward component edges describe the drivable gap even
                    # when the tape itself is almost horizontal in this row.
                    left_x = float(np.max(left_pixels))
                    right_x = float(np.min(right_pixels))
                    lane_width = right_x - left_x
                    if minimum_width <= lane_width <= maximum_width:
                        shared.append((y_value, left_x, right_x))

                if len(shared) < minimum_rows or shared_presence_rows <= 0:
                    continue
                paired_fraction = float(len(shared)) / max(1.0, float(len(rows)))
                y_values = [item[0] for item in shared]
                vertical_coverage = (
                    float(max(y_values) - min(y_values)) / scan_span
                )
                if paired_fraction < float(
                    self.config["close_curve_minimum_paired_row_fraction"]
                ):
                    continue
                if vertical_coverage < float(
                    self.config[
                        "close_curve_minimum_vertical_coverage_fraction"
                    ]
                ):
                    continue
                lookahead_distance = min(
                    abs(float(y_value) - float(target_y))
                    for y_value in y_values
                )
                if lookahead_distance > target_tolerance:
                    continue

                left_points = [(item[0], item[1]) for item in shared]
                right_points = [(item[0], item[2]) for item in shared]
                left_fit = self._component_edge_fit(left_points, degree=3)
                right_fit = self._component_edge_fit(right_points, degree=3)
                if left_fit is None or right_fit is None:
                    continue
                fit_residual = (
                    float(left_fit["mean_residual"])
                    + float(right_fit["mean_residual"])
                ) / 2.0
                if fit_residual > float(
                    self.config["close_curve_maximum_fit_residual_px"]
                ):
                    continue

                overlap_top = max(scan_top, min(y_values))
                overlap_bottom = min(scan_bottom, max(y_values))
                y_grid = np.indices(binary.shape)[0]
                overlap_mask = (
                    (y_grid >= overlap_top) & (y_grid <= overlap_bottom)
                )
                left_component = component_masks[left_label]
                right_component = component_masks[right_label]
                kernel = np.ones((3, 3), dtype=np.uint8)
                left_boundary = (
                    cv2.morphologyEx(
                        left_component, cv2.MORPH_GRADIENT, kernel
                    )
                    > 0
                ) & overlap_mask
                right_boundary = (
                    cv2.morphologyEx(
                        right_component, cv2.MORPH_GRADIENT, kernel
                    )
                    > 0
                ) & overlap_mask
                if not np.any(left_boundary) or not np.any(right_boundary):
                    continue
                distance_to_left = cv2.distanceTransform(
                    np.uint8(1 - left_component), cv2.DIST_L2, 5
                )
                distance_to_right = cv2.distanceTransform(
                    np.uint8(1 - right_component), cv2.DIST_L2, 5
                )
                directed_distributions = (
                    distance_to_right[left_boundary],
                    distance_to_left[right_boundary],
                )
                separation_options = []
                for distribution in directed_distributions:
                    if distribution.size < minimum_rows:
                        continue
                    p10, median, p90 = np.percentile(
                        distribution, [10.0, 50.0, 90.0]
                    )
                    variation = float(p90 - p10) / max(1.0, float(median))
                    separation_options.append(
                        (variation, float(median), float(p10), float(p90))
                    )
                if not separation_options:
                    continue
                separation_options.sort(key=lambda item: item[0])
                separation_variation, normal_width, p10, p90 = (
                    separation_options[0]
                )
                if not minimum_width <= normal_width <= maximum_width:
                    continue
                if separation_variation > float(
                    self.config[
                        "close_curve_maximum_lane_width_variation_fraction"
                    ]
                ):
                    continue
                width_error_fraction = abs(
                    normal_width - expected_width
                ) / max(1.0, expected_width)
                if width_error_fraction > float(
                    self.config["close_curve_maximum_width_error_fraction"]
                ):
                    continue

                pair_quality = float(len(shared)) / float(
                    shared_presence_rows
                )
                if pair_quality < float(
                    self.config["close_curve_minimum_pair_quality"]
                ):
                    continue

                left_x = float(np.polyval(
                    left_fit["coefficients"], target_y
                ))
                right_x = float(np.polyval(
                    right_fit["coefficients"], target_y
                ))
                if not (
                    math.isfinite(left_x)
                    and math.isfinite(right_x)
                    and 0.0 <= left_x < right_x <= width - 1
                    and minimum_width <= right_x - left_x <= maximum_width
                ):
                    continue

                support_quality = min(
                    1.0,
                    paired_fraction / float(self.config[
                        "close_curve_minimum_paired_row_fraction"
                    ]),
                    vertical_coverage / float(self.config[
                        "close_curve_minimum_vertical_coverage_fraction"
                    ]),
                )
                stability_quality = clamp(
                    1.0 - separation_variation / float(self.config[
                        "close_curve_maximum_lane_width_variation_fraction"
                    ]),
                    0.0,
                    1.0,
                )
                width_agreement_quality = clamp(
                    1.0 - width_error_fraction / float(self.config[
                        "close_curve_maximum_width_error_fraction"
                    ]),
                    0.0,
                    1.0,
                )
                fit_quality = clamp(
                    1.0 - fit_residual / float(self.config[
                        "close_curve_maximum_fit_residual_px"
                    ]),
                    0.0,
                    1.0,
                )
                score = (
                    0.30 * support_quality
                    + 0.25 * stability_quality
                    + 0.20 * pair_quality
                    + 0.15 * width_agreement_quality
                    + 0.10 * fit_quality
                )
                traced = []
                for y_value, edge_left, edge_right in shared:
                    traced.append(
                        (
                            y_value,
                            {
                                "left": {
                                    "x": edge_left,
                                    "width": 1,
                                    "quality": pair_quality,
                                },
                                "right": {
                                    "x": edge_right,
                                    "width": 1,
                                    "quality": pair_quality,
                                },
                                "width": edge_right - edge_left,
                                "centre": (edge_left + edge_right) / 2.0,
                                "quality": pair_quality,
                            },
                        )
                    )
                accepted_pairs.append(
                    {
                        "score": float(score),
                        "traced": traced,
                        "left_fit": left_fit,
                        "right_fit": right_fit,
                        "left_x": left_x,
                        "right_x": right_x,
                        "lane_width": right_x - left_x,
                        "lane_centre": (left_x + right_x) / 2.0,
                        "normal_lane_width": normal_width,
                        "paired_fraction": paired_fraction,
                        "vertical_coverage": vertical_coverage,
                        "fit_residual": fit_residual,
                        "lookahead_support_distance": lookahead_distance,
                        "width_variation": separation_variation,
                        "pair_quality": pair_quality,
                        "width_error_fraction": width_error_fraction,
                        "separation_p10": p10,
                        "separation_p90": p90,
                    }
                )

        if not accepted_pairs:
            return None
        accepted_pairs.sort(key=lambda item: item["score"], reverse=True)
        if (
            len(accepted_pairs) > 1
            and accepted_pairs[0]["score"] - accepted_pairs[1]["score"] < 0.10
        ):
            # Two similarly convincing rail pairs are ambiguous clutter.
            return None
        return accepted_pairs[0]

    def _detect_boundaries(self, black_mask, destination):
        """Track two current-frame tape rails without assuming image halves."""
        height, width = black_mask.shape[:2]
        scan_top = int(round(height * float(self.config["scan_top_fraction"])))
        scan_bottom = int(
            round(height * float(self.config["scan_bottom_fraction"]))
        )
        scan_top = max(0, min(height - 1, scan_top))
        scan_bottom = max(scan_top + 1, min(height - 1, scan_bottom))
        step = int(self.config["scan_row_step_px"])
        scan_y_values = list(range(scan_bottom, scan_top - 1, -step))
        target_y = int(height * float(self.config["lookahead_y_fraction"]))
        minimum_width = width * float(
            self.config["minimum_lane_width_fraction"]
        )
        maximum_width = width * float(
            self.config["maximum_lane_width_fraction"]
        )
        expected_width = abs(float(destination[1][0]) - float(destination[0][0]))
        if self.remembered_lane_width is not None:
            expected_width = float(self.remembered_lane_width)
        reference_centre = (
            float(self.remembered_lane_centre)
            if self.remembered_lane_centre is not None
            else (float(destination[0][0]) + float(destination[1][0])) / 2.0
        )
        maximum_run_width = max(
            int(self.config["minimum_tape_run_width_px"]),
            int(round(width * float(
                self.config["maximum_tape_run_width_fraction"]
            ))),
        )

        rows = []
        invalid_width_seen = False
        overflow_rows = 0
        maximum_row_runs = int(self.config["maximum_row_runs"])
        pair_width_tolerance = float(
            self.config["pair_width_tolerance_fraction"]
        )
        for y_value in scan_y_values:
            runs = self._row_runs(
                black_mask,
                y_value,
                int(self.config["scan_band_height_px"]),
                float(self.config["minimum_column_fill_fraction"]),
                int(self.config["minimum_tape_run_width_px"]),
                maximum_run_width,
            )
            raw_run_count = len(runs)
            if raw_run_count > maximum_row_runs:
                overflow_rows += 1
                runs = sorted(
                    runs,
                    key=lambda item: (
                        -float(item["quality"]) * float(item["width"]),
                        float(item["x"]),
                    ),
                )[:maximum_row_runs]
                runs.sort(key=lambda item: item["x"])
            pairs = self._pair_candidates(
                runs,
                minimum_width,
                maximum_width,
                expected_width,
                pair_width_tolerance,
            )
            if len(runs) >= 2 and not pairs:
                invalid_width_seen = True
            rows.append(
                {
                    "y": y_value,
                    "runs": runs,
                    "pairs": pairs,
                    "raw_run_count": raw_run_count,
                }
            )

        traced = self._trace_pairs(
            rows,
            target_y,
            expected_width,
            reference_centre,
            width * float(self.config["maximum_boundary_step_fraction"]),
        )
        left_points = [(y_value, pair["left"]["x"]) for y_value, pair in traced]
        right_points = [
            (y_value, pair["right"]["x"]) for y_value, pair in traced
        ]
        residual_threshold = float(self.config["fit_residual_threshold_px"])
        minimum_inliers = float(self.config["minimum_fit_inlier_fraction"])
        preliminary_left_fit = self._robust_quadratic(
            left_points, residual_threshold, minimum_inliers
        )
        preliminary_right_fit = self._robust_quadratic(
            right_points, residual_threshold, minimum_inliers
        )

        left_fit = None
        right_fit = None
        trusted_traced = []
        if preliminary_left_fit is not None and preliminary_right_fit is not None:
            left_fit, right_fit, common_inliers = self._shared_quadratic_fits(
                traced,
                preliminary_left_fit,
                preliminary_right_fit,
                residual_threshold,
            )
            trusted_traced = [traced[index] for index in common_inliers]

        paired_fraction = float(len(trusted_traced)) / max(
            1.0, float(len(rows))
        )
        vertical_coverage = 0.0
        if len(trusted_traced) >= 2:
            y_values = [item[0] for item in trusted_traced]
            vertical_coverage = (max(y_values) - min(y_values)) / max(
                1.0, float(scan_bottom - scan_top)
            )
        run_overflow_fraction = float(overflow_rows) / max(
            1.0, float(len(rows))
        )

        left_x = None
        right_x = None
        lane_width = None
        lane_centre = None
        width_variation = 1.0
        fit_residual = None
        lookahead_support_distance = None
        status = "LOST"
        support_mode = "NONE"
        normal_lane_width = None
        widths = []
        pair_quality = 0.0
        width_error_fraction = 1.0

        if (
            left_fit is not None
            and right_fit is not None
            and len(trusted_traced) >= 3
        ):
            # Validate only where both rails were actually paired.  Testing
            # the polynomial outside its observed y-span would turn harmless
            # far-field extrapolation into a false INVALID_WIDTH on bends.
            paired_y_values = [item[0] for item in trusted_traced]
            lookahead_support_distance = min(
                abs(float(y_value) - float(target_y))
                for y_value in paired_y_values
            )
            evaluation_y = np.linspace(
                min(paired_y_values), max(paired_y_values), 25
            )
            evaluated_left = np.polyval(
                left_fit["coefficients"], evaluation_y
            )
            evaluated_right = np.polyval(
                right_fit["coefficients"], evaluation_y
            )
            widths = evaluated_right - evaluated_left
            finite_geometry = bool(
                np.all(np.isfinite(evaluated_left))
                and np.all(np.isfinite(evaluated_right))
            )
            ordered_geometry = finite_geometry and bool(np.all(widths > 0.0))
            plausible_widths = ordered_geometry and bool(
                np.all(widths >= minimum_width)
                and np.all(widths <= maximum_width)
            )
            median_width = float(np.median(widths)) if plausible_widths else 0.0
            if plausible_widths and median_width > 0.0:
                half_window = (
                    height
                    * float(self.config[
                        "lane_width_evaluation_window_fraction"
                    ])
                    / 2.0
                )
                local_y_start = max(
                    float(min(paired_y_values)), target_y - half_window
                )
                local_y_stop = min(
                    float(max(paired_y_values)), target_y + half_window
                )
                local_y = np.linspace(local_y_start, local_y_stop, 15)
                local_widths = (
                    np.polyval(right_fit["coefficients"], local_y)
                    - np.polyval(left_fit["coefficients"], local_y)
                )
                local_median_width = float(np.median(local_widths))
                width_variation = float(
                    (np.max(local_widths) - np.min(local_widths))
                    / max(1.0, local_median_width)
                )
            fit_residual = (
                float(left_fit["mean_residual"])
                + float(right_fit["mean_residual"])
            ) / 2.0
            pair_qualities = [
                float(pair["quality"]) for _y, pair in trusted_traced
            ]
            pair_quality = (
                float(np.mean(pair_qualities)) if pair_qualities else 0.0
            )
            if plausible_widths and expected_width > 1.0:
                width_error_fraction = abs(
                    median_width - expected_width
                ) / expected_width

            standard_geometry = (
                plausible_widths
                and left_fit["inlier_fraction"] >= minimum_inliers
                and right_fit["inlier_fraction"] >= minimum_inliers
                and lookahead_support_distance
                <= float(step + int(self.config["scan_band_height_px"]) // 2)
                and paired_fraction
                >= float(self.config["minimum_paired_row_fraction"])
                and vertical_coverage
                >= float(self.config["minimum_vertical_coverage_fraction"])
                and width_variation
                <= float(self.config["maximum_lane_width_variation_fraction"])
                and run_overflow_fraction
                <= float(self.config["maximum_run_overflow_row_fraction"])
            )
            if standard_geometry:
                left_x = float(np.polyval(left_fit["coefficients"], target_y))
                right_x = float(np.polyval(right_fit["coefficients"], target_y))
                if (
                    math.isfinite(left_x)
                    and math.isfinite(right_x)
                    and 0.0 <= left_x < right_x <= width - 1
                ):
                    lane_width = right_x - left_x
                    lane_centre = (left_x + right_x) / 2.0
                    status = "FULL"
                    support_mode = "STANDARD"
                else:
                    status = "INVALID_WIDTH"
            elif trusted_traced:
                status = "INVALID_WIDTH"

        if run_overflow_fraction > float(
            self.config["maximum_run_overflow_row_fraction"]
        ):
            status = "RUN_OVERFLOW"
            left_x = None
            right_x = None
            lane_width = None
            lane_centre = None

        if status not in ("FULL", "RUN_OVERFLOW"):
            component_curve = self._component_curve_fallback(
                black_mask,
                rows,
                scan_top,
                scan_bottom,
                target_y,
                expected_width,
                minimum_width,
                maximum_width,
            )
            if component_curve is not None:
                status = "FULL"
                support_mode = "COMPONENT_CURVE"
                trusted_traced = component_curve["traced"]
                traced = list(trusted_traced)
                left_fit = component_curve["left_fit"]
                right_fit = component_curve["right_fit"]
                left_x = component_curve["left_x"]
                right_x = component_curve["right_x"]
                lane_width = component_curve["lane_width"]
                lane_centre = component_curve["lane_centre"]
                normal_lane_width = component_curve["normal_lane_width"]
                paired_fraction = component_curve["paired_fraction"]
                vertical_coverage = component_curve["vertical_coverage"]
                fit_residual = component_curve["fit_residual"]
                lookahead_support_distance = component_curve[
                    "lookahead_support_distance"
                ]
                width_variation = component_curve["width_variation"]
                pair_quality = component_curve["pair_quality"]
                width_error_fraction = component_curve[
                    "width_error_fraction"
                ]

        # Remembered geometry may associate a single observed rail, but it can
        # never promote that frame to FULL.  The safety supervisor therefore
        # continues to command STOP for every partial observation.
        if status == "LOST" and self.remembered_lane_width is not None:
            nearest_row = min(rows, key=lambda row: abs(row["y"] - target_y))
            if nearest_row["runs"]:
                predicted_left = (
                    self.remembered_lane_centre
                    - self.remembered_lane_width / 2.0
                )
                predicted_right = (
                    self.remembered_lane_centre
                    + self.remembered_lane_width / 2.0
                )
                associations = []
                for run in nearest_row["runs"]:
                    associations.append(
                        (abs(run["x"] - predicted_left), "LEFT", run)
                    )
                    associations.append(
                        (abs(run["x"] - predicted_right), "RIGHT", run)
                    )
                associations.sort(key=lambda item: item[0])
                _distance, side, run = associations[0]
                lane_width = float(self.remembered_lane_width)
                if side == "LEFT":
                    left_x = float(run["x"])
                    right_x = left_x + lane_width
                    status = "PARTIAL_LEFT"
                else:
                    right_x = float(run["x"])
                    left_x = right_x - lane_width
                    status = "PARTIAL_RIGHT"
                lane_centre = (left_x + right_x) / 2.0
        elif status == "LOST" and invalid_width_seen:
            status = "INVALID_WIDTH"

        left_support = paired_fraction * (
            float(np.mean([
                pair["left"]["quality"] for _y, pair in trusted_traced
            ]))
            if trusted_traced
            else 0.0
        )
        right_support = paired_fraction * (
            float(np.mean([
                pair["right"]["quality"] for _y, pair in trusted_traced
            ]))
            if trusted_traced
            else 0.0
        )
        fit_quality = 0.0
        if fit_residual is not None:
            fit_quality = clamp(
                1.0 - fit_residual / max(1.0, residual_threshold), 0.0, 1.0
            )
        width_quality = 0.0
        if lane_width is not None and expected_width > 1.0:
            width_quality = clamp(
                1.0 - abs(lane_width - expected_width) / (0.30 * expected_width),
                0.0,
                1.0,
            )

        if status == "FULL":
            if support_mode == "COMPONENT_CURVE":
                support_quality = clamp(
                    min(
                        paired_fraction / float(self.config[
                            "close_curve_minimum_paired_row_fraction"
                        ]),
                        vertical_coverage / float(self.config[
                            "close_curve_minimum_vertical_coverage_fraction"
                        ]),
                    ),
                    0.0,
                    1.0,
                )
                width_stability_quality = clamp(
                    1.0
                    - width_variation / float(self.config[
                        "close_curve_maximum_lane_width_variation_fraction"
                    ]),
                    0.0,
                    1.0,
                )
                width_agreement_quality = clamp(
                    1.0
                    - width_error_fraction / float(self.config[
                        "close_curve_maximum_width_error_fraction"
                    ]),
                    0.0,
                    1.0,
                )
                confidence = clamp(
                    0.05
                    + 0.20 * support_quality
                    + 0.20 * fit_quality
                    + 0.20 * pair_quality
                    + 0.20 * width_stability_quality
                    + 0.15 * width_agreement_quality,
                    0.0,
                    1.0,
                )
            else:
                confidence = clamp(
                    0.15
                    + 0.30 * paired_fraction
                    + 0.20 * vertical_coverage
                    + 0.20 * fit_quality
                    + 0.15 * width_quality,
                    0.0,
                    1.0,
                )
            width_to_remember = (
                normal_lane_width
                if normal_lane_width is not None
                else lane_width
            )
            self.remembered_lane_width = (
                width_to_remember
                if self.remembered_lane_width is None
                else 0.8 * self.remembered_lane_width
                + 0.2 * width_to_remember
            )
            self.remembered_lane_centre = lane_centre
        elif status.startswith("PARTIAL"):
            confidence = min(
                0.49,
                0.10 + 0.25 * max(left_support, right_support),
            )
        else:
            confidence = 0.0

        return {
            "status": status,
            "support_mode": support_mode,
            "confidence": float(confidence),
            "left_x": left_x,
            "right_x": right_x,
            "lane_width": lane_width,
            "lane_centre": lane_centre,
            "target_y": target_y,
            "rows": rows,
            "traced": trusted_traced,
            "candidate_traced": traced,
            "left_fit": left_fit,
            "right_fit": right_fit,
            "paired_fraction": float(paired_fraction),
            "vertical_coverage": float(vertical_coverage),
            "fit_residual": fit_residual,
            "lookahead_support_distance": lookahead_support_distance,
            "width_variation": float(width_variation),
            "run_overflow_fraction": float(run_overflow_fraction),
            "maximum_raw_row_runs": max(
                [row["raw_run_count"] for row in rows] or [0]
            ),
            "left_support": float(clamp(left_support, 0.0, 1.0)),
            "right_support": float(clamp(right_support, 0.0, 1.0)),
            "vertical_quality": float(fit_quality),
            "width_quality": float(width_quality),
            "pair_quality": float(pair_quality),
            "width_error_fraction": float(width_error_fraction),
        }

    def observe(self, frame, include_debug=True):
        """Measure the lane and optionally render diagnostic images.

        Physical headless control consumes only the numeric observation.  The
        previous implementation still ran Canny, copied two full frames and
        drew every fitted point on every control iteration.  On the two-core
        Jetson Nano 5 W profile that work competed with YOLO and made otherwise
        valid obstacle results expire.  Diagnostics retain the original
        visuals by using the default ``include_debug=True``.
        """
        warped, source, destination, matrix = self.warp(frame)
        height, width = warped.shape[:2]

        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        prepared = self._prepared_geometry(width, height)
        valid_warp_mask = prepared["valid_warp_mask"]
        analysis_roi_mask = prepared["analysis_roi_mask"]
        analysis_mask = prepared["analysis_mask"]
        gray[analysis_mask == 0] = 255
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _unused_threshold, black_mask = cv2.threshold(
            blurred,
            int(self.config["black_threshold"]),
            255,
            cv2.THRESH_BINARY_INV,
        )

        black_mask = cv2.morphologyEx(
            black_mask, cv2.MORPH_CLOSE, self._close_kernel
        )

        black_mask = cv2.morphologyEx(
            black_mask, cv2.MORPH_OPEN, self._opening_kernel
        )
        black_mask = cv2.bitwise_and(black_mask, analysis_mask)
        edges = cv2.Canny(black_mask, 50, 150) if include_debug else None

        if self.config["detector_mode"] == SINGLE_LINE_DETECTOR_MODE:
            detection = detect_single_line(
                black_mask, destination, self.config
            )
        else:
            detection = self._detect_boundaries(black_mask, destination)
        target_y = detection["target_y"]
        centre_x = width / 2.0
        left_x = detection.get("left_x")
        right_x = detection.get("right_x")
        lane_centre = detection["lane_centre"]
        lane_width = detection["lane_width"]
        status = detection["status"]

        lane_offset = 0.0
        if detection.get("lookahead_offset") is not None:
            lane_offset = clamp(
                float(detection["lookahead_offset"]), -1.0, 1.0
            )
        elif lane_centre is not None:
            lane_offset = clamp(
                (lane_centre - centre_x) / max(1.0, centre_x), -1.0, 1.0
            )

        left_support = detection["left_support"]
        right_support = detection["right_support"]
        vertical_quality = detection["vertical_quality"]
        width_quality = detection["width_quality"]
        confidence = detection["confidence"]

        original_overlay = None
        warped_overlay = None
        if include_debug:
            original_overlay = frame.copy()
            cv2.polylines(
                original_overlay,
                [np.int32(source)],
                True,
                (0, 200, 255),
                2,
            )

            warped_overlay = warped.copy()
            if self.config["detector_mode"] == SINGLE_LINE_DETECTOR_MODE:
                for y_value, x_value in detection.get("centre_points", []):
                    cv2.circle(
                        warped_overlay,
                        (int(round(x_value)), int(y_value)),
                        2,
                        (0, 180, 0),
                        -1,
                    )
                fits_and_colours = (
                    (detection.get("centre_fit"), (0, 255, 0)),
                )
            else:
                for y_value, pair in detection["traced"]:
                    cv2.circle(
                        warped_overlay,
                        (int(round(pair["left"]["x"])), int(y_value)),
                        2,
                        (0, 180, 0),
                        -1,
                    )
                    cv2.circle(
                        warped_overlay,
                        (int(round(pair["right"]["x"])), int(y_value)),
                        2,
                        (0, 180, 0),
                        -1,
                    )
                fits_and_colours = (
                    (detection["left_fit"], (0, 255, 0)),
                    (detection["right_fit"], (0, 255, 0)),
                )
            for fit, colour in fits_and_colours:
                if fit is not None:
                    if self.config["detector_mode"] == SINGLE_LINE_DETECTOR_MODE:
                        fit_top = self.config[
                            "single_line_scan_top_fraction"
                        ]
                        fit_bottom = self.config[
                            "single_line_scan_bottom_fraction"
                        ]
                    else:
                        fit_top = self.config["scan_top_fraction"]
                        fit_bottom = self.config["scan_bottom_fraction"]
                    y_values = range(
                        int(height * float(fit_top)),
                        int(height * float(fit_bottom)),
                        3,
                    )
                    curve = []
                    for y_value in y_values:
                        x_value = float(
                            np.polyval(fit["coefficients"], y_value)
                        )
                        if math.isfinite(x_value) and 0 <= x_value < width:
                            curve.append((int(round(x_value)), int(y_value)))
                    if len(curve) >= 2:
                        cv2.polylines(
                            warped_overlay,
                            [np.int32(curve)],
                            False,
                            colour,
                            2,
                        )
            cv2.line(
                warped_overlay,
                (int(centre_x), height - 1),
                (int(centre_x), 0),
                (255, 0, 0),
                1,
            )
            if left_x is not None:
                cv2.circle(
                    warped_overlay,
                    (int(left_x), target_y),
                    6,
                    (0, 255, 0),
                    -1,
                )
            if right_x is not None:
                cv2.circle(
                    warped_overlay,
                    (int(right_x), target_y),
                    6,
                    (0, 255, 0),
                    -1,
                )
            if lane_centre is not None:
                cv2.circle(
                    warped_overlay,
                    (int(lane_centre), target_y),
                    7,
                    (0, 0, 255),
                    -1,
                )

        return {
            "status": status,
            "lane_offset": float(lane_offset),
            "confidence": float(confidence),
            "lane_width_px": None if lane_width is None else float(lane_width),
            "line_count": int(
                detection.get(
                    "support_point_count",
                    len(detection.get("traced", [])),
                )
            ),
            "left_x": left_x,
            "right_x": right_x,
            "lookahead_y": target_y,
            "original_overlay": original_overlay,
            "warped": warped,
            "warped_overlay": warped_overlay,
            "mask": black_mask,
            "edges": edges,
            "valid_warp_mask": valid_warp_mask,
            "analysis_roi_mask": analysis_mask,
            "matrix": matrix,
            "source_polygon": source.tolist(),
            "destination_polygon": destination.tolist(),
            "left_support": float(left_support),
            "right_support": float(right_support),
            "vertical_quality": float(vertical_quality),
            "width_quality": float(width_quality),
            "detector_mode": self.config["detector_mode"],
            "target_line_role": self.config.get("target_line_role"),
            "support_mode": detection["support_mode"],
            "paired_row_fraction": float(detection["paired_fraction"]),
            "vertical_coverage_fraction": float(
                detection["vertical_coverage"]
            ),
            "fit_residual_px": detection["fit_residual"],
            "lookahead_support_distance_px": detection[
                "lookahead_support_distance"
            ],
            "lane_width_variation_fraction": float(
                detection["width_variation"]
            ),
            "run_overflow_row_fraction": float(
                detection["run_overflow_fraction"]
            ),
            "maximum_raw_row_runs": int(
                detection["maximum_raw_row_runs"]
            ),
            "near_field_offset": (
                float(detection["near_field_offset"])
                if detection.get("near_field_offset") is not None
                else float(lane_offset)
            ),
            "lookahead_offset": (
                float(detection["lookahead_offset"])
                if detection.get("lookahead_offset") is not None
                else float(lane_offset)
            ),
            "centreline_x_px": detection.get("lookahead_x"),
            "near_field_x_px": detection.get("near_x"),
            "heading_error_rad": detection.get("heading_error_rad"),
            "curvature_per_px": detection.get("curvature_per_px"),
            "fit_inlier_fraction": (
                detection.get("centre_fit", {}).get("inlier_fraction")
                if detection.get("centre_fit") is not None
                else None
            ),
            "candidate_count": int(detection.get("candidate_count", 0)),
            "line_like_component_count": int(
                detection.get("line_like_component_count", 0)
            ),
            "ambiguity_margin": float(
                detection.get("ambiguity_margin", 0.0)
            ),
            "support_point_count": int(
                detection.get(
                    "support_point_count",
                    len(detection.get("traced", [])),
                )
            ),
            "tape_width_px": detection.get("tape_width_px"),
            "calibration_state": self.config.get("calibration_state", "UNKNOWN"),
        }
