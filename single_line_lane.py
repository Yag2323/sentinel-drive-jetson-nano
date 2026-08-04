#!/usr/bin/env python3
"""Fail-safe current-frame detector for one black-tape track centreline.

This module is intentionally stateless.  It never carries a line estimate
between frames and it never promotes a missing or partial observation.  The
caller supplies the already-thresholded, IPM-warped black-tape mask.
"""

from __future__ import print_function

import math
import warnings

import cv2
import numpy as np


SINGLE_LINE_DETECTOR_MODE = "SINGLE_LINE_POLYNOMIAL_V1"


SINGLE_LINE_DEFAULTS = {
    "detector_mode": SINGLE_LINE_DETECTOR_MODE,
    "single_line_scan_top_fraction": 0.25,
    "single_line_scan_bottom_fraction": 0.96,
    "single_line_roi_left_fraction": 0.02,
    "single_line_roi_right_fraction": 0.98,
    "single_line_lookahead_y_fraction": 0.62,
    "single_line_near_field_y_fraction": 0.86,
    "single_line_scan_row_step_px": 3,
    "single_line_polynomial_degree": 2,
    "single_line_minimum_component_area_fraction": 0.0008,
    "single_line_minimum_support_row_fraction": 0.42,
    "single_line_minimum_vertical_coverage_fraction": 0.42,
    "single_line_minimum_fit_inlier_fraction": 0.72,
    "single_line_maximum_fit_residual_px": 6.0,
    "single_line_maximum_support_distance_px": 10.0,
    "single_line_minimum_tape_width_px": 2.0,
    # The observed close physical tape has a 57.969 px maximum diameter in
    # the 640 px warped image.  Ten percent gives a 64 px ceiling: enough for
    # that measured perspective expansion while remaining a strict upper
    # bound for rejecting broad dark objects.
    "single_line_maximum_tape_width_fraction": 0.10,
    "single_line_maximum_tape_width_variation_fraction": 0.65,
    "single_line_maximum_raw_row_runs": 2,
    "single_line_maximum_run_overflow_fraction": 0.08,
    "single_line_maximum_near_field_offset": 0.35,
    "single_line_maximum_absolute_offset": 0.92,
    "single_line_maximum_absolute_heading_rad": 1.20,
    "single_line_maximum_absolute_curvature_per_px": 0.02,
    "single_line_minimum_candidate_score": 0.52,
    "single_line_minimum_ambiguity_margin": 0.12,
}


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _number(config, key):
    value = config[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a JSON number.".format(key))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite.".format(key))
    return value


def _integer(config, key):
    value = config[key]
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} must be a JSON integer.".format(key))
    return int(value)


def validate_single_line_config(config):
    """Return a validated copy of *config* with single-line defaults added."""
    if not isinstance(config, dict):
        raise ValueError("Single-line configuration must be a dictionary.")
    validated = dict(config)
    for key, value in SINGLE_LINE_DEFAULTS.items():
        validated.setdefault(key, value)

    if validated.get("detector_mode") != SINGLE_LINE_DETECTOR_MODE:
        raise ValueError(
            "detector_mode must be {}.".format(SINGLE_LINE_DETECTOR_MODE)
        )

    row_step = _integer(validated, "single_line_scan_row_step_px")
    degree = _integer(validated, "single_line_polynomial_degree")
    maximum_runs = _integer(validated, "single_line_maximum_raw_row_runs")
    if not 1 <= row_step <= 20:
        raise ValueError("single_line_scan_row_step_px must be in 1..20.")
    if not 1 <= degree <= 3:
        raise ValueError("single_line_polynomial_degree must be in 1..3.")
    if not 1 <= maximum_runs <= 8:
        raise ValueError("single_line_maximum_raw_row_runs must be in 1..8.")

    top = _number(validated, "single_line_scan_top_fraction")
    bottom = _number(validated, "single_line_scan_bottom_fraction")
    roi_left = _number(validated, "single_line_roi_left_fraction")
    roi_right = _number(validated, "single_line_roi_right_fraction")
    lookahead = _number(validated, "single_line_lookahead_y_fraction")
    near_field = _number(validated, "single_line_near_field_y_fraction")
    if not 0.05 <= top < bottom <= 0.99:
        raise ValueError(
            "single_line scan fractions must satisfy 0.05 <= top < "
            "bottom <= 0.99."
        )
    if not 0.0 <= roi_left < roi_right <= 1.0:
        raise ValueError(
            "single_line ROI fractions must satisfy "
            "0 <= left < right <= 1."
        )
    if roi_right - roi_left < 0.50:
        raise ValueError("single_line ROI must cover at least half the image.")
    if not top <= lookahead < near_field <= bottom:
        raise ValueError(
            "single_line lookahead and near-field fractions must satisfy "
            "scan_top <= lookahead < near_field <= scan_bottom."
        )

    bounded_zero_one = (
        "single_line_minimum_component_area_fraction",
        "single_line_minimum_support_row_fraction",
        "single_line_minimum_vertical_coverage_fraction",
        "single_line_minimum_fit_inlier_fraction",
        "single_line_maximum_tape_width_fraction",
        "single_line_maximum_tape_width_variation_fraction",
        "single_line_maximum_run_overflow_fraction",
        "single_line_maximum_near_field_offset",
        "single_line_maximum_absolute_offset",
        "single_line_minimum_candidate_score",
        "single_line_minimum_ambiguity_margin",
    )
    for key in bounded_zero_one:
        value = _number(validated, key)
        if not 0.0 < value <= 1.0:
            raise ValueError("{} must be in (0, 1].".format(key))

    if not 0.00005 <= validated[
        "single_line_minimum_component_area_fraction"
    ] <= 0.05:
        raise ValueError(
            "single_line_minimum_component_area_fraction must be in "
            "0.00005..0.05."
        )
    if not 0.30 <= validated[
        "single_line_minimum_fit_inlier_fraction"
    ] <= 1.0:
        raise ValueError(
            "single_line_minimum_fit_inlier_fraction must be in 0.30..1.0."
        )
    if not 0.005 <= validated[
        "single_line_maximum_tape_width_fraction"
    ] <= 0.25:
        raise ValueError(
            "single_line_maximum_tape_width_fraction must be in 0.005..0.25."
        )

    minimum_width = _number(validated, "single_line_minimum_tape_width_px")
    residual = _number(validated, "single_line_maximum_fit_residual_px")
    support_distance = _number(
        validated, "single_line_maximum_support_distance_px"
    )
    maximum_heading = _number(
        validated, "single_line_maximum_absolute_heading_rad"
    )
    maximum_curvature = _number(
        validated, "single_line_maximum_absolute_curvature_per_px"
    )
    if not 1.0 <= minimum_width <= 50.0:
        raise ValueError("single_line_minimum_tape_width_px must be in 1..50.")
    if not 0.5 <= residual <= 100.0:
        raise ValueError(
            "single_line_maximum_fit_residual_px must be in 0.5..100."
        )
    if not 1.0 <= support_distance <= 100.0:
        raise ValueError(
            "single_line_maximum_support_distance_px must be in 1..100."
        )
    if not 0.10 <= maximum_heading <= math.pi / 2.0:
        raise ValueError(
            "single_line_maximum_absolute_heading_rad must be in "
            "0.10..pi/2."
        )
    if not 0.00001 <= maximum_curvature <= 0.25:
        raise ValueError(
            "single_line_maximum_absolute_curvature_per_px must be in "
            "0.00001..0.25."
        )
    return validated


def _base_result(height, width, target_y, near_y):
    return {
        "status": "LOST",
        "support_mode": "NONE",
        "confidence": 0.0,
        "lane_centre": None,
        "lane_width": None,
        "target_y": int(target_y),
        "centre_points": [],
        "centre_fit": None,
        "paired_fraction": 0.0,
        "vertical_coverage": 0.0,
        "fit_residual": None,
        "lookahead_support_distance": None,
        "width_variation": 1.0,
        "run_overflow_fraction": 0.0,
        "maximum_raw_row_runs": 0,
        "left_support": 0.0,
        "right_support": 0.0,
        "vertical_quality": 0.0,
        "width_quality": 0.0,
        "pair_quality": 0.0,
        "width_error_fraction": 1.0,
        "near_x": None,
        "lookahead_x": None,
        "near_y": int(near_y),
        "near_field_offset": None,
        "lookahead_offset": None,
        "heading_error_rad": None,
        "curvature_per_px": None,
        "candidate_count": 0,
        "line_like_component_count": 0,
        "ambiguity_margin": 0.0,
        "support_point_count": 0,
        "tape_width_px": None,
        "image_width": int(width),
        "image_height": int(height),
    }


def _row_runs(pixel_x_values):
    if pixel_x_values.size == 0:
        return []
    ordered = np.sort(pixel_x_values.astype(np.int32))
    runs = []
    start = int(ordered[0])
    previous = start
    for raw_x in ordered[1:]:
        x_value = int(raw_x)
        if x_value > previous + 1:
            runs.append((start, previous))
            start = x_value
        previous = x_value
    runs.append((start, previous))
    return runs


def _component_is_line_like_evidence(
    label, labels, stats, scan_top, scan_bottom, width, height, config
):
    """Conservatively flag secondary elongated tape-like components.

    This check is intentionally weaker than candidate qualification.  It
    prevents a strong inner-circle segment being selected merely because the
    intended outer segment is fragmented or has insufficient row support.
    """
    area = int(stats[label, cv2.CC_STAT_AREA])
    minimum_area = max(
        20,
        int(round(
            width
            * height
            * float(config["single_line_minimum_component_area_fraction"])
        )),
    )
    if area < minimum_area:
        return False
    component_top = int(stats[label, cv2.CC_STAT_TOP])
    component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
    component_bottom = component_top + component_height - 1
    overlap_top = max(int(scan_top), component_top)
    overlap_bottom = min(int(scan_bottom), component_bottom)
    overlap_height = max(0, overlap_bottom - overlap_top + 1)
    minimum_height = max(20, int(round(0.20 * (scan_bottom - scan_top))))
    if overlap_height < minimum_height:
        return False
    maximum_width = max(
        float(config["single_line_minimum_tape_width_px"]),
        width * float(config["single_line_maximum_tape_width_fraction"]),
    )
    # Do not reject an otherwise tape-like component because of one local
    # perspective/morphology bulge.  Use the median widths of all horizontal
    # runs in the scan overlap.  This evidence test is deliberately weaker
    # than full candidate qualification so a secondary line still makes the
    # frame AMBIGUOUS.
    run_widths = []
    row_step = int(config["single_line_scan_row_step_px"])
    for y_value in range(overlap_top, overlap_bottom + 1, row_step):
        x_values = np.where(labels[y_value] == label)[0]
        for start, stop in _row_runs(x_values):
            run_widths.append(float(stop - start + 1))
    if len(run_widths) < 4:
        return False
    median_width = float(np.median(np.float64(run_widths)))
    return bool(
        math.isfinite(median_width)
        and float(config["single_line_minimum_tape_width_px"])
        <= median_width
        <= maximum_width
    )


def _polyfit_robust(points, degree, threshold, minimum_inlier_fraction):
    if len(points) < max(degree + 1, 4):
        return None
    y_values = np.float64([point[0] for point in points])
    x_values = np.float64([point[1] for point in points])
    active = np.ones(len(points), dtype=bool)
    rank_warning = getattr(np, "RankWarning", Warning)

    coefficients = None
    for _iteration in range(4):
        if int(np.count_nonzero(active)) < degree + 1:
            return None
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("error", rank_warning)
                coefficients = np.polyfit(
                    y_values[active], x_values[active], degree
                )
        except (ValueError, rank_warning, np.linalg.LinAlgError):
            return None
        if not np.all(np.isfinite(coefficients)):
            return None
        residuals = np.abs(np.polyval(coefficients, y_values) - x_values)
        updated = residuals <= float(threshold)
        if np.array_equal(updated, active):
            break
        active = updated

    if coefficients is None:
        return None
    inlier_count = int(np.count_nonzero(active))
    inlier_fraction = float(inlier_count) / float(len(points))
    if (
        inlier_count < max(degree + 1, 4)
        or inlier_fraction < float(minimum_inlier_fraction)
    ):
        return None

    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", rank_warning)
            coefficients = np.polyfit(
                y_values[active], x_values[active], degree
            )
    except (ValueError, rank_warning, np.linalg.LinAlgError):
        return None
    if not np.all(np.isfinite(coefficients)):
        return None
    final_residuals = np.abs(
        np.polyval(coefficients, y_values[active]) - x_values[active]
    )
    return {
        "coefficients": coefficients,
        "mean_residual": float(np.mean(final_residuals)),
        "maximum_residual": float(np.max(final_residuals)),
        "inlier_fraction": inlier_fraction,
        "inlier_count": inlier_count,
        "inlier_indices": [
            int(index)
            for index, accepted in enumerate(active.tolist())
            if accepted
        ],
    }


def _component_candidate(
    label,
    labels,
    stats,
    scan_y_values,
    scan_top,
    scan_bottom,
    target_y,
    near_y,
    width,
    height,
    config,
):
    area = int(stats[label, cv2.CC_STAT_AREA])
    minimum_area = max(
        20,
        int(round(
            width
            * height
            * float(config["single_line_minimum_component_area_fraction"])
        )),
    )
    if area < minimum_area:
        return None

    # Distance-transform only the component's bounding box plus one in-frame
    # background pixel on each available side.  The former full-frame mask
    # allocated and transformed width*height pixels once per component even
    # though every pixel outside this small region was zero.  A one-pixel
    # zero ring is sufficient because the connected-component bounding box is
    # tight; clipping (rather than inventing padding) at an image edge keeps
    # OpenCV's original image-boundary semantics unchanged.
    component_left = int(stats[label, cv2.CC_STAT_LEFT])
    component_top = int(stats[label, cv2.CC_STAT_TOP])
    component_width = int(stats[label, cv2.CC_STAT_WIDTH])
    component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
    roi_left = max(0, component_left - 1)
    roi_top = max(0, component_top - 1)
    roi_right = min(width, component_left + component_width + 1)
    roi_bottom = min(height, component_top + component_height + 1)
    component = np.uint8(
        labels[roi_top:roi_bottom, roi_left:roi_right] == label
    )
    distance = cv2.distanceTransform(component, cv2.DIST_L2, 5)
    maximum_diameter = 2.0 * float(np.max(distance))
    maximum_width = max(
        float(config["single_line_minimum_tape_width_px"]),
        width * float(config["single_line_maximum_tape_width_fraction"]),
    )
    if (
        not math.isfinite(maximum_diameter)
        or maximum_diameter < float(config["single_line_minimum_tape_width_px"])
    ):
        return None

    points = []
    local_widths = []
    overflow_rows = 0
    maximum_raw_runs = 0
    maximum_allowed_runs = int(config["single_line_maximum_raw_row_runs"])
    for y_value in scan_y_values:
        if y_value < roi_top or y_value >= roi_bottom:
            continue
        local_y = int(y_value - roi_top)
        x_values = np.where(component[local_y] > 0)[0] + roi_left
        runs = _row_runs(x_values)
        maximum_raw_runs = max(maximum_raw_runs, len(runs))
        if len(runs) > maximum_allowed_runs:
            overflow_rows += 1
            continue
        if len(runs) != 1:
            continue
        start, stop = runs[0]
        centre_x = (float(start) + float(stop)) / 2.0
        rounded_x = int(round(_clamp(centre_x, 0.0, float(width - 1))))
        local_y = int(y_value - roi_top)
        local_x = int(rounded_x - roi_left)
        local_diameter = 2.0 * float(distance[local_y, local_x])
        if not math.isfinite(local_diameter) or local_diameter <= 0.0:
            continue
        points.append((int(y_value), centre_x))
        local_widths.append(local_diameter)

    total_rows = max(1, len(scan_y_values))
    support_fraction = float(len(points)) / float(total_rows)
    overflow_fraction = float(overflow_rows) / float(total_rows)
    if overflow_fraction > float(
        config["single_line_maximum_run_overflow_fraction"]
    ):
        return None
    if support_fraction < float(
        config["single_line_minimum_support_row_fraction"]
    ):
        return None
    if len(points) < 4:
        return None

    y_values = [point[0] for point in points]
    scan_span = max(1.0, float(scan_bottom - scan_top))
    vertical_coverage = float(max(y_values) - min(y_values)) / scan_span
    if vertical_coverage < float(
        config["single_line_minimum_vertical_coverage_fraction"]
    ):
        return None

    fit = _polyfit_robust(
        points,
        int(config["single_line_polynomial_degree"]),
        float(config["single_line_maximum_fit_residual_px"]),
        float(config["single_line_minimum_fit_inlier_fraction"]),
    )
    if fit is None:
        return None
    trusted_points = [points[index] for index in fit["inlier_indices"]]
    trusted_widths = np.float64(
        [local_widths[index] for index in fit["inlier_indices"]]
    )
    trusted_y = [point[0] for point in trusted_points]
    support_limit = float(config["single_line_maximum_support_distance_px"])
    lookahead_distance = min(
        abs(float(y_value) - float(target_y)) for y_value in trusted_y
    )
    near_distance = min(
        abs(float(y_value) - float(near_y)) for y_value in trusted_y
    )
    if lookahead_distance > support_limit or near_distance > support_limit:
        return None

    tape_width = float(np.median(trusted_widths))
    p10, p90 = np.percentile(trusted_widths, [10.0, 90.0])
    width_variation = float(p90 - p10) / max(1.0, tape_width)
    if width_variation > float(
        config["single_line_maximum_tape_width_variation_fraction"]
    ):
        return None
    if not (
        float(config["single_line_minimum_tape_width_px"])
        <= tape_width
        <= maximum_width
    ):
        return None

    coefficients = fit["coefficients"]
    near_x = float(np.polyval(coefficients, near_y))
    lookahead_x = float(np.polyval(coefficients, target_y))
    centre_x = width / 2.0
    if not (
        math.isfinite(near_x)
        and math.isfinite(lookahead_x)
        and 0.0 <= near_x <= width - 1
        and 0.0 <= lookahead_x <= width - 1
    ):
        return None
    near_offset = (near_x - centre_x) / max(1.0, centre_x)
    lookahead_offset = (lookahead_x - centre_x) / max(1.0, centre_x)
    maximum_near_offset = float(
        config["single_line_maximum_near_field_offset"]
    )
    maximum_offset = float(config["single_line_maximum_absolute_offset"])
    if (
        abs(near_offset) > maximum_near_offset
        or abs(lookahead_offset) > maximum_offset
    ):
        return None

    first_derivative = np.polyder(coefficients, 1)
    second_derivative = np.polyder(coefficients, 2)
    dx_dy = float(np.polyval(first_derivative, near_y))
    d2x_dy2 = (
        float(np.polyval(second_derivative, near_y))
        if len(second_derivative) > 0
        else 0.0
    )
    heading = math.atan2(-dx_dy, 1.0)
    curvature = d2x_dy2 / math.pow(1.0 + dx_dy * dx_dy, 1.5)
    if not (math.isfinite(heading) and math.isfinite(curvature)):
        return None
    if (
        abs(heading) > float(
            config["single_line_maximum_absolute_heading_rad"]
        )
        or abs(curvature) > float(
            config["single_line_maximum_absolute_curvature_per_px"]
        )
    ):
        return None

    support_quality = _clamp(
        support_fraction
        / float(config["single_line_minimum_support_row_fraction"]),
        0.0,
        1.0,
    )
    vertical_quality = _clamp(
        vertical_coverage
        / float(config["single_line_minimum_vertical_coverage_fraction"]),
        0.0,
        1.0,
    )
    fit_quality = _clamp(
        1.0
        - float(fit["mean_residual"])
        / float(config["single_line_maximum_fit_residual_px"]),
        0.0,
        1.0,
    )
    width_quality = _clamp(
        1.0
        - width_variation
        / float(config[
            "single_line_maximum_tape_width_variation_fraction"
        ]),
        0.0,
        1.0,
    )
    centre_quality = _clamp(
        1.0 - 0.5 * (abs(near_offset) + abs(lookahead_offset)),
        0.0,
        1.0,
    )
    score = (
        0.25 * support_quality
        + 0.20 * vertical_quality
        + 0.25 * fit_quality
        + 0.15 * width_quality
        + 0.15 * centre_quality
    )
    return {
        "score": float(score),
        "fit": fit,
        "points": trusted_points,
        "support_fraction": support_fraction,
        "vertical_coverage": vertical_coverage,
        "fit_residual": float(fit["mean_residual"]),
        "lookahead_distance": float(lookahead_distance),
        "near_distance": float(near_distance),
        "width_variation": width_variation,
        "overflow_fraction": overflow_fraction,
        "maximum_raw_runs": maximum_raw_runs,
        "tape_width": tape_width,
        "near_x": near_x,
        "lookahead_x": lookahead_x,
        "near_offset": near_offset,
        "lookahead_offset": lookahead_offset,
        "heading": heading,
        "curvature": curvature,
        "vertical_quality": vertical_quality,
        "width_quality": width_quality,
        "fit_quality": fit_quality,
    }


def detect_single_line(black_mask, destination, config):
    """Detect one unambiguous black-tape centreline in the current mask.

    A successful result has ``status == 'FULL'``.  All other statuses are
    stop conditions; no missing geometry is inferred from prior frames.
    """
    validated = validate_single_line_config(config)
    if black_mask is None or not isinstance(black_mask, np.ndarray):
        raise ValueError("black_mask must be a NumPy array.")
    if black_mask.ndim != 2 or black_mask.size == 0:
        raise ValueError("black_mask must be a non-empty single-channel mask.")
    height, width = black_mask.shape[:2]
    if height < 16 or width < 16:
        raise ValueError("black_mask is too small for line detection.")

    destination_array = np.asarray(destination, dtype=np.float64)
    if destination_array.shape != (4, 2):
        raise ValueError("destination must contain four two-dimensional points.")
    if not np.all(np.isfinite(destination_array)):
        raise ValueError("destination points must be finite.")

    scan_top = int(round(
        height * float(validated["single_line_scan_top_fraction"])
    ))
    scan_bottom = int(round(
        height * float(validated["single_line_scan_bottom_fraction"])
    ))
    scan_top = max(0, min(height - 2, scan_top))
    scan_bottom = max(scan_top + 1, min(height - 1, scan_bottom))
    target_y = int(round(
        height * float(validated["single_line_lookahead_y_fraction"])
    ))
    near_y = int(round(
        height * float(validated["single_line_near_field_y_fraction"])
    ))
    result = _base_result(height, width, target_y, near_y)

    binary = np.uint8(black_mask > 0)
    component_count, labels, stats, _centroids = (
        cv2.connectedComponentsWithStats(binary, 8)
    )
    scan_y_values = list(range(
        scan_bottom,
        scan_top - 1,
        -int(validated["single_line_scan_row_step_px"]),
    ))
    candidates = []
    line_like_component_count = 0
    for label in range(1, component_count):
        if _component_is_line_like_evidence(
            label,
            labels,
            stats,
            scan_top,
            scan_bottom,
            width,
            height,
            validated,
        ):
            line_like_component_count += 1
        candidate = _component_candidate(
            label,
            labels,
            stats,
            scan_y_values,
            scan_top,
            scan_bottom,
            target_y,
            near_y,
            width,
            height,
            validated,
        )
        if candidate is not None and candidate["score"] >= float(
            validated["single_line_minimum_candidate_score"]
        ):
            candidates.append(candidate)

    candidates.sort(key=lambda item: item["score"], reverse=True)
    result["candidate_count"] = len(candidates)
    result["line_like_component_count"] = line_like_component_count
    if not candidates:
        return result

    if len(candidates) == 1:
        ambiguity_margin = 1.0
    else:
        ambiguity_margin = float(
            candidates[0]["score"] - candidates[1]["score"]
        )
    result["ambiguity_margin"] = ambiguity_margin
    # The target is identified by physical near-field anchoring, not by a
    # remembered component ID.  If two components still satisfy every anchor,
    # width, support and geometry guard, their identity is genuinely
    # ambiguous.  Never choose the slightly higher-scoring line: on the
    # concentric physical track that could silently switch from the outer
    # circle to the inner circle.
    if len(candidates) > 1 or line_like_component_count > 1:
        result["status"] = "AMBIGUOUS"
        return result

    selected = candidates[0]
    ambiguity_quality = (
        1.0
        if len(candidates) == 1
        else _clamp(
            ambiguity_margin
            / float(validated["single_line_minimum_ambiguity_margin"]),
            0.0,
            1.0,
        )
    )
    confidence = _clamp(
        0.10
        + 0.35 * selected["score"]
        + 0.20 * selected["fit"]["inlier_fraction"]
        + 0.15 * selected["vertical_quality"]
        + 0.10 * selected["width_quality"]
        + 0.10 * ambiguity_quality,
        0.0,
        1.0,
    )
    coefficients = [float(value) for value in selected["fit"]["coefficients"]]
    centre_fit = dict(selected["fit"])
    centre_fit["coefficients"] = coefficients

    result.update({
        "status": "FULL",
        "support_mode": SINGLE_LINE_DETECTOR_MODE,
        "confidence": float(confidence),
        "lane_centre": float(selected["lookahead_x"]),
        "lane_width": None,
        "centre_points": [
            (int(point[0]), float(point[1])) for point in selected["points"]
        ],
        "centre_fit": centre_fit,
        "paired_fraction": float(selected["support_fraction"]),
        "vertical_coverage": float(selected["vertical_coverage"]),
        "fit_residual": float(selected["fit_residual"]),
        "lookahead_support_distance": float(
            selected["lookahead_distance"]
        ),
        "width_variation": float(selected["width_variation"]),
        "run_overflow_fraction": float(selected["overflow_fraction"]),
        "maximum_raw_row_runs": int(selected["maximum_raw_runs"]),
        "left_support": float(selected["support_fraction"]),
        "right_support": float(selected["support_fraction"]),
        "vertical_quality": float(selected["vertical_quality"]),
        "width_quality": float(selected["width_quality"]),
        "pair_quality": float(ambiguity_quality),
        "width_error_fraction": float(selected["width_variation"]),
        "near_x": float(selected["near_x"]),
        "lookahead_x": float(selected["lookahead_x"]),
        "near_field_offset": float(selected["near_offset"]),
        "lookahead_offset": float(selected["lookahead_offset"]),
        "heading_error_rad": float(selected["heading"]),
        "curvature_per_px": float(selected["curvature"]),
        "support_point_count": len(selected["points"]),
        "tape_width_px": float(selected["tape_width"]),
    })
    return result
