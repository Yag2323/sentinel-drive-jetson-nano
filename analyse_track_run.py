#!/usr/bin/env python3
"""Convert an immutable track CSV into report-ready engineering statistics."""

from __future__ import print_function

import argparse
import collections
import csv
import hashlib
import json
import math
import os
import statistics
import sys


def reject_non_finite_json(value):
    raise ValueError("non-finite JSON number is not permitted: {}".format(value))


def parse_finite_json_float(value):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("JSON number overflows finite range: {}".format(value))
    return parsed


def finite_summary_number(summary, key):
    value = summary.get(key)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a JSON number".format(key))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(key))
    return value


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        while True:
            block = input_file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def numeric(rows, key):
    values = []
    for row in rows:
        value = row.get(key, "").strip()
        if not value:
            continue
        try:
            parsed = float(value)
        except ValueError:
            continue
        if math.isfinite(parsed):
            values.append(parsed)
    return values


def numeric_where(rows, key, predicate):
    """Return finite values only from rows accepted by ``predicate``."""
    return numeric([row for row in rows if predicate(row)], key)


def downward_crossings(values, threshold):
    return sum(
        1
        for previous, current in zip(values, values[1:])
        if previous >= threshold and current < threshold
    )


def main():
    parser = argparse.ArgumentParser(description="Analyse a physical track run")
    parser.add_argument("summary_json")
    parser.add_argument("--distance-m", type=float)
    parser.add_argument("--final-error-cm", type=float)
    parser.add_argument("--notes", default="")
    arguments = parser.parse_args()
    summary_path = os.path.abspath(arguments.summary_json)
    if not os.path.isfile(summary_path):
        print("ERROR: summary does not exist: {}".format(summary_path))
        return 1
    try:
        with open(summary_path, "r") as input_file:
            source_summary = json.load(
                input_file,
                parse_constant=reject_non_finite_json,
                parse_float=parse_finite_json_float,
            )
    except (ValueError, OSError) as error:
        print("ERROR: source summary is not strict JSON: {}".format(error))
        return 1
    if not isinstance(source_summary, dict):
        print("ERROR: source summary must be a JSON object.")
        return 1
    if source_summary.get("test") != "PHYSICAL_TRACK_RUN":
        print("ERROR: expected a PHYSICAL_TRACK_RUN summary.")
        return 1
    if source_summary.get("run_successful") is not True:
        print("ERROR: source track run was not successful.")
        return 1
    if arguments.distance_m is not None and (
        not math.isfinite(arguments.distance_m)
        or arguments.distance_m <= 0.0
    ):
        print("ERROR: --distance-m must be positive when supplied.")
        return 1
    if arguments.final_error_cm is not None and not math.isfinite(
        arguments.final_error_cm
    ):
        print("ERROR: --final-error-cm must be finite when supplied.")
        return 1
    csv_path = source_summary.get("csv")
    if not csv_path or not os.path.isfile(csv_path):
        print("ERROR: linked CSV does not exist: {}".format(csv_path))
        return 1
    with open(csv_path, "r", newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    if not rows:
        print("ERROR: track CSV is empty.")
        return 1
    expected_csv_hash = source_summary.get("csv_sha256")
    actual_csv_hash = sha256_file(csv_path)
    if (
        not isinstance(expected_csv_hash, str)
        or len(expected_csv_hash) != 64
        or expected_csv_hash.lower() != actual_csv_hash
    ):
        print("ERROR: linked CSV does not match the track summary hash.")
        return 1
    run_id = source_summary.get("run_id")
    if not isinstance(run_id, str) or not run_id:
        print("ERROR: source summary has no valid run_id.")
        return 1
    if any(row.get("run_id") != run_id for row in rows):
        print("ERROR: CSV contains a row from a different run_id.")
        return 1
    expected_frames = source_summary.get("frames")
    if (
        isinstance(expected_frames, bool)
        or not isinstance(expected_frames, int)
        or expected_frames != len(rows)
    ):
        print("ERROR: CSV row count does not match the source summary.")
        return 1
    frame_ids = numeric(rows, "frame_id")
    if frame_ids != [float(index) for index in range(1, len(rows) + 1)]:
        print("ERROR: CSV frame_id sequence is incomplete or invalid.")
        return 1

    times = numeric(rows, "monotonic_s")
    voltages = numeric(rows, "battery_voltage_v")
    offsets = numeric_where(
        rows,
        "lane_offset",
        lambda row: row.get("lane_status", "") == "FULL"
        or row.get("lane_status", "").startswith("PARTIAL"),
    )
    fps_values = numeric(rows, "control_fps")
    seen_yolo_frames = set()
    inference_rows = []
    for row in rows:
        source_frame = row.get("yolo_source_frame", "").strip()
        if (
            not source_frame
            or not row.get("yolo_age_s", "").strip()
            or row.get("yolo_fresh", "") != "1"
            or source_frame in seen_yolo_frames
        ):
            continue
        seen_yolo_frames.add(source_frame)
        inference_rows.append(row)
    inference_values = numeric(inference_rows, "yolo_inference_ms")
    if source_summary.get("unique_yolo_results") != len(inference_values):
        print("ERROR: unique YOLO count does not match the source summary.")
        return 1
    left_duties = numeric(rows, "left_mapped_duty")
    right_duties = numeric(rows, "right_mapped_duty")
    lane_counts = collections.Counter(row.get("lane_status", "") for row in rows)
    safety_counts = collections.Counter(row.get("safety_state", "") for row in rows)
    reason_counts = collections.Counter(row.get("safety_reason", "") for row in rows)
    motion_rows = sum(
        1 for row in rows if row.get("physical_motor_commands", "") == "1"
    )
    if source_summary.get("motion_command_frames") != motion_rows:
        print("ERROR: motion-row count does not match the source summary.")
        return 1
    if source_summary.get("safety_state_counts") != dict(safety_counts):
        print("ERROR: safety-state counts do not match the source summary.")
        return 1
    if source_summary.get("safety_reason_counts") != dict(reason_counts):
        print("ERROR: safety-reason counts do not match the source summary.")
        return 1
    usable = sum(
        count
        for status, count in lane_counts.items()
        if status == "FULL" or status.startswith("PARTIAL")
    )
    duration = times[-1] - times[0] if len(times) >= 2 else 0.0
    try:
        starting_voltage = finite_summary_number(
            source_summary, "starting_voltage_v"
        )
        minimum_voltage = finite_summary_number(
            source_summary, "minimum_voltage_v"
        )
        ending_voltage = finite_summary_number(
            source_summary, "ending_voltage_v"
        )
        largest_voltage_sag = finite_summary_number(
            source_summary, "largest_voltage_sag_v"
        )
    except ValueError as error:
        print("ERROR: source summary metric is invalid: {}".format(error))
        return 1
    voltage_sequence = list(voltages)
    if isinstance(starting_voltage, (int, float)):
        voltage_sequence.insert(0, float(starting_voltage))
    source_result_valid = source_summary.get("run_valid_for_results") is True
    manual_measurements_complete = (
        arguments.distance_m is not None
        and arguments.final_error_cm is not None
    )
    report_eligible = source_result_valid and manual_measurements_complete
    analysis = {
        "schema_version": 1,
        "analysis": "PHYSICAL_TRACK_RUN_RESULTS",
        "run_id": run_id,
        "run_purpose": source_summary.get("run_purpose"),
        "source_run_valid_for_results": source_result_valid,
        "report_eligible": report_eligible,
        "evidence_classification": (
            "REPORT_RESULT" if report_eligible else "TUNING_OR_INCOMPLETE"
        ),
        "source_summary": summary_path,
        "source_summary_sha256": sha256_file(summary_path),
        "source_csv": csv_path,
        "source_csv_sha256": actual_csv_hash,
        "frames": len(rows),
        "duration_s": duration,
        "average_control_fps": statistics.mean(fps_values) if fps_values else None,
        "lane_status_counts": dict(lane_counts),
        "lane_usable_rate_pct": 100.0 * usable / len(rows),
        "mean_absolute_lane_offset": statistics.mean(abs(value) for value in offsets) if offsets else None,
        "maximum_absolute_lane_offset": max(abs(value) for value in offsets) if offsets else None,
        "safety_state_counts": dict(safety_counts),
        "safety_reason_counts": dict(reason_counts),
        "mean_yolo_inference_ms": statistics.mean(inference_values) if inference_values else None,
        "unique_yolo_results": len(inference_values),
        "starting_voltage_v": starting_voltage,
        "minimum_voltage_v": minimum_voltage,
        "maximum_voltage_v": max(voltage_sequence) if voltage_sequence else None,
        "average_voltage_v": statistics.mean(voltages) if voltages else None,
        "ending_voltage_v": ending_voltage,
        "largest_observed_voltage_sag_v": largest_voltage_sag,
        "downward_crossings_7_000_v": downward_crossings(
            voltage_sequence, 7.0
        ),
        "downward_crossings_6_600_v": downward_crossings(
            voltage_sequence, 6.6
        ),
        "mean_left_duty": statistics.mean(left_duties) if left_duties else None,
        "mean_right_duty": statistics.mean(right_duties) if right_duties else None,
        "measured_distance_m": arguments.distance_m,
        "final_lateral_error_cm": arguments.final_error_cm,
        "meets_5_cm_straightness_target": (
            abs(arguments.final_error_cm) <= 5.0
            if arguments.final_error_cm is not None
            else None
        ),
        "operator_notes": arguments.notes,
        "measurement_warning": (
            "Distance and lateral error are manual measurements unless an "
            "independent calibrated tracking system is documented."
        ),
    }
    output_base = os.path.splitext(summary_path)[0] + ".analysis"
    json_path = output_base + ".json"
    markdown_path = output_base + ".md"
    with open(json_path, "w") as output_file:
        json.dump(
            analysis,
            output_file,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        output_file.write("\n")
    lines = [
        "# Track run analysis: {}".format(analysis["run_id"]),
        "",
        "- Evidence classification: {}".format(
            analysis["evidence_classification"]
        ),
        "- Report eligible: {}".format(analysis["report_eligible"]),
        "- Frames: {}".format(analysis["frames"]),
        "- Duration: {:.3f} s".format(analysis["duration_s"]),
        "- Mean control rate: {} FPS".format(
            "n/a" if analysis["average_control_fps"] is None else "{:.2f}".format(analysis["average_control_fps"])
        ),
        "- Usable-lane rate: {:.2f}%".format(analysis["lane_usable_rate_pct"]),
        "- Start/min/end voltage: {} / {} / {} V".format(
            analysis["starting_voltage_v"], analysis["minimum_voltage_v"], analysis["ending_voltage_v"]
        ),
        "- Largest observed start-to-minimum sag: {} V".format(analysis["largest_observed_voltage_sag_v"]),
        "- Downward threshold crossings: 7.000 V = {}; 6.600 V = {}".format(
            analysis["downward_crossings_7_000_v"], analysis["downward_crossings_6_600_v"]
        ),
        "- Measured distance: {} m".format(analysis["measured_distance_m"]),
        "- Final lateral error: {} cm".format(analysis["final_lateral_error_cm"]),
        "- Safety states: {}".format(analysis["safety_state_counts"]),
        "- Notes: {}".format(analysis["operator_notes"] or "None recorded"),
        "",
        "Raw evidence hashes are stored in the adjacent JSON analysis file.",
    ]
    with open(markdown_path, "w") as output_file:
        output_file.write("\n".join(lines) + "\n")
    print(json.dumps(analysis, indent=2, sort_keys=True))
    print("Analysis JSON: {}".format(json_path))
    print("Report summary: {}".format(markdown_path))
    if not report_eligible:
        print(
            "NOTE: classified as tuning/incomplete; do not present it as a "
            "final track result."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
