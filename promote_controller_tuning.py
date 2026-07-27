#!/usr/bin/env python3
"""Promote controller gains only from an intact guarded tuning ladder.

This utility never imports or commands motor hardware.  It validates the raw
0.5 s, 1.0 s and 2.0 s tuning summaries and their linked CSV files, records an
operator observation, backs up ``controller_config.json``, and atomically
changes only its verification classification/provenance.  Validation gates
that bind this configuration must then be re-run; this script does not edit or
silently refresh gate evidence.

Compatible with Python 3.6.
"""

from __future__ import print_function

import argparse
import collections
import csv
import datetime
import hashlib
import io
import json
import math
import os
import shutil
import sys

from control_core import load_controller_config, validate_controller_config
from motor_mapping import load_motor_config
from validation_manager import GATES_PATH, REQUIRED_MOTION_GATES, read_json


PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
CONTROLLER_CONFIG_PATH = os.path.join(
    PROJECT_DIRECTORY, "controller_config.json"
)
PROVISIONAL_STATE = "SOFTWARE_INITIAL_VALUES_NOT_PHYSICALLY_TUNED"
TUNED_STATE = "PHYSICALLY_TUNED"
CONFIRMATION_TOKEN = "ACCEPT-PHYSICALLY-TUNED"
MINIMUM_FINAL_DRIVE_ROWS = 5
MINIMUM_FINAL_MOTION_ROWS = 5
MINIMUM_FINAL_UNIQUE_YOLO_RESULTS = 2
EXPECTED_DURATIONS = (0.5, 1.0, 2.0)
FLOAT_TOLERANCE = 0.000001
CONTROLLER_PARAMETER_KEYS = frozenset((
    "kp",
    "ki",
    "kd",
    "steering_limit",
    "lane_confidence_minimum",
    "camera_stale_timeout_s",
    "yolo_image_size",
    "yolo_confidence_threshold",
    "yolo_iou_threshold",
    "yolo_stale_timeout_s",
    "yolo_max_frame_lag",
    "command_lease_s",
    "motor_voltage_stop_v",
    "maximum_motor_voltage_v",
))
MOTOR_MAPPING_PARAMETER_KEYS = frozenset((
    "left_deadband_duty",
    "right_deadband_duty",
    "left_cruise_duty",
    "right_cruise_duty",
    "left_max_duty",
    "right_max_duty",
))


def reject_non_finite_json(value):
    raise ValueError("non-finite JSON number is not permitted: {}".format(value))


def parse_finite_json_float(value):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("JSON number overflows finite range: {}".format(value))
    return parsed


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def read_file_bytes(path):
    with open(path, "rb") as input_file:
        return input_file.read()


def parse_strict_json_bytes(value, path):
    try:
        text = value.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError("JSON is not UTF-8: {} ({})".format(path, error))
    value = json.loads(
        text,
        parse_constant=reject_non_finite_json,
        parse_float=parse_finite_json_float,
    )
    if not isinstance(value, dict):
        raise ValueError("JSON root must be an object: {}".format(path))
    return value


def read_strict_json(path):
    return parse_strict_json_bytes(read_file_bytes(path), path)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        while True:
            block = input_file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def require_boolean(mapping, key, expected):
    value = mapping.get(key)
    if value is not expected:
        raise ValueError("{} must be {!r}; found {!r}".format(
            key, expected, value
        ))


def require_integer(mapping, key):
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} must be a JSON integer".format(key))
    return value


def require_number(mapping, key):
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a JSON number".format(key))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(key))
    return value


def row_truth(value):
    return str(value).strip() == "1"


def validate_controller_identity(summary, current_config):
    recorded = summary.get("controller_parameters")
    if not isinstance(recorded, dict) or set(recorded) != CONTROLLER_PARAMETER_KEYS:
        raise ValueError("controller_parameters is missing or invalid")
    for key, recorded_value in recorded.items():
        if key not in current_config:
            raise ValueError("recorded controller key is no longer present: {}".format(
                key
            ))
        current_value = current_config[key]
        if isinstance(recorded_value, bool) or not isinstance(
            recorded_value, (int, float)
        ):
            raise ValueError("recorded controller parameter is not numeric: {}".format(
                key
            ))
        if abs(float(recorded_value) - float(current_value)) > FLOAT_TOLERANCE:
            raise ValueError(
                "controller parameter changed since {}: {} recorded={!r} current={!r}".format(
                    summary.get("run_id"), key, recorded_value, current_value
                )
            )


def validate_motor_mapping_identity(summary, current_motor_config):
    recorded = summary.get("motor_mapping_parameters")
    if (
        not isinstance(recorded, dict)
        or set(recorded) != MOTOR_MAPPING_PARAMETER_KEYS
    ):
        raise ValueError("motor_mapping_parameters is missing or invalid")
    for key, recorded_value in recorded.items():
        if key not in current_motor_config:
            raise ValueError("recorded motor-mapping key is absent: {}".format(key))
        current_value = current_motor_config[key]
        if isinstance(recorded_value, bool) or not isinstance(
            recorded_value, (int, float)
        ):
            raise ValueError("recorded motor-mapping value is not numeric: {}".format(
                key
            ))
        if abs(float(recorded_value) - float(current_value)) > FLOAT_TOLERANCE:
            raise ValueError(
                "motor mapping changed since {}: {} recorded={!r} current={!r}".format(
                    summary.get("run_id"), key, recorded_value, current_value
                )
            )


def validate_gate_snapshot(
    summary, current_gate_state, current_controller_sha256
):
    snapshot = summary.get("validation_gate_snapshot")
    if not isinstance(snapshot, dict):
        raise ValueError("validation_gate_snapshot is missing")
    if snapshot.get("physical_motion_authorized") is not True:
        raise ValueError("run did not start from an authorized gate snapshot")
    if snapshot.get("authorization_integrity_status") != "CURRENT":
        raise ValueError("run authorization-integrity snapshot was not CURRENT")
    gates = snapshot.get("gates")
    if not isinstance(gates, dict) or not gates:
        raise ValueError("run gate snapshot has no gate records")
    missing = [name for name in REQUIRED_MOTION_GATES if name not in gates]
    if missing:
        raise ValueError("run gate snapshot is incomplete: {}".format(
            ", ".join(missing)
        ))
    not_current = [
        name
        for name, value in gates.items()
        if not isinstance(value, dict)
        or value.get("passed") is not True
        or value.get("integrity_status") != "CURRENT"
    ]
    if not_current:
        raise ValueError(
            "run gate snapshot was not fully current: {}".format(
                ", ".join(sorted(not_current))
            )
        )
    current_gates = current_gate_state.get("gates", {})
    for gate_name in REQUIRED_MOTION_GATES:
        recorded_gate = gates[gate_name]
        current_gate = current_gates.get(gate_name, {})
        if (
            recorded_gate.get("artifact_sha256")
            != current_gate.get("artifact_sha256")
            or recorded_gate.get("evidence_sha256")
            != current_gate.get("evidence_sha256")
        ):
            raise ValueError(
                "{} evidence/artifact identity differs from the current "
                "validated build".format(gate_name)
            )
    if (
        snapshot.get("authorization_artifact_sha256")
        != current_gate_state.get("authorization_artifact_sha256")
    ):
        raise ValueError(
            "run authorization artifact identity differs from the current build"
        )
    motor_gate = gates.get("motor_mapping_software", {})
    motor_artifacts = motor_gate.get("artifact_sha256", {})
    if not isinstance(motor_artifacts, dict) or motor_artifacts.get(
        "controller_config.json"
    ) != current_controller_sha256:
        raise ValueError(
            "current controller_config.json does not match the run's "
            "hash-bound gate snapshot"
        )


def gate_state_identity(state):
    gates = state.get("gates", {}) if isinstance(state, dict) else {}
    return {
        "authorization_artifact_sha256": state.get(
            "authorization_artifact_sha256"
        ) if isinstance(state, dict) else None,
        "gates": {
            gate_name: {
                "artifact_sha256": gates.get(gate_name, {}).get(
                    "artifact_sha256"
                ),
                "evidence_sha256": gates.get(gate_name, {}).get(
                    "evidence_sha256"
                ),
            }
            for gate_name in REQUIRED_MOTION_GATES
        },
    }


def validate_summary_and_csv(
    summary_path,
    current_config,
    current_motor_config,
    current_gate_state,
    current_controller_sha256,
):
    summary_path = os.path.realpath(os.path.abspath(summary_path))
    if not os.path.isfile(summary_path):
        raise ValueError("summary does not exist: {}".format(summary_path))
    summary_bytes = read_file_bytes(summary_path)
    summary = parse_strict_json_bytes(summary_bytes, summary_path)
    if summary.get("test") != "PHYSICAL_TRACK_RUN":
        raise ValueError("expected PHYSICAL_TRACK_RUN: {}".format(summary_path))
    require_boolean(summary, "completed", True)
    require_boolean(summary, "run_successful", True)
    require_boolean(summary, "run_valid_for_results", False)
    require_boolean(summary, "physical_motor_interface_opened", True)
    require_boolean(summary, "physical_motor_commands", True)
    if summary.get("run_purpose") != "tuning":
        raise ValueError("source run must have run_purpose=tuning")
    if summary.get("controller_verification_state") != PROVISIONAL_STATE:
        raise ValueError("source run was not made with the provisional controller")
    expected_result_requirements = {
        "minimum_drive_rows": MINIMUM_FINAL_DRIVE_ROWS,
        "minimum_motion_rows": MINIMUM_FINAL_MOTION_ROWS,
        "minimum_unique_yolo_results": MINIMUM_FINAL_UNIQUE_YOLO_RESULTS,
    }
    if summary.get("result_acceptance_requirements") != expected_result_requirements:
        raise ValueError(
            "source run result-acceptance policy differs from this promotion "
            "tool: recorded={!r} expected={!r}".format(
                summary.get("result_acceptance_requirements"),
                expected_result_requirements,
            )
        )
    if summary.get("error") is not None:
        raise ValueError("source run contains an error: {!r}".format(
            summary.get("error")
        ))
    if str(summary.get("yolo_device", "")).lower() not in ("cuda", "cuda:0"):
        raise ValueError("source run did not use CUDA YOLO")
    require_boolean(summary, "yolo_fp16", True)
    if summary.get("watchdog_fault") is not None:
        raise ValueError("source run contains a watchdog fault")
    if require_integer(summary, "watchdog_trip_count") != 0:
        raise ValueError("source run contains a watchdog trip")

    duration = require_number(summary, "requested_maximum_duration_s")
    if duration < 0.5 - FLOAT_TOLERANCE or duration > 2.0 + FLOAT_TOLERANCE:
        raise ValueError("tuning duration must be within 0.5..2.0 seconds")
    total_energized = require_number(
        summary, "actual_total_energized_duration_s"
    )
    maximum_energized = require_number(
        summary, "maximum_continuous_energized_duration_s"
    )
    if not 0.0 < total_energized <= duration + FLOAT_TOLERANCE:
        raise ValueError("total energized duration is outside the run bound")
    if not 0.0 < maximum_energized <= duration + FLOAT_TOLERANCE:
        raise ValueError("continuous energized duration is outside the run bound")
    deadline = summary.get("deadline_stopper")
    if not isinstance(deadline, dict) or deadline.get("stop_error") is not None:
        raise ValueError("deadline stopper is missing or contains an error")

    minimum_voltage = require_number(summary, "minimum_voltage_v")
    if minimum_voltage < float(current_config["motor_voltage_stop_v"]):
        raise ValueError(
            "minimum voltage {:.3f} V is below the configured {:.3f} V stop".format(
                minimum_voltage,
                float(current_config["motor_voltage_stop_v"]),
            )
        )
    validate_controller_identity(summary, current_config)
    validate_motor_mapping_identity(summary, current_motor_config)
    validate_gate_snapshot(
        summary, current_gate_state, current_controller_sha256
    )

    csv_path = summary.get("csv")
    if not isinstance(csv_path, str) or not csv_path:
        raise ValueError("linked CSV path is missing")
    csv_path = os.path.realpath(os.path.abspath(csv_path))
    if not os.path.isfile(csv_path):
        raise ValueError("linked CSV does not exist: {}".format(csv_path))
    expected_csv_hash = summary.get("csv_sha256")
    csv_bytes = read_file_bytes(csv_path)
    actual_csv_hash = sha256_bytes(csv_bytes)
    if (
        not isinstance(expected_csv_hash, str)
        or len(expected_csv_hash) != 64
        or expected_csv_hash.lower() != actual_csv_hash
    ):
        raise ValueError("linked CSV SHA-256 does not match the summary")

    try:
        csv_text = csv_bytes.decode("utf-8-sig")
    except UnicodeDecodeError as error:
        raise ValueError("linked CSV is not UTF-8: {}".format(error))
    rows = list(csv.DictReader(io.StringIO(csv_text)))
    if not rows:
        raise ValueError("linked CSV is empty")
    run_id = summary.get("run_id")
    if not isinstance(run_id, str) or not run_id:
        raise ValueError("run_id is missing")
    if any(row.get("run_id") != run_id for row in rows):
        raise ValueError("CSV contains rows from another run_id")
    if require_integer(summary, "frames") != len(rows):
        raise ValueError("CSV row count does not match summary frames")
    for expected_frame, row in enumerate(rows, 1):
        try:
            frame_id = int(row.get("frame_id", ""))
        except ValueError:
            raise ValueError("CSV frame_id is not an integer")
        if frame_id != expected_frame:
            raise ValueError("CSV frame_id sequence is incomplete")

    safety_counts = collections.Counter(
        row.get("safety_state", "") for row in rows
    )
    reason_counts = collections.Counter(
        row.get("safety_reason", "") for row in rows
    )
    lane_counts = collections.Counter(row.get("lane_status", "") for row in rows)
    motion_rows = sum(
        1 for row in rows if row_truth(row.get("physical_motor_commands", ""))
    )
    fresh_yolo_source_frames = set()
    for row in rows:
        source_frame = str(row.get("yolo_source_frame", "")).strip()
        if source_frame and row_truth(row.get("yolo_fresh", "")):
            fresh_yolo_source_frames.add(source_frame)
    if summary.get("safety_state_counts") != dict(safety_counts):
        raise ValueError("CSV safety-state counts do not match the summary")
    if summary.get("safety_reason_counts") != dict(reason_counts):
        raise ValueError("CSV safety-reason counts do not match the summary")
    if require_integer(summary, "motion_command_frames") != motion_rows:
        raise ValueError("CSV motion-row count does not match the summary")
    if require_integer(summary, "unique_yolo_results") != len(
        fresh_yolo_source_frames
    ):
        raise ValueError("CSV unique-YOLO count does not match the summary")

    return {
        "summary": summary_path,
        "summary_sha256": sha256_bytes(summary_bytes),
        "csv": csv_path,
        "csv_sha256": actual_csv_hash,
        "run_id": run_id,
        "completed_utc": summary.get("completed_utc"),
        "requested_maximum_duration_s": duration,
        "frames": len(rows),
        "drive_rows": int(safety_counts.get("DRIVE", 0)),
        "motion_rows": motion_rows,
        "unique_yolo_results": len(fresh_yolo_source_frames),
        "lane_status_counts": dict(lane_counts),
        "safety_state_counts": dict(safety_counts),
        "minimum_voltage_v": minimum_voltage,
        "mean_absolute_lane_offset": summary.get("mean_absolute_lane_offset"),
        "actual_total_energized_duration_s": total_energized,
    }


def match_ladder(records):
    unmatched = list(records)
    matched = []
    for expected in EXPECTED_DURATIONS:
        candidate_index = None
        for index, record in enumerate(unmatched):
            if abs(record["requested_maximum_duration_s"] - expected) <= FLOAT_TOLERANCE:
                candidate_index = index
                break
        if candidate_index is None:
            raise ValueError(
                "evidence ladder must contain exact 0.5 s, 1.0 s and 2.0 s runs"
            )
        matched.append(unmatched.pop(candidate_index))
    if unmatched:
        raise ValueError("supply exactly three tuning summaries")
    run_ids = [record["run_id"] for record in matched]
    if len(set(run_ids)) != len(run_ids):
        raise ValueError("tuning ladder run_id values must be distinct")
    completion_times = [
        parse_completed_utc(record["completed_utc"])
        for record in matched
    ]
    if not all(
        earlier < later
        for earlier, later in zip(completion_times, completion_times[1:])
    ):
        raise ValueError(
            "tuning ladder must be chronological: 0.5 s, then 1.0 s, "
            "then 2.0 s"
        )
    return matched


def parse_completed_utc(value):
    if not isinstance(value, str) or not value.endswith("Z"):
        raise ValueError("completed_utc must be an ISO-8601 UTC string")
    timestamp = value[:-1]
    for format_string in ("%Y-%m-%dT%H:%M:%S.%f", "%Y-%m-%dT%H:%M:%S"):
        try:
            return datetime.datetime.strptime(timestamp, format_string)
        except ValueError:
            pass
    raise ValueError("completed_utc is not a recognized UTC timestamp: {}".format(
        value
    ))


def find_latest_ladder_paths():
    results_directory = os.path.join(PROJECT_DIRECTORY, "results")
    if not os.path.isdir(results_directory):
        raise ValueError("results directory does not exist: {}".format(
            results_directory
        ))
    candidates = []
    for file_name in os.listdir(results_directory):
        if not file_name.endswith(".json") or file_name.endswith(".analysis.json"):
            continue
        path = os.path.join(results_directory, file_name)
        try:
            summary = read_strict_json(path)
            duration = require_number(
                summary, "requested_maximum_duration_s"
            )
        except (IOError, OSError, ValueError):
            continue
        if (
            summary.get("test") != "PHYSICAL_TRACK_RUN"
            or summary.get("run_purpose") != "tuning"
            or summary.get("run_successful") is not True
            or summary.get("controller_verification_state") != PROVISIONAL_STATE
        ):
            continue
        candidates.append((os.path.getmtime(path), duration, path))

    selected = []
    for expected in EXPECTED_DURATIONS:
        matches = [
            item
            for item in candidates
            if abs(item[1] - expected) <= FLOAT_TOLERANCE
        ]
        if not matches:
            raise ValueError(
                "no successful provisional {:.1f}-second tuning summary was found".format(
                    expected
                )
            )
        selected.append(max(matches, key=lambda item: item[0])[2])
    return selected


def require_final_observation_counts(final_record):
    failures = []
    if final_record["drive_rows"] < MINIMUM_FINAL_DRIVE_ROWS:
        failures.append(
            "DRIVE rows {} < {}".format(
                final_record["drive_rows"], MINIMUM_FINAL_DRIVE_ROWS
            )
        )
    if final_record["motion_rows"] < MINIMUM_FINAL_MOTION_ROWS:
        failures.append(
            "motion rows {} < {}".format(
                final_record["motion_rows"], MINIMUM_FINAL_MOTION_ROWS
            )
        )
    if (
        final_record["unique_yolo_results"]
        < MINIMUM_FINAL_UNIQUE_YOLO_RESULTS
    ):
        failures.append(
            "unique YOLO results {} < {}".format(
                final_record["unique_yolo_results"],
                MINIMUM_FINAL_UNIQUE_YOLO_RESULTS,
            )
        )
    if failures:
        raise ValueError(
            "2.0-second candidate lacks enough observations: {}".format(
                "; ".join(failures)
            )
        )


def utc_now():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def atomic_json_write(path, value):
    temporary = path + ".tmp"
    try:
        with open(temporary, "w") as output_file:
            json.dump(
                value,
                output_file,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            output_file.write("\n")
            output_file.flush()
            os.fsync(output_file.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def atomic_bytes_write(path, value):
    temporary = path + ".tmp"
    try:
        with open(temporary, "wb") as output_file:
            output_file.write(value)
            output_file.flush()
            os.fsync(output_file.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def promote(
    records,
    current_config,
    original_controller_bytes,
    expected_gate_identity,
):
    expected_controller_sha256 = sha256_bytes(original_controller_bytes)
    promoted = dict(current_config)
    promoted["verification_state"] = TUNED_STATE
    notes = promoted.get("notes", [])
    if notes is None:
        notes = []
    if not isinstance(notes, list) or any(not isinstance(note, str) for note in notes):
        raise ValueError("controller notes must be a list of strings")
    notes = [
        note
        for note in notes
        if "gains are provisional" not in note.lower()
    ]
    notes.append(
        "Existing PID gains were physically accepted after guarded 0.5 s, "
        "1.0 s and 2.0 s closed-loop tuning trials. See "
        "physical_tuning_evidence for recorded source hashes."
    )
    promoted["notes"] = notes
    promoted["physical_tuning_evidence"] = {
        "accepted_utc": utc_now(),
        "operator_confirmation": CONFIRMATION_TOKEN,
        "acceptance_basis": (
            "Operator observed stable contained steering and automatic stops; "
            "all three raw summary/CSV pairs were internally consistent and "
            "matched one current validated artifact identity; the 2.0-second "
            "run met track_run.py's native minimum DRIVE, motion, and "
            "fresh-YOLO result-acceptance counts. Safety STOP rows remain "
            "preserved in the raw evidence."
        ),
        "minimum_final_observation_requirements": {
            "drive_rows": MINIMUM_FINAL_DRIVE_ROWS,
            "motion_rows": MINIMUM_FINAL_MOTION_ROWS,
            "unique_yolo_results": MINIMUM_FINAL_UNIQUE_YOLO_RESULTS,
        },
        "validation_identity_at_tuning": expected_gate_identity,
        "runs": records,
    }
    validate_controller_config(promoted)
    if sha256_file(CONTROLLER_CONFIG_PATH) != expected_controller_sha256:
        raise RuntimeError(
            "controller_config.json changed during evidence review; retry"
        )
    latest_gate_state = read_json(GATES_PATH)
    if (
        latest_gate_state.get("physical_motion_authorized") is not True
        or latest_gate_state.get("authorization_integrity_status") != "CURRENT"
        or gate_state_identity(latest_gate_state) != expected_gate_identity
    ):
        raise RuntimeError(
            "validation gate identity changed during evidence review; retry"
        )
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    backup_path = os.path.join(
        PROJECT_DIRECTORY,
        "controller_config.before_physical_tuning_{}.json".format(stamp),
    )
    shutil.copy2(CONTROLLER_CONFIG_PATH, backup_path)
    atomic_json_write(CONTROLLER_CONFIG_PATH, promoted)
    try:
        reloaded = load_controller_config(CONTROLLER_CONFIG_PATH)
        if reloaded.get("verification_state") != TUNED_STATE:
            raise RuntimeError("written verification state is not PHYSICALLY_TUNED")
    except Exception as error:
        atomic_bytes_write(CONTROLLER_CONFIG_PATH, original_controller_bytes)
        if sha256_file(CONTROLLER_CONFIG_PATH) != expected_controller_sha256:
            raise RuntimeError(
                "promotion verification and exact rollback both failed: {}".format(
                    error
                )
            )
        raise RuntimeError(
            "atomic promotion verification failed; original config was "
            "restored: {}".format(error)
        )
    return backup_path, promoted


def main(argv=None, input_function=input):
    parser = argparse.ArgumentParser(
        description="Promote controller gains from a guarded tuning ladder"
    )
    parser.add_argument(
        "summary_json",
        nargs="*",
        help="exact 0.5 s, 1.0 s and 2.0 s tuning summary JSON paths",
    )
    parser.add_argument(
        "--latest",
        action="store_true",
        help=(
            "select the newest successful provisional result for each exact "
            "0.5 s, 1.0 s and 2.0 s duration"
        ),
    )
    arguments = parser.parse_args(argv)
    try:
        controller_bytes = read_file_bytes(CONTROLLER_CONFIG_PATH)
        current_controller_sha256 = sha256_bytes(controller_bytes)
        current_config = validate_controller_config(
            parse_strict_json_bytes(controller_bytes, CONTROLLER_CONFIG_PATH)
        )
        if current_config.get("verification_state") == TUNED_STATE:
            print("NO CHANGE: controller_config.json is already PHYSICALLY_TUNED.")
            print("Run: python validation_manager.py verify")
            return 0
        if current_config.get("verification_state") != PROVISIONAL_STATE:
            raise ValueError("controller verification_state is not recognized")

        current_motor_config = load_motor_config()
        current_gate_state = read_json(GATES_PATH)
        if current_gate_state.get("physical_motion_authorized") is not True:
            raise ValueError(
                "current validation gates are not fully authorized; run "
                "python validation_manager.py verify"
            )
        if current_gate_state.get("authorization_integrity_status") != "CURRENT":
            raise ValueError("current authorization artifact state is not CURRENT")
        current_gate_identity = gate_state_identity(current_gate_state)

        if arguments.latest:
            if arguments.summary_json:
                raise ValueError("use either --latest or three explicit paths")
            summary_paths = find_latest_ladder_paths()
            print("Newest successful tuning ladder selected:")
            for path in summary_paths:
                print("  {}".format(path))
        else:
            if len(arguments.summary_json) != 3:
                raise ValueError(
                    "supply three summary paths, or use --latest"
                )
            summary_paths = arguments.summary_json

        records = [
            validate_summary_and_csv(
                path,
                current_config,
                current_motor_config,
                current_gate_state,
                current_controller_sha256,
            )
            for path in summary_paths
        ]
        records = match_ladder(records)
        require_final_observation_counts(records[-1])

        print("=" * 72)
        print("CONTROLLER PHYSICAL-TUNING EVIDENCE REVIEW")
        print("=" * 72)
        for record in records:
            print(
                "{:.1f} s | {} | frames {} | DRIVE {} | motion {} | "
                "YOLO {} | min {:.3f} V".format(
                    record["requested_maximum_duration_s"],
                    record["run_id"],
                    record["frames"],
                    record["drive_rows"],
                    record["motion_rows"],
                    record["unique_yolo_results"],
                    record["minimum_voltage_v"],
                )
            )
        print("All three JSON/CSV pairs are internally consistent and match")
        print("the same current validated build identity and safety limits.")
        print("These editable files are not cryptographically authenticated.")
        print("This tool does not infer physical behaviour that was not logged.")
        print("Safety STOP rows remain evidence and are not treated as motion.")
        print("Promotion confirms tuning behaviour, not full-loop completion.")
        print("Confirm only if every run stayed inside the lane, steering was")
        print("controlled, there was no collision, and every run stopped itself.")
        response = input_function(
            "Type {} to accept the current PID gains: ".format(
                CONFIRMATION_TOKEN
            )
        ).strip()
        if response != CONFIRMATION_TOKEN:
            print("CANCELLED: controller_config.json was not changed.")
            return 1

        backup_path, promoted = promote(
            records,
            current_config,
            controller_bytes,
            current_gate_identity,
        )
        print("=" * 72)
        print("PROMOTION COMPLETE")
        print("Controller state: {}".format(promoted["verification_state"]))
        print("Backup: {}".format(backup_path))
        print("Config: {}".format(CONTROLLER_CONFIG_PATH))
        print("EXPECTED: controller-bound gates are now stale.")
        print("No result run is authorized until those gates are revalidated.")
        print("Next: python validation_manager.py show")
        return 0
    except (IOError, OSError, RuntimeError, ValueError, KeyError) as error:
        print("PROMOTION REFUSED: {}".format(error))
        return 1


if __name__ == "__main__":
    sys.exit(main())
