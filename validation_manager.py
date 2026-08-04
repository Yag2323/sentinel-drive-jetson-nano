#!/usr/bin/env python3
"""Record and verify evidence-backed integration gates; never command motors.

Every passed gate is bound to both its evidence file and the exact project
artifacts that were present when the evidence was recorded.  Importers such as
``track_run.py`` receive an in-memory fail-safe view: a missing or changed file
invalidates the affected gate and makes physical motion unauthorized.

Compatible with Python 3.6.
"""

from __future__ import print_function

import argparse
import copy
import datetime
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys

from source_integrity import git_source_state, source_tree_sha256
from single_line_lane import SINGLE_LINE_DETECTOR_MODE


PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
GATES_PATH = os.path.join(PROJECT_DIRECTORY, "integration_gates.json")
IPM_CONFIG_PATH = os.path.join(PROJECT_DIRECTORY, "ipm_config.json")
EVIDENCE_DIRECTORY = os.path.join(PROJECT_DIRECTORY, "evidence")
EXPECTED_YOLOV5_V6_COMMIT = "956be8e642b5c10af4a1533e09084ca32ff4f21f"
EXPECTED_YOLOV5N_SHA256 = "649e089f59b78ac021025de035b2d9c45dc26e544ea252955d0ffcefc1099e2f"
EXPECTED_TARGET_LINE_ROLE = "OUTER_CIRCLE_CENTERLINE"

REQUIRED_MOTION_GATES = (
    "camera_orientation",
    "motor_mapping_software",
    "yolo_current_regression",
    "ipm_physical_calibration",
    "ipm_live_dry_run",
    "outer_circle_positions",
    "integrated_control_dry_run",
    "raised_motor_adapter",
    "short_floor_steering",
)

EVIDENCE_RULES = {
    "camera_orientation": "PHYSICAL_CSI_CAMERA_ORIENTATION",
    "motor_mapping_software": "INTEGRATION_SOFTWARE_SELF_TEST",
    "yolo_current_regression": "YOLOV5N_REAL_CSI_BENCHMARK",
    "ipm_physical_calibration": "PHYSICAL_IPM_CALIBRATION",
    "ipm_live_dry_run": "PHYSICAL_CSI_IPM_DRY_RUN",
    "outer_circle_positions": "OUTER_CIRCLE_REPRESENTATIVE_POSITIONS",
    "integrated_control_dry_run": "INTEGRATED_PERCEPTION_CONTROL_DRY_RUN",
    "raised_motor_adapter": "RAISED_STEERING_AND_COMMAND_LEASE_VALIDATION",
    "short_floor_steering": "SHORT_FLOOR_STEERING_VALIDATION",
}

GATE_PREREQUISITES = {
    "camera_orientation": (),
    "motor_mapping_software": (),
    "yolo_current_regression": ("camera_orientation",),
    "ipm_physical_calibration": ("camera_orientation",),
    "ipm_live_dry_run": (
        "camera_orientation",
        "ipm_physical_calibration",
    ),
    "outer_circle_positions": (
        "camera_orientation",
        "ipm_physical_calibration",
        "ipm_live_dry_run",
    ),
    "integrated_control_dry_run": (
        "camera_orientation",
        "motor_mapping_software",
        "yolo_current_regression",
        "ipm_physical_calibration",
        "ipm_live_dry_run",
        "outer_circle_positions",
    ),
    "raised_motor_adapter": (
        "camera_orientation",
        "motor_mapping_software",
        "yolo_current_regression",
        "ipm_physical_calibration",
        "ipm_live_dry_run",
        "outer_circle_positions",
        "integrated_control_dry_run",
    ),
    "short_floor_steering": (
        "camera_orientation",
        "motor_mapping_software",
        "yolo_current_regression",
        "ipm_physical_calibration",
        "ipm_live_dry_run",
        "outer_circle_positions",
        "integrated_control_dry_run",
        "raised_motor_adapter",
    ),
}

GATE_CLASSIFICATIONS = {
    "camera_orientation": "PHYSICALLY_VERIFIED",
    "motor_mapping_software": "SOFTWARE_VERIFIED",
    "yolo_current_regression": "SOFTWARE_VERIFIED_ON_PHYSICAL_CSI",
    "ipm_physical_calibration": "PHYSICALLY_VERIFIED",
    "ipm_live_dry_run": "SOFTWARE_VERIFIED_ON_PHYSICAL_CSI",
    "outer_circle_positions": "PHYSICALLY_VERIFIED",
    "integrated_control_dry_run": "SOFTWARE_VERIFIED_ON_PHYSICAL_CSI",
    "raised_motor_adapter": "PHYSICALLY_VERIFIED",
    "short_floor_steering": "PHYSICALLY_VERIFIED",
}

# ``validation_manager.py`` is included in every gate so a change to the
# integrity policy itself invalidates previously recorded evidence.
GATE_ARTIFACTS = {
    "camera_orientation": (
        "validation_manager.py",
        "patch_dashboard_camera_rotation.py",
        "robot_dashboard.py",
        "gst_camera_bridge.py",
    ),
    "motor_mapping_software": (
        "validation_manager.py",
        "integration_self_test.py",
        "control_core.py",
        "motor_mapping.py",
        "safe_motor_output.py",
        "motor_config.json",
        "controller_config.json",
    ),
    "yolo_current_regression": (
        "validation_manager.py",
        "source_integrity.py",
        "gst_camera_bridge.py",
        "yolov5_runtime.py",
        "yolo_csi_benchmark.py",
        "yolov5_v6",
        "yolov5_v6/yolov5n.pt",
    ),
    "ipm_physical_calibration": (
        "validation_manager.py",
        "gst_camera_bridge.py",
        "ipm_lane.py",
        "single_line_lane.py",
        "calibrate_ipm.py",
        "ipm_config.json",
    ),
    "ipm_live_dry_run": (
        "validation_manager.py",
        "gst_camera_bridge.py",
        "ipm_lane.py",
        "single_line_lane.py",
        "ipm_live_dry_run.py",
        "ipm_config.json",
    ),
    "outer_circle_positions": (
        "validation_manager.py",
        "gst_camera_bridge.py",
        "ipm_lane.py",
        "single_line_lane.py",
        "outer_circle_position_validation.py",
        "ipm_config.json",
    ),
    "integrated_control_dry_run": (
        "validation_manager.py",
        "source_integrity.py",
        "gst_camera_bridge.py",
        "ipm_lane.py",
        "single_line_lane.py",
        "ipm_config.json",
        "yolov5_runtime.py",
        "yolov5_v6",
        "yolov5_v6/yolov5n.pt",
        "control_core.py",
        "motor_mapping.py",
        "motor_config.json",
        "controller_config.json",
        "integrated_control_dry_run.py",
    ),
    "raised_motor_adapter": (
        "validation_manager.py",
        "manual_motor_control.py",
        "safe_motor_output.py",
        "motor_mapping.py",
        "motor_config.json",
        "controller_config.json",
        "motor_adapter_raised_test.py",
    ),
    "short_floor_steering": (
        "validation_manager.py",
        "manual_motor_control.py",
        "safe_motor_output.py",
        "motor_mapping.py",
        "motor_config.json",
        "controller_config.json",
        "steering_floor_validation.py",
    ),
}

# These are checked once all individual gates pass.  This prevents a runner or
# integrity-policy edit after validation from silently retaining authorization.
AUTHORIZATION_ARTIFACTS = (
    "validation_manager.py",
    "source_integrity.py",
    "track_run.py",
    "guarded_live_circle_run.py",
    "guarded_straight_line_run.py",
)


def utc_now():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def reject_non_finite_json(value):
    raise ValueError("non-finite JSON number is not permitted: {}".format(value))


def parse_finite_json_float(value):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("JSON number overflows finite range: {}".format(value))
    return parsed


def read_json_unverified(path):
    with open(path, "r") as input_file:
        return json.load(
            input_file,
            parse_constant=reject_non_finite_json,
            parse_float=parse_finite_json_float,
        )


def evidence_number(mapping, key):
    """Return a finite JSON number; reject bools, strings and infinities."""
    value = mapping.get(key) if isinstance(mapping, dict) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a JSON number".format(key))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(key))
    return value


def evidence_integer(mapping, key):
    """Return a strict JSON integer; reject booleans and numeric strings."""
    value = mapping.get(key) if isinstance(mapping, dict) else None
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} must be a JSON integer".format(key))
    return value


def same_path(left, right):
    return os.path.realpath(os.path.abspath(left)) == os.path.realpath(
        os.path.abspath(right)
    )


def read_json(path):
    """Read JSON; gate-state reads are integrity-verified automatically.

    ``track_run.py`` imports this function, so stale artifacts fail closed even
    if the stored JSON still contains a historical ``passed: true`` value.
    """
    value = read_json_unverified(path)
    if same_path(path, GATES_PATH) and isinstance(value, dict):
        return verify_gate_state(value, require_authorization_snapshot=True)
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


def project_path(relative_path):
    return os.path.join(PROJECT_DIRECTORY, relative_path.replace("/", os.sep))


def hash_artifacts(relative_paths):
    hashes = {}
    missing = []
    for relative_path in relative_paths:
        path = project_path(relative_path)
        if os.path.isfile(path):
            hashes[relative_path] = sha256_file(path)
        elif os.path.isdir(path):
            hashes[relative_path] = source_tree_sha256(path)[0]
        else:
            missing.append(relative_path)
    if missing:
        raise RuntimeError(
            "Required artifacts are missing: {}".format(", ".join(missing))
        )
    return hashes


def capture_artifact_snapshot(gate_name, excluded_paths=()):
    """Hash a gate's current artifacts for producer start/end evidence."""
    if gate_name not in GATE_ARTIFACTS:
        raise ValueError("Unknown validation gate: {}".format(gate_name))
    excluded = set(excluded_paths)
    return hash_artifacts(
        relative_path
        for relative_path in GATE_ARTIFACTS[gate_name]
        if relative_path not in excluded
    )


def verify_hashes(recorded, required_paths):
    errors = []
    if not isinstance(recorded, dict):
        return ["artifact hashes were not recorded"]

    required = set(required_paths)
    recorded_names = set(recorded.keys())
    for relative_path in sorted(required - recorded_names):
        errors.append("missing recorded hash: {}".format(relative_path))
    for relative_path in sorted(recorded_names - required):
        errors.append("artifact rule changed: {}".format(relative_path))

    for relative_path in sorted(required & recorded_names):
        path = project_path(relative_path)
        if os.path.isfile(path):
            current_hash = sha256_file(path)
        elif os.path.isdir(path):
            try:
                current_hash = source_tree_sha256(path)[0]
            except Exception as error:
                errors.append(
                    "artifact unreadable: {} ({})".format(
                        relative_path, error
                    )
                )
                continue
        else:
            errors.append("artifact missing: {}".format(relative_path))
            continue
        if current_hash != recorded.get(relative_path):
            errors.append("artifact changed: {}".format(relative_path))
    return errors


def verify_evidence_file(gate):
    errors = []
    evidence_path = gate.get("evidence")
    expected_hash = gate.get("evidence_sha256")
    if not evidence_path or not isinstance(evidence_path, str):
        return ["evidence path was not recorded"]
    if not expected_hash or not isinstance(expected_hash, str):
        return ["evidence hash was not recorded"]
    if not os.path.isfile(evidence_path):
        return ["evidence file is missing: {}".format(evidence_path)]
    if sha256_file(evidence_path) != expected_hash:
        errors.append("evidence file changed: {}".format(evidence_path))
    return errors


def verify_gate_state(gates, require_authorization_snapshot=True):
    """Return a deep-copied, fail-safe gate view with current integrity state."""
    state = copy.deepcopy(gates)
    gate_items = state.setdefault("gates", {})

    for gate_name in REQUIRED_MOTION_GATES:
        item = gate_items.setdefault(
            gate_name,
            {
                "passed": False,
                "classification": "UNVERIFIED",
                "evidence": "Gate definition is missing.",
            },
        )
        if item.get("passed") is not True:
            if item.get("integrity_status") != "STALE":
                item["integrity_status"] = "UNRECORDED"
            continue

        errors = []
        errors.extend(verify_evidence_file(item))
        errors.extend(
            verify_hashes(
                item.get("artifact_sha256"),
                GATE_ARTIFACTS[gate_name],
            )
        )
        if errors:
            item["passed"] = False
            item["integrity_status"] = "STALE"
            item["integrity_errors"] = errors
        else:
            item["integrity_status"] = "CURRENT"
            item.pop("integrity_errors", None)

    # A current downstream artifact is not sufficient when evidence it relied
    # on has become stale. The required-gate order is topological, so one pass
    # propagates every prerequisite failure through raised and floor motion.
    for gate_name in REQUIRED_MOTION_GATES:
        item = gate_items[gate_name]
        if item.get("passed") is not True:
            continue
        unavailable = [
            prerequisite
            for prerequisite in GATE_PREREQUISITES.get(gate_name, ())
            if gate_items.get(prerequisite, {}).get("passed") is not True
        ]
        if unavailable:
            item["passed"] = False
            item["integrity_status"] = "STALE"
            item.setdefault("integrity_errors", []).append(
                "prerequisite gate not current: {}".format(
                    ", ".join(unavailable)
                )
            )

    all_gates_pass = all(
        gate_items.get(name, {}).get("passed") is True
        for name in REQUIRED_MOTION_GATES
    )
    authorization_errors = []
    if all_gates_pass and require_authorization_snapshot:
        authorization_errors = verify_hashes(
            state.get("authorization_artifact_sha256"),
            AUTHORIZATION_ARTIFACTS,
        )

    state["physical_motion_authorized"] = bool(
        all_gates_pass and not authorization_errors
    )
    if authorization_errors:
        state["authorization_integrity_status"] = "STALE"
        state["authorization_integrity_errors"] = authorization_errors
    elif state["physical_motion_authorized"]:
        state["authorization_integrity_status"] = "CURRENT"
        state.pop("authorization_integrity_errors", None)
    else:
        state["authorization_integrity_status"] = "LOCKED"
        state.pop("authorization_integrity_errors", None)
    return state


def atomic_json_write(path, value):
    temporary = path + ".tmp"
    with open(temporary, "w") as output_file:
        json.dump(value, output_file, indent=2, sort_keys=True)
        output_file.write("\n")
        output_file.flush()
        os.fsync(output_file.fileno())
    os.replace(temporary, path)


def save_gates(gates):
    state = verify_gate_state(gates, require_authorization_snapshot=False)
    all_gates_pass = all(
        state["gates"].get(name, {}).get("passed") is True
        for name in REQUIRED_MOTION_GATES
    )
    if all_gates_pass:
        state["authorization_artifact_sha256"] = hash_artifacts(
            AUTHORIZATION_ARTIFACTS
        )
        state["physical_motion_authorized"] = True
        state["authorization_integrity_status"] = "CURRENT"
        state.pop("authorization_integrity_errors", None)
    else:
        state["physical_motion_authorized"] = False
        state["authorization_integrity_status"] = "LOCKED"
        state.pop("authorization_artifact_sha256", None)
        state.pop("authorization_integrity_errors", None)
    state["schema_version"] = 2

    if os.path.isfile(GATES_PATH):
        stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%S%fZ")
        backup = GATES_PATH + ".{}.bak".format(stamp)
        shutil.copy2(GATES_PATH, backup)
    atomic_json_write(GATES_PATH, state)


def print_status(gates):
    print("INTEGRATION VALIDATION GATES")
    for name in REQUIRED_MOTION_GATES:
        item = gates["gates"].get(name, {})
        integrity = item.get("integrity_status", "UNRECORDED")
        if item.get("passed"):
            label = "PASS"
        elif integrity == "STALE":
            label = "STALE"
        else:
            label = "WAIT"
        print("{} {:31s} {} / {}".format(
            label,
            name,
            item.get("classification", "UNKNOWN"),
            integrity,
        ))
        print("     {}".format(item.get("evidence", "No evidence")))
        for error in item.get("integrity_errors", []):
            print("     INTEGRITY: {}".format(error))
    for error in gates.get("authorization_integrity_errors", []):
        print("AUTHORIZATION INTEGRITY: {}".format(error))
    print("PHYSICAL MOTION AUTHORIZED: {}".format(
        bool(gates.get("physical_motion_authorized"))
    ))


def validate_evidence_payload(gate_name, evidence):
    expected_test = EVIDENCE_RULES[gate_name]
    if evidence.get("test") != expected_test:
        raise ValueError(
            "expected test {}, received {}".format(
                expected_test, evidence.get("test")
            )
        )
    if evidence.get("passed") is not True:
        raise ValueError("evidence does not contain passed=true")
    if evidence.get("artifact_snapshot_stable") is not True:
        raise ValueError("test artifacts were not stable for the complete run")
    if not isinstance(evidence.get("artifact_sha256_at_test"), dict):
        raise ValueError("evidence lacks the test-time artifact snapshot")

    if gate_name == "camera_orientation":
        if evidence.get("physical_motor_commands") is not False:
            raise ValueError("camera confirmation must declare motor commands false")
        if evidence.get("operator_confirmed_upright") is not True:
            raise ValueError("camera evidence requires operator_confirmed_upright=true")
        if evidence_integer(evidence, "flip_method") != 2:
            raise ValueError("camera evidence must confirm flip_method=2")
    elif gate_name == "motor_mapping_software":
        if evidence.get("physical_motor_commands") is not False:
            raise ValueError("software self-test must declare motor commands false")
        if evidence_integer(evidence, "self_test_return_code") != 0:
            raise ValueError("self-test evidence requires return code 0")
        if evidence.get("pass_marker") != "INTEGRATION SELF-TEST: PASS":
            raise ValueError("self-test PASS marker is missing")
    elif gate_name == "yolo_current_regression":
        if evidence.get("model_variant") != "yolov5n":
            raise ValueError("YOLO motion gate accepts only the yolov5n model")
        if evidence.get("motion_gate_eligible") is not True:
            raise ValueError("YOLO evidence is explicitly report-only")
        if str(evidence.get("device", "")).lower() not in ("cuda", "cuda:0"):
            raise ValueError("YOLO motion gate requires a CUDA benchmark")
        if evidence.get("fp16") is not True:
            raise ValueError("YOLO motion gate requires CUDA FP16 inference")
        if evidence.get("motor_commands") is not False:
            raise ValueError("YOLO benchmark must declare motor_commands=false")
        if evidence_integer(evidence, "camera_capture_fps_configured") != 21:
            raise ValueError(
                "YOLO motion gate requires the operational 21 Hz CSI source"
            )
        if evidence_integer(evidence, "camera_flip_method") != 2:
            raise ValueError("YOLO benchmark camera orientation is not flip 2")
        if evidence.get("measurement_window_completed") is not True:
            raise ValueError("YOLO benchmark did not complete its full window")
        if evidence_integer(evidence, "image_size") != 320:
            raise ValueError("YOLO motion gate requires the tested 320 input size")
        confidence = evidence_number(evidence, "confidence_threshold")
        iou = evidence_number(evidence, "iou_threshold")
        if not 0.0 < confidence <= 1.0 or not 0.0 < iou <= 1.0:
            raise ValueError("YOLO confidence/IoU thresholds are invalid")
        camera_pipeline = evidence.get("camera_pipeline")
        if (
            not isinstance(camera_pipeline, str)
            or "nvarguscamerasrc" not in camera_pipeline
            or "flip-method=2" not in camera_pipeline
            or "framerate=(fraction)21/1" not in camera_pipeline
        ):
            raise ValueError("YOLO evidence lacks the operational CSI pipeline")
        if evidence.get("yolov5_repository_commit") != EXPECTED_YOLOV5_V6_COMMIT:
            raise ValueError("YOLO benchmark is not from the pinned v6.0 commit")
        weights_hash = evidence.get("weights_sha256", "")
        if (
            not isinstance(weights_hash, str)
            or len(weights_hash) != 64
            or any(character not in "0123456789abcdef" for character in weights_hash.lower())
        ):
            raise ValueError("YOLO benchmark must record the weights SHA-256")
        current_weights_hash = sha256_file(
            project_path("yolov5_v6/yolov5n.pt")
        )
        if (
            weights_hash.lower() != current_weights_hash
            or current_weights_hash != EXPECTED_YOLOV5N_SHA256
        ):
            raise ValueError(
                "YOLO benchmark requires the pinned official v6.0 Nano weights"
            )
        if evidence.get("yolov5_tracked_source_clean") is not True:
            raise ValueError("YOLO benchmark requires a clean tracked source tree")
        if evidence.get("yolov5_source_tree_clean") is not True:
            raise ValueError("YOLO benchmark contains untracked or modified source")
        if evidence.get("source_integrity_stable_during_measurement") is not True:
            raise ValueError("YOLO source or weights changed during measurement")
        source_hash, source_file_count = source_tree_sha256(
            project_path("yolov5_v6")
        )
        if evidence.get("yolov5_source_tree_sha256") != source_hash:
            raise ValueError("YOLO benchmark source tree does not match current source")
        if evidence_integer(
            evidence, "yolov5_source_file_count"
        ) != source_file_count:
            raise ValueError("YOLO benchmark source-file count does not match")
        git_state = git_source_state(project_path("yolov5_v6"))
        if (
            git_state.get("commit") != EXPECTED_YOLOV5_V6_COMMIT
            or git_state.get("source_tree_clean") is not True
        ):
            raise ValueError("current YOLO source is not the pinned clean checkout")
        if evidence_integer(evidence, "frames") < 50:
            raise ValueError("YOLO benchmark requires at least 50 measured frames")
        if evidence_number(
            evidence, "complete_measurement_window_fps"
        ) <= 0.0:
            raise ValueError("YOLO benchmark did not record positive throughput")
    elif gate_name == "ipm_physical_calibration":
        if evidence.get("motor_commands") is not False:
            raise ValueError("IPM calibration must declare motor_commands=false")
        if evidence.get("calibration_state") != "PHYSICALLY_CALIBRATED":
            raise ValueError("IPM calibration state is not physical")
        if not os.path.isfile(str(evidence.get("evidence_image", ""))):
            raise ValueError("IPM calibration evidence image is missing")
        if evidence.get("detector_mode") != SINGLE_LINE_DETECTOR_MODE:
            raise ValueError("IPM calibration used the wrong detector mode")
        if evidence.get("target_line_role") != EXPECTED_TARGET_LINE_ROLE:
            raise ValueError("IPM calibration used the wrong target-line role")
        if evidence.get("calibration_target") != (
            "FOUR_CORNER_GROUND_PLANE_TARGET"
        ):
            raise ValueError(
                "IPM calibration must use the independent ground-plane target"
            )
    elif gate_name == "ipm_live_dry_run":
        if evidence.get("motor_commands") is not False:
            raise ValueError("IPM dry run must declare motor_commands=false")
        if evidence.get("calibration_state") != "PHYSICALLY_CALIBRATED":
            raise ValueError("IPM dry run must use physical calibration")
        if evidence.get("default_ipm_config_used") is not True:
            raise ValueError("IPM dry run must use the gate-bound default config")
        if evidence.get("detector_mode") != SINGLE_LINE_DETECTOR_MODE:
            raise ValueError("IPM dry run used the wrong detector mode")
        if evidence.get("target_line_role") != EXPECTED_TARGET_LINE_ROLE:
            raise ValueError("IPM dry run used the wrong target-line role")
        if evidence.get("outer_tape_only_operator_confirmed") is not True:
            raise ValueError(
                "IPM dry run did not confirm removal/masking of the inner "
                "circle"
            )
        if evidence.get("centred_reference_metric") != "near_field_offset":
            raise ValueError(
                "IPM centred-reference gate must use near_field_offset"
            )
        if evidence.get("measurement_window_completed") is not True:
            raise ValueError("IPM dry run did not complete its full window")
        if evidence_integer(evidence, "frames") < 50:
            raise ValueError("IPM dry run requires at least 50 frames")
        full_lane_rate = evidence_number(evidence, "full_lane_rate_pct")
        if not 90.0 <= full_lane_rate <= 100.0:
            raise ValueError("IPM dry run requires at least 90% FULL-lane observations")
        usable_lane_rate = evidence_number(evidence, "usable_lane_rate_pct")
        if not 95.0 <= usable_lane_rate <= 100.0:
            raise ValueError("IPM dry run requires at least 95% usable-lane observations")
        if evidence.get("expected_centered_reference") is not True:
            raise ValueError("IPM dry run must use a physically centred reference")
        mean_absolute_offset = evidence_number(
            evidence, "mean_absolute_offset"
        )
        if not 0.0 <= mean_absolute_offset <= 0.08:
            raise ValueError("IPM dry run mean absolute offset exceeds 0.08")
        mean_lane_confidence = evidence_number(
            evidence, "mean_lane_confidence"
        )
        if not 0.55 <= mean_lane_confidence <= 1.0:
            raise ValueError("IPM dry run mean lane confidence is below 0.55")
        if evidence_number(evidence, "average_loop_fps") < 5.0:
            raise ValueError("IPM dry run average loop rate is below 5 FPS")
    elif gate_name == "outer_circle_positions":
        if evidence.get("physical_motor_commands") is not False:
            raise ValueError(
                "outer-circle position validation must declare motor commands false"
            )
        if evidence.get("motor_commands") is not False:
            raise ValueError(
                "outer-circle position validation must not command motors"
            )
        if evidence.get("motor_modules_imported") is not False:
            raise ValueError(
                "outer-circle position validation imported a motor module"
            )
        if evidence.get("loaded_motor_modules") != []:
            raise ValueError(
                "outer-circle evidence lists loaded motor modules"
            )
        if evidence.get(
            "operator_confirmed_motor_battery_disconnected"
        ) is not True:
            raise ValueError(
                "outer-circle validation requires disconnected motor battery"
            )
        if evidence.get("outer_tape_only_operator_confirmed") is not True:
            raise ValueError(
                "outer-circle validation requires the inner circle removed "
                "or fully masked"
            )
        if evidence.get("default_ipm_config_used") is not True:
            raise ValueError(
                "outer-circle validation must use the gate-bound default config"
            )
        if evidence.get("calibration_state") != "PHYSICALLY_CALIBRATED":
            raise ValueError(
                "outer-circle validation must use physical calibration"
            )
        if evidence.get("detector_mode") != SINGLE_LINE_DETECTOR_MODE:
            raise ValueError(
                "outer-circle validation used the wrong detector mode"
            )
        if evidence.get("target_line_role") != EXPECTED_TARGET_LINE_ROLE:
            raise ValueError(
                "outer-circle validation used the wrong target-line role"
            )
        if evidence.get("all_positions_passed") is not True:
            raise ValueError(
                "not every representative outer-circle position passed"
            )
        expected_positions = [
            "STRAIGHT",
            "CURVE_ENTRY",
            "CURVE_APEX",
            "CURVE_EXIT",
        ]
        if evidence.get("positions_expected") != expected_positions:
            raise ValueError(
                "outer-circle evidence has the wrong position sequence"
            )
        near_limit = evidence_number(
            evidence, "maximum_absolute_near_field_offset"
        )
        if not 0.0 < near_limit <= 0.25:
            raise ValueError(
                "outer-circle near-field limit must be no greater than 0.25"
            )
        if evidence_number(evidence, "minimum_confidence") < 0.55:
            raise ValueError(
                "outer-circle confidence requirement is below 0.55"
            )
        if evidence_number(evidence, "maximum_camera_age_s") > 0.25:
            raise ValueError(
                "outer-circle camera-age limit exceeds 0.25 seconds"
            )
        if evidence_integer(evidence, "minimum_samples_per_position") < 10:
            raise ValueError(
                "outer-circle validation requires at least 10 samples per position"
            )
        positions = evidence.get("positions")
        if not isinstance(positions, list) or [
            item.get("position") if isinstance(item, dict) else None
            for item in positions
        ] != expected_positions:
            raise ValueError(
                "outer-circle evidence does not contain all four positions"
            )
        for item in positions:
            if item.get("passed") is not True:
                raise ValueError(
                    "outer-circle position {} did not pass".format(
                        item.get("position")
                    )
                )
            if item.get("measurement_window_completed") is not True:
                raise ValueError(
                    "outer-circle position window did not complete"
                )
            samples = evidence_integer(item, "samples")
            if samples < 10:
                raise ValueError(
                    "outer-circle position has fewer than 10 samples"
                )
            if evidence_integer(item, "trustworthy_samples") != samples:
                raise ValueError(
                    "outer-circle position contains untrustworthy samples"
                )
            if evidence_integer(item, "full_samples") != samples:
                raise ValueError(
                    "outer-circle position contains a non-FULL sample"
                )
            if evidence_integer(item, "failed_samples") != 0:
                raise ValueError(
                    "outer-circle position contains failed samples"
                )
            if evidence_integer(item, "stale_samples") != 0:
                raise ValueError(
                    "outer-circle position contains stale samples"
                )
            confidence = evidence_number(item, "minimum_confidence")
            if not 0.55 <= confidence <= 1.0:
                raise ValueError(
                    "outer-circle position confidence fell below 0.55"
                )
            maximum_offset = evidence_number(
                item, "maximum_absolute_near_field_offset"
            )
            if not 0.0 <= maximum_offset <= near_limit:
                raise ValueError(
                    "outer-circle position near-field offset exceeded its limit"
                )
            maximum_age = evidence_number(item, "maximum_camera_age_s")
            if not 0.0 <= maximum_age <= 0.25:
                raise ValueError(
                    "outer-circle position camera observation became stale"
                )
            if not os.path.isfile(str(item.get("evidence_image", ""))):
                raise ValueError(
                    "outer-circle position evidence image is missing"
                )
        if not os.path.isfile(str(evidence.get("csv", ""))):
            raise ValueError("outer-circle position CSV is missing")
        evidence_images = evidence.get("evidence_images")
        if not isinstance(evidence_images, list) or len(evidence_images) != 4:
            raise ValueError(
                "outer-circle validation requires four evidence images"
            )
        if any(not os.path.isfile(str(path)) for path in evidence_images):
            raise ValueError(
                "one or more outer-circle evidence images are missing"
            )
    elif gate_name == "integrated_control_dry_run":
        if evidence.get("physical_motor_commands") is not False:
            raise ValueError("integrated dry run must declare motor commands false")
        if str(evidence.get("yolo_device", "")).lower() not in (
            "cuda",
            "cuda:0",
        ):
            raise ValueError("integrated dry run requires current CUDA YOLO")
        if evidence.get("yolo_fp16") is not True:
            raise ValueError("integrated dry run requires CUDA FP16 inference")
        if evidence.get("ipm_calibration_state") != "PHYSICALLY_CALIBRATED":
            raise ValueError("integrated dry run must use physical IPM calibration")
        if evidence.get("default_ipm_config_used") is not True:
            raise ValueError("integrated dry run must use the gate-bound IPM config")
        if evidence.get("detector_mode") != SINGLE_LINE_DETECTOR_MODE:
            raise ValueError("integrated dry run used the wrong detector mode")
        if evidence.get("target_line_role") != EXPECTED_TARGET_LINE_ROLE:
            raise ValueError(
                "integrated dry run used the wrong target-line role"
            )
        if evidence.get("measurement_window_completed") is not True:
            raise ValueError("integrated dry run did not complete its full window")
        if evidence_integer(evidence, "frames") < 50:
            raise ValueError("integrated dry run requires at least 50 frames")
        full_lane_rate = evidence_number(evidence, "full_lane_rate_pct")
        if not 80.0 <= full_lane_rate <= 100.0:
            raise ValueError("integrated dry run requires at least 80% FULL-lane observations")
        usable_lane_rate = evidence_number(evidence, "usable_lane_rate_pct")
        if not 95.0 <= usable_lane_rate <= 100.0:
            raise ValueError("integrated dry run requires at least 95% usable-lane observations")
        yolo_fresh_rate = evidence_number(evidence, "yolo_fresh_rate_pct")
        if not 90.0 <= yolo_fresh_rate <= 100.0:
            raise ValueError("integrated dry run requires at least 90% fresh YOLO state")
        if evidence_number(evidence, "average_control_fps") < 5.0:
            raise ValueError("integrated dry run average control rate is below 5 FPS")
        safety_counts = evidence.get("safety_state_counts", {})
        if not isinstance(safety_counts, dict) or evidence_integer(
            safety_counts, "DRIVE"
        ) < 1:
            raise ValueError("integrated dry run must exercise a clear DRIVE state")
        if evidence_integer(evidence, "obstacle_stop_frames") < 1:
            raise ValueError("integrated dry run must exercise obstacle STOP")
        if evidence_integer(evidence, "lane_stop_frames") < 1:
            raise ValueError("integrated dry run must exercise lane-loss STOP")
        if evidence_integer(evidence, "controlled_frames") < 20:
            raise ValueError("integrated dry run requires at least 20 controlled frames")
        if evidence_integer(evidence, "steering_sign_mismatches") != 0:
            raise ValueError("integrated dry run contains steering-sign mismatches")
        if evidence_integer(evidence, "mapped_duty_violations") != 0:
            raise ValueError("integrated dry run contains mapped-duty violations")
        steering_saturation_rate = evidence_number(
            evidence, "steering_saturation_rate_pct"
        )
        if not 0.0 <= steering_saturation_rate <= 20.0:
            raise ValueError("integrated dry run steering saturation exceeds 20%")
        large_derivative_rate = evidence_number(
            evidence, "large_derivative_rate_pct"
        )
        if not 0.0 <= large_derivative_rate <= 10.0:
            raise ValueError("integrated dry run derivative-spike rate exceeds 10%")
        steering_flip_rate = evidence_number(
            evidence, "steering_flips_per_100_controlled_frames"
        )
        if not 0.0 <= steering_flip_rate <= 15.0:
            raise ValueError("integrated dry run steering-flip rate exceeds 15 per 100 frames")
    elif gate_name == "raised_motor_adapter":
        if evidence.get("physical_motor_commands") is not True:
            raise ValueError("raised test must declare physical_motor_commands=true")
        if evidence.get("wheels_raised") is not True:
            raise ValueError("raised test must declare wheels_raised=true")
        if evidence.get("command_lease_software_passed") is not True:
            raise ValueError("raised test command-lease software check did not pass")
        if evidence.get("command_lease_visual_confirmation") is not True:
            raise ValueError("raised test lacks visual command-lease confirmation")
        if evidence.get("command_lease_stop_reason") != "COMMAND_LEASE_EXPIRED":
            raise ValueError("raised test stop reason must be COMMAND_LEASE_EXPIRED")
        if evidence.get("hardware_off_confirmed") is not True:
            raise ValueError("raised test lacks confirmed hardware OFF state")
        if evidence.get("hardware_off_retry_required") is not False:
            raise ValueError("raised test ended with a pending hardware OFF retry")
        if evidence_integer(evidence, "watchdog_trip_count") != 1:
            raise ValueError("raised test requires exactly one lease trip")
        command_lease_seconds = evidence_number(
            evidence, "command_lease_seconds"
        )
        command_lease_duration = evidence_number(
            evidence, "command_lease_actual_energized_duration_s"
        )
        if not (
            0.0 < command_lease_duration
            <= command_lease_seconds + 0.20
        ):
            raise ValueError(
                "raised command-lease energized duration is outside its "
                "best-effort software bound"
            )
        pulse_seconds = evidence_number(evidence, "pulse_seconds")
        if not 0.0 < pulse_seconds <= 0.60:
            raise ValueError("raised steering pulses must be <= 0.60 seconds")
        maximum_actual = evidence_number(
            evidence, "maximum_actual_energized_duration_s"
        )
        if not 0.0 < maximum_actual <= pulse_seconds:
            raise ValueError(
                "raised measured energized duration exceeds its pulse limit"
            )
        phases = evidence.get("phases")
        expected_names = (
            "STRAIGHT",
            "RIGHT_CORRECTION",
            "LEFT_CORRECTION",
        )
        if not isinstance(phases, list) or [
            phase.get("name") if isinstance(phase, dict) else None
            for phase in phases
        ] != list(expected_names):
            raise ValueError(
                "raised test phases must be STRAIGHT, RIGHT_CORRECTION, "
                "LEFT_CORRECTION in that order"
            )
        for phase in phases:
            if phase.get("status") != "PASS":
                raise ValueError(
                    "raised phase {} does not have status PASS".format(
                        phase.get("name")
                    )
                )
            if phase.get("operator_confirmed") is not True:
                raise ValueError(
                    "raised phase {} lacks operator confirmation".format(
                        phase.get("name")
                    )
                )
            left_duty = evidence_number(phase, "left_mapped_duty")
            right_duty = evidence_number(phase, "right_mapped_duty")
            if not 0.0 <= left_duty <= 1.0 or not 0.0 <= right_duty <= 1.0:
                raise ValueError("raised test mapped duties must be within 0..1")
            actual_duration = evidence_number(
                phase, "actual_energized_duration_s"
            )
            if not 0.0 < actual_duration <= pulse_seconds:
                raise ValueError(
                    "raised phase energized duration exceeds its pulse limit"
                )
            stopper = phase.get("deadline_stopper")
            if not isinstance(stopper, dict) or stopper.get("stop_error") is not None:
                raise ValueError("raised phase deadline stopper is invalid")
    elif gate_name == "short_floor_steering":
        required_true = (
            "physical_motor_commands",
            "wheels_on_floor",
            "operator_confirmed_clear_lane_and_spotter",
            "straight_motion_passed",
            "left_correction_passed",
            "right_correction_passed",
            "controlled_stop_confirmed",
        )
        missing = [name for name in required_true if evidence.get(name) is not True]
        if missing:
            raise ValueError(
                "short floor evidence requires true: {}".format(", ".join(missing))
            )
        pulse_seconds = evidence_number(evidence, "maximum_pulse_s")
        if not 0.0 < pulse_seconds <= 0.60:
            raise ValueError("short floor steering pulses must be <= 0.60 seconds")
        maximum_actual = evidence_number(
            evidence, "maximum_actual_energized_duration_s"
        )
        if not 0.0 < maximum_actual <= pulse_seconds:
            raise ValueError(
                "short floor measured energized duration exceeds its pulse limit"
            )
        if evidence.get("autonomous_input_used") is not False:
            raise ValueError("short floor steering must be manual-only")
        if evidence.get("hardware_off_confirmed") is not True:
            raise ValueError("short floor test lacks confirmed hardware OFF state")
        if evidence.get("hardware_off_retry_required") is not False:
            raise ValueError("short floor test ended with a pending OFF retry")
        if evidence.get("watchdog_fault") is not None:
            raise ValueError("short floor test contains a watchdog fault")
        if evidence_integer(evidence, "watchdog_trip_count") != 0:
            raise ValueError("short floor test contains a watchdog trip")
        phases = evidence.get("phases")
        expected_names = ("STRAIGHT", "RIGHT_CORRECTION", "LEFT_CORRECTION")
        if not isinstance(phases, list) or [
            phase.get("name") if isinstance(phase, dict) else None
            for phase in phases
        ] != list(expected_names):
            raise ValueError("short floor evidence has unexpected phases")
        if any(
            phase.get("status") != "PASS"
            or phase.get("operator_confirmed") is not True
            for phase in phases
        ):
            raise ValueError("every short floor phase needs a confirmed PASS")
        phase_duties = []
        for phase in phases:
            left_duty = evidence_number(phase, "left_mapped_duty")
            right_duty = evidence_number(phase, "right_mapped_duty")
            if not 0.0 <= left_duty <= 1.0 or not 0.0 <= right_duty <= 1.0:
                raise ValueError("short floor mapped duties must be within 0..1")
            actual_duration = evidence_number(
                phase, "actual_energized_duration_s"
            )
            if not 0.0 < actual_duration <= pulse_seconds:
                raise ValueError(
                    "short floor phase energized duration exceeds its limit"
                )
            stopper = phase.get("deadline_stopper")
            if not isinstance(stopper, dict) or stopper.get("stop_error") is not None:
                raise ValueError("short floor deadline stopper is invalid")
            phase_duties.append((left_duty, right_duty))
        if not (
            phase_duties[1][0] > phase_duties[1][1]
            and phase_duties[2][1] > phase_duties[2][0]
        ):
            raise ValueError("short floor steering duty relationships are invalid")


def require_gate_prerequisites(gates, gate_name):
    missing = [
        prerequisite
        for prerequisite in GATE_PREREQUISITES.get(gate_name, ())
        if gates.get("gates", {}).get(prerequisite, {}).get("passed") is not True
    ]
    if missing:
        raise RuntimeError(
            "{} requires current prerequisite gates: {}".format(
                gate_name, ", ".join(missing)
            )
        )


def transitive_gate_dependents(gate_name):
    """Return every gate whose evidence depends on ``gate_name``."""
    dependents = set()
    changed = True
    while changed:
        changed = False
        for candidate, prerequisites in GATE_PREREQUISITES.items():
            if candidate == gate_name or candidate in dependents:
                continue
            if gate_name in prerequisites or any(
                prerequisite in dependents for prerequisite in prerequisites
            ):
                dependents.add(candidate)
                changed = True
    return [
        name for name in REQUIRED_MOTION_GATES if name in dependents
    ]


def record_gate(gates, gate_name, evidence_path):
    require_gate_prerequisites(gates, gate_name)
    evidence_path = os.path.abspath(evidence_path)
    if not os.path.isfile(evidence_path):
        raise RuntimeError("evidence file does not exist: {}".format(evidence_path))
    evidence = read_json_unverified(evidence_path)
    validate_evidence_payload(gate_name, evidence)
    artifact_hashes = hash_artifacts(GATE_ARTIFACTS[gate_name])
    if evidence.get("artifact_sha256_at_test") != artifact_hashes:
        raise RuntimeError(
            "current artifacts differ from the versions exercised by the test"
        )

    gate = gates["gates"][gate_name]
    gate.update(
        {
            "passed": True,
            "classification": GATE_CLASSIFICATIONS[gate_name],
            "evidence": evidence_path,
            "evidence_sha256": sha256_file(evidence_path),
            "artifact_sha256": artifact_hashes,
            "integrity_status": "CURRENT",
            "recorded_utc": utc_now(),
        }
    )
    gate.pop("integrity_errors", None)
    # Re-recording upstream physical or software evidence changes the basis on
    # which every later validation was accepted. Even if source-file hashes
    # are unchanged, old downstream observations are now prior evidence.
    for dependent_name in transitive_gate_dependents(gate_name):
        dependent = gates["gates"][dependent_name]
        if dependent.get("passed") is True:
            dependent["passed"] = False
            dependent["integrity_status"] = "STALE"
            dependent["integrity_errors"] = [
                "prerequisite gate re-recorded: {}".format(gate_name)
            ]
    save_gates(gates)


def ensure_evidence_directory():
    if not os.path.isdir(EVIDENCE_DIRECTORY):
        os.makedirs(EVIDENCE_DIRECTORY)


def write_generated_evidence(prefix, payload):
    ensure_evidence_directory()
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%S%fZ")
    path = os.path.join(
        EVIDENCE_DIRECTORY,
        "{}_{}.json".format(prefix, stamp),
    )
    atomic_json_write(path, payload)
    return path


def confirm_camera_orientation(gates):
    artifact_snapshot_start = capture_artifact_snapshot("camera_orientation")
    print("CAMERA ORIENTATION CONFIRMATION - NO MOTOR COMMANDS")
    print("First view the live flip-method=2 CSI feed with the robot stationary.")
    print("Only confirm if up/down and left/right match the physical scene.")
    response = input("Type UPRIGHT-FLIP-2 to confirm: ").strip()
    if response != "UPRIGHT-FLIP-2":
        raise RuntimeError("camera orientation was not confirmed")
    evidence = {
        "schema_version": 1,
        "test": EVIDENCE_RULES["camera_orientation"],
        "passed": True,
        "physical_motor_commands": False,
        "operator_confirmed_upright": True,
        "flip_method": 2,
        "confirmation_token": response,
        "completed_utc": utc_now(),
    }
    artifact_snapshot_end = capture_artifact_snapshot("camera_orientation")
    evidence["artifact_snapshot_stable"] = (
        artifact_snapshot_start == artifact_snapshot_end
    )
    evidence["artifact_sha256_at_test"] = artifact_snapshot_end
    if not evidence["artifact_snapshot_stable"]:
        raise RuntimeError("camera artifacts changed during confirmation")
    evidence_path = write_generated_evidence("camera_orientation", evidence)
    record_gate(gates, "camera_orientation", evidence_path)
    return evidence_path


def run_and_record_self_test(gates):
    script_path = project_path("integration_self_test.py")
    if not os.path.isfile(script_path):
        raise RuntimeError("integration_self_test.py is missing")
    artifact_snapshot_start = capture_artifact_snapshot(
        "motor_mapping_software"
    )
    command = [sys.executable, script_path]
    try:
        completed = subprocess.run(
            command,
            cwd=PROJECT_DIRECTORY,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            timeout=30.0,
        )
    except subprocess.TimeoutExpired:
        raise RuntimeError("integration self-test exceeded 30 seconds")
    output = completed.stdout or ""
    print(output.rstrip())
    marker = "INTEGRATION SELF-TEST: PASS"
    if completed.returncode != 0 or marker not in output:
        raise RuntimeError("integration self-test did not pass")
    evidence = {
        "schema_version": 1,
        "test": EVIDENCE_RULES["motor_mapping_software"],
        "passed": True,
        "physical_motor_commands": False,
        "self_test_return_code": completed.returncode,
        "pass_marker": marker,
        "command": command,
        "stdout": output,
        "completed_utc": utc_now(),
    }
    artifact_snapshot_end = capture_artifact_snapshot(
        "motor_mapping_software"
    )
    evidence["artifact_snapshot_stable"] = (
        artifact_snapshot_start == artifact_snapshot_end
    )
    evidence["artifact_sha256_at_test"] = artifact_snapshot_end
    if not evidence["artifact_snapshot_stable"]:
        raise RuntimeError("self-test artifacts changed while the test ran")
    evidence_path = write_generated_evidence("integration_self_test", evidence)
    record_gate(gates, "motor_mapping_software", evidence_path)
    return evidence_path


def record_ipm_calibration(gates):
    require_gate_prerequisites(gates, "ipm_physical_calibration")
    config = read_json_unverified(IPM_CONFIG_PATH)
    if config.get("calibration_state") != "PHYSICALLY_CALIBRATED":
        raise RuntimeError("ipm_config.json is not PHYSICALLY_CALIBRATED")
    evidence_path = config.get("calibration_evidence_json")
    if not evidence_path:
        raise RuntimeError(
            "ipm_config.json does not name calibration_evidence_json; "
            "repeat calibration with the current tool"
        )
    record_gate(
        gates,
        "ipm_physical_calibration",
        evidence_path,
    )


def main():
    parser = argparse.ArgumentParser(description="Integration evidence manager")
    subparsers = parser.add_subparsers(dest="command")
    subparsers.add_parser("show")
    subparsers.add_parser("verify")
    subparsers.add_parser("confirm-camera-orientation")
    subparsers.add_parser("record-self-test")
    subparsers.add_parser("record-ipm-calibration")
    record_parser = subparsers.add_parser("record")
    record_parser.add_argument("gate", choices=sorted(EVIDENCE_RULES.keys()))
    record_parser.add_argument("evidence_json")
    arguments = parser.parse_args()

    try:
        gates = read_json(GATES_PATH)
        if arguments.command is None or arguments.command == "show":
            print_status(gates)
            return 0
        if arguments.command == "verify":
            print_status(gates)
            return 0 if gates.get("physical_motion_authorized") else 1
        if arguments.command == "confirm-camera-orientation":
            evidence_path = confirm_camera_orientation(gates)
            print("Camera evidence: {}".format(evidence_path))
        elif arguments.command == "record-self-test":
            evidence_path = run_and_record_self_test(gates)
            print("Self-test evidence: {}".format(evidence_path))
        elif arguments.command == "record-ipm-calibration":
            record_ipm_calibration(gates)
        elif arguments.command == "record":
            record_gate(gates, arguments.gate, arguments.evidence_json)
        print_status(read_json(GATES_PATH))
        return 0
    except (IOError, OSError, RuntimeError, ValueError, KeyError) as error:
        print("REFUSED: {}".format(error))
        return 1


if __name__ == "__main__":
    sys.exit(main())
