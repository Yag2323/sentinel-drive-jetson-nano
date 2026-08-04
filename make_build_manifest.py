#!/usr/bin/env python3
"""Create a hash-addressed manifest for report and defence reproducibility."""

from __future__ import print_function

import datetime
import hashlib
import json
import os
import platform
import sys

from source_integrity import git_source_state, source_tree_sha256


BUILD_ID = "SENTINEL-INTEGRATION-2026.08.01-SINGLE-LINE3"
EXPECTED_YOLOV5_V6_COMMIT = "956be8e642b5c10af4a1533e09084ca32ff4f21f"
EXPECTED_YOLOV5N_SHA256 = "649e089f59b78ac021025de035b2d9c45dc26e544ea252955d0ffcefc1099e2f"
EXPECTED_YOLOV5S_SHA256 = "c3b140f32001a9eec4afa07120b3851eb1b6c2c7c7e7a4303af9eadfacbeb598"
PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
CORE_FILES = (
    "robot_dashboard.py",
    "patch_dashboard_camera_rotation.py",
    "gst_camera_bridge.py",
    "source_integrity.py",
    "prepare_yolov5_v6.py",
    "install_integration_bundle.py",
    "install_motor_calibration.py",
    "make_build_manifest.py",
    "jetson_perception_preflight.py",
    "yolov5_runtime.py",
    "yolo_csi_benchmark.py",
    "ipm_lane.py",
    "single_line_lane.py",
    "calibrate_ipm.py",
    "install_single_line_profile.py",
    "test_ipm_lane_synthetic.py",
    "test_single_line_synthetic.py",
    "ipm_alignment_diagnostic.py",
    "ipm_live_dry_run.py",
    "outer_circle_position_validation.py",
    "ipm_config.json",
    "control_core.py",
    "controller_config.json",
    "motor_mapping.py",
    "motor_config.json",
    "manual_motor_control.py",
    "safe_motor_output.py",
    "integrated_control_dry_run.py",
    "integration_self_test.py",
    "motor_adapter_raised_test.py",
    "steering_floor_validation.py",
    "validation_manager.py",
    "track_run.py",
    "analyse_track_run.py",
    "promote_controller_tuning.py",
    "integration_gates.json",
)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        while True:
            block = input_file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def main():
    files = []
    missing = []
    for name in CORE_FILES:
        path = os.path.join(PROJECT_DIRECTORY, name)
        if os.path.isfile(path):
            files.append(
                {
                    "path": name,
                    "size_bytes": os.path.getsize(path),
                    "sha256": sha256_file(path),
                }
            )
        else:
            missing.append(name)

    yolo_directory = os.path.join(PROJECT_DIRECTORY, "yolov5_v6")
    weights = os.path.join(yolo_directory, "yolov5n.pt")
    baseline_weights = os.path.join(yolo_directory, "yolov5s.pt")
    yolo_source = None
    yolo_git_state = None
    if os.path.isdir(yolo_directory):
        try:
            source_hash, source_count = source_tree_sha256(yolo_directory)
            yolo_source = {
                "sha256": source_hash,
                "file_count": source_count,
            }
            yolo_git_state = git_source_state(yolo_directory)
        except Exception as error:
            missing.append("yolov5_v6 source integrity: {}".format(error))
    else:
        missing.append("yolov5_v6")
    if not os.path.isfile(weights):
        missing.append("yolov5_v6/yolov5n.pt")
    if not os.path.isfile(baseline_weights):
        missing.append("yolov5_v6/yolov5s.pt")
    weights_hash = sha256_file(weights) if os.path.isfile(weights) else None
    baseline_weights_hash = (
        sha256_file(baseline_weights)
        if os.path.isfile(baseline_weights)
        else None
    )
    source_identity_valid = bool(
        yolo_git_state
        and yolo_git_state["commit"] == EXPECTED_YOLOV5_V6_COMMIT
        and yolo_git_state["source_tree_clean"]
        and yolo_source is not None
        and weights_hash == EXPECTED_YOLOV5N_SHA256
        and baseline_weights_hash == EXPECTED_YOLOV5S_SHA256
    )
    integrity_errors = []
    if yolo_git_state is not None:
        if yolo_git_state["commit"] != EXPECTED_YOLOV5_V6_COMMIT:
            integrity_errors.append("YOLO checkout is not the pinned v6.0 commit")
        if not yolo_git_state["source_tree_clean"]:
            integrity_errors.append(
                "YOLO source tree is modified or has untracked source"
            )
    if weights_hash is not None and weights_hash != EXPECTED_YOLOV5N_SHA256:
        integrity_errors.append("YOLOv5n weights are not the pinned v6.0 asset")
    if (
        baseline_weights_hash is not None
        and baseline_weights_hash != EXPECTED_YOLOV5S_SHA256
    ):
        integrity_errors.append("YOLOv5s weights are not the pinned v6.0 asset")

    result = {
        "schema_version": 2,
        "build_id": BUILD_ID,
        "created_utc": datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z",
        "platform": platform.platform(),
        "python": sys.version,
        "files": files,
        "missing_files": missing,
        "integrity_errors": integrity_errors,
        "build_complete": bool(not missing and source_identity_valid),
        "expected_yolov5_revision": EXPECTED_YOLOV5_V6_COMMIT,
        "expected_yolov5n_sha256": EXPECTED_YOLOV5N_SHA256,
        "expected_yolov5s_sha256": EXPECTED_YOLOV5S_SHA256,
        "yolov5_revision": (
            None if yolo_git_state is None else yolo_git_state["commit"]
        ),
        "yolov5_tracked_source_clean": bool(
            yolo_git_state and yolo_git_state["tracked_source_clean"]
        ),
        "yolov5_source_tree_clean": bool(
            yolo_git_state and yolo_git_state["source_tree_clean"]
        ),
        "yolov5_untracked_source_files": (
            []
            if yolo_git_state is None
            else yolo_git_state["untracked_source_files"]
        ),
        "yolov5_source_tree": yolo_source,
        "weights": None,
        "report_baseline_weights": None,
    }
    if os.path.isfile(weights):
        result["weights"] = {
            "path": os.path.relpath(weights, PROJECT_DIRECTORY),
            "size_bytes": os.path.getsize(weights),
            "sha256": weights_hash,
        }
    if os.path.isfile(baseline_weights):
        result["report_baseline_weights"] = {
            "path": os.path.relpath(baseline_weights, PROJECT_DIRECTORY),
            "size_bytes": os.path.getsize(baseline_weights),
            "sha256": baseline_weights_hash,
            "identity_valid": (
                baseline_weights_hash == EXPECTED_YOLOV5S_SHA256
            ),
            "role": "REPORT_ONLY_RAW_PYTORCH_BASELINE",
        }

    evidence_directory = os.path.join(PROJECT_DIRECTORY, "evidence")
    if not os.path.isdir(evidence_directory):
        os.makedirs(evidence_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    output_path = os.path.join(
        evidence_directory, "build_manifest_{}.json".format(stamp)
    )
    with open(output_path, "w") as output_file:
        json.dump(result, output_file, indent=2, sort_keys=True)
        output_file.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    print("Build manifest: {}".format(output_path))
    return 0 if not missing and source_identity_valid else 1


if __name__ == "__main__":
    sys.exit(main())
