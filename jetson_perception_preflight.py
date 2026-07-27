#!/usr/bin/env python3
"""Read-only compatibility audit for the existing Jetson perception stack."""

from __future__ import print_function

import datetime
import hashlib
import json
import os
import platform
import subprocess
import sys

from source_integrity import git_source_state, source_tree_sha256


PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
YOLO_DIRECTORY = os.path.join(PROJECT_DIRECTORY, "yolov5_v6")
WEIGHTS_PATH = os.path.join(YOLO_DIRECTORY, "yolov5n.pt")
YOLOV5S_WEIGHTS_PATH = os.path.join(YOLO_DIRECTORY, "yolov5s.pt")
EXPECTED_YOLOV5_V6_COMMIT = "956be8e642b5c10af4a1533e09084ca32ff4f21f"
EXPECTED_YOLOV5N_SHA256 = "649e089f59b78ac021025de035b2d9c45dc26e544ea252955d0ffcefc1099e2f"
EXPECTED_YOLOV5S_SHA256 = "c3b140f32001a9eec4afa07120b3851eb1b6c2c7c7e7a4303af9eadfacbeb598"


def utc_now():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        while True:
            block = input_file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def command_output(command, cwd=None):
    try:
        output = subprocess.check_output(
            command,
            cwd=cwd,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            timeout=8,
        )
        return True, output.strip()
    except subprocess.CalledProcessError as error:
        output = error.output or ""
        if not isinstance(output, str):
            output = output.decode("utf-8", "replace")
        return False, "exit_code={} output={}".format(
            error.returncode,
            output.strip() or "no command output",
        )
    except Exception as error:
        return False, repr(error)


def required_checks_pass(checks):
    """Return true when every operationally required check passed."""
    return all(item["passed"] or not item["required"] for item in checks)


def main():
    checks = []
    source_tree_hash = None
    source_file_count = 0
    git_state = None
    opencv_gstreamer_enabled = None

    def record(name, passed, evidence, required=True):
        checks.append(
            {
                "name": name,
                "passed": bool(passed),
                "required": bool(required),
                "evidence": str(evidence),
            }
        )

    record("python_3_6_or_newer", sys.version_info >= (3, 6), sys.version)
    record("yolov5_v6_directory", os.path.isdir(YOLO_DIRECTORY), YOLO_DIRECTORY)
    record("yolov5n_weights", os.path.isfile(WEIGHTS_PATH), WEIGHTS_PATH)
    model_file = os.path.join(YOLO_DIRECTORY, "models", "experimental.py")
    record("yolov5_v6_model_api", os.path.isfile(model_file), model_file)

    weights_sha256 = None
    yolov5s_sha256 = None
    if os.path.isfile(WEIGHTS_PATH):
        weights_sha256 = sha256_file(WEIGHTS_PATH)
        record(
            "weights_identity_recorded",
            weights_sha256 == EXPECTED_YOLOV5N_SHA256,
            "expected={} actual={} bytes={}".format(
                EXPECTED_YOLOV5N_SHA256,
                weights_sha256,
                os.path.getsize(WEIGHTS_PATH),
            ),
        )

    if os.path.isfile(YOLOV5S_WEIGHTS_PATH):
        yolov5s_sha256 = sha256_file(YOLOV5S_WEIGHTS_PATH)

    if os.path.isdir(YOLO_DIRECTORY):
        git_state = git_source_state(YOLO_DIRECTORY)
        record(
            "yolov5_v6_pinned_commit",
            git_state["commit"] == EXPECTED_YOLOV5_V6_COMMIT,
            "expected={} actual={}".format(
                EXPECTED_YOLOV5_V6_COMMIT,
                git_state["commit"]
                or "unavailable: {}".format(git_state["commit_error"]),
            ),
        )
        record(
            "yolov5_tracked_source_clean",
            git_state["tracked_source_clean"],
            git_state["tracked_status"]
            or git_state["tracked_status_error"]
            or "clean",
        )
        record(
            "yolov5_no_untracked_source",
            git_state["source_tree_clean"],
            ", ".join(git_state["untracked_source_files"])
            or git_state["untracked_source_status_error"]
            or "no untracked source files",
        )
        try:
            source_tree_hash, source_file_count = source_tree_sha256(
                YOLO_DIRECTORY
            )
            record(
                "yolov5_source_tree_recorded",
                source_file_count > 0,
                "sha256={} files={}".format(
                    source_tree_hash, source_file_count
                ),
            )
        except Exception as error:
            record("yolov5_source_tree_recorded", False, repr(error))

    l4t_release_path = "/etc/nv_tegra_release"
    try:
        with open(l4t_release_path, "r") as release_file:
            l4t_release = release_file.readline().strip()
        record("l4t_release", l4t_release.startswith("# R32"), l4t_release)
    except Exception as error:
        record("l4t_release", False, repr(error))

    for check_name, command in (
        ("nvpmodel_query", ["nvpmodel", "-q", "--verbose"]),
        ("jetson_clocks_query", ["jetson_clocks", "--show"]),
    ):
        command_ok, output = command_output(command)
        # These commands record performance context; they are not inference
        # dependencies. On L4T R32.7.x a non-root ``jetson_clocks --show`` can
        # return exit 1 solely because an EMC-cap sysfs node is unreadable.
        # Preserve that warning in evidence without hiding failures in CUDA,
        # the pinned source/weights, imports, or the CSI GStreamer plugins.
        record(check_name, command_ok, output[-2000:], required=False)

    try:
        import cv2

        record("opencv_import", True, cv2.__version__)
        try:
            build_information = cv2.getBuildInformation()
            opencv_gstreamer_enabled = any(
                "GStreamer" in line and "YES" in line.upper()
                for line in build_information.splitlines()
            )
            record(
                "opencv_gstreamer_capability_observed",
                True,
                "enabled={}; CSI uses gst_camera_bridge/GI regardless".format(
                    opencv_gstreamer_enabled
                ),
            )
        except Exception as build_error:
            record(
                "opencv_gstreamer_capability_observed",
                True,
                "unknown={!r}; CSI uses gst_camera_bridge/GI".format(
                    build_error
                ),
            )
    except Exception as error:
        record("opencv_import", False, repr(error))

    try:
        import numpy

        record("numpy_import", True, numpy.__version__)
    except Exception as error:
        record("numpy_import", False, repr(error))

    try:
        import torch

        record("torch_import", True, torch.__version__)
        record("cuda_available", torch.cuda.is_available(), torch.version.cuda)
        if torch.cuda.is_available():
            record("cuda_device", True, torch.cuda.get_device_name(0))
    except Exception as error:
        record("torch_import", False, repr(error))
        record("cuda_available", False, "torch import failed")

    try:
        import gi

        gi.require_version("Gst", "1.0")
        gi.require_version("GstApp", "1.0")
        from gi.repository import Gst, GstApp  # noqa: F401

        Gst.init(None)
        plugins = (
            "nvarguscamerasrc",
            "nvvidconv",
            "videoconvert",
            "appsink",
        )
        missing_plugins = [
            name for name in plugins if Gst.ElementFactory.find(name) is None
        ]
        record(
            "gstreamer_csi_plugins",
            not missing_plugins,
            "missing: {}".format(", ".join(missing_plugins))
            if missing_plugins
            else "all required plugins present",
        )
    except Exception as error:
        record("gstreamer_csi_plugins", False, repr(error))

    if os.path.isdir(YOLO_DIRECTORY):
        try:
            if YOLO_DIRECTORY not in sys.path:
                sys.path.insert(0, YOLO_DIRECTORY)
            from models.experimental import attempt_load  # noqa: F401
            from utils.datasets import letterbox  # noqa: F401
            from utils.general import (  # noqa: F401
                check_img_size,
                non_max_suppression,
                scale_coords,
            )

            record("yolov5_v6_imports", True, "v6 API imports succeeded")
        except Exception as error:
            record("yolov5_v6_imports", False, repr(error))

    passed = required_checks_pass(checks)
    warning_count = sum(
        1 for item in checks if not item["required"] and not item["passed"]
    )
    result = {
        "schema_version": 2,
        "test": "JETSON_PERCEPTION_PREFLIGHT",
        "passed": passed,
        "timestamp_utc": utc_now(),
        "platform": platform.platform(),
        "opencv_gstreamer_enabled": opencv_gstreamer_enabled,
        "yolov5_repository_commit_expected": EXPECTED_YOLOV5_V6_COMMIT,
        "weights_path": WEIGHTS_PATH,
        "weights_sha256": weights_sha256,
        "expected_weights_sha256": EXPECTED_YOLOV5N_SHA256,
        "optional_report_baseline": {
            "model": "yolov5s",
            "path": YOLOV5S_WEIGHTS_PATH,
            "present": os.path.isfile(YOLOV5S_WEIGHTS_PATH),
            "sha256": yolov5s_sha256,
            "expected_sha256": EXPECTED_YOLOV5S_SHA256,
            "identity_valid": yolov5s_sha256 == EXPECTED_YOLOV5S_SHA256,
            "motion_gate_eligible": False,
        },
        "yolov5_tracked_source_clean": bool(
            git_state and git_state["tracked_source_clean"]
        ),
        "yolov5_source_tree_clean": bool(
            git_state and git_state["source_tree_clean"]
        ),
        "yolov5_untracked_source_files": (
            [] if git_state is None else git_state["untracked_source_files"]
        ),
        "yolov5_source_tree_sha256": source_tree_hash,
        "yolov5_source_file_count": source_file_count,
        "checks": checks,
        "optional_diagnostic_warning_count": warning_count,
        "dependency_policy": (
            "PRESERVE_NVIDIA_CUDA_TORCH_AND_SYSTEM_OPENCV; "
            "DO_NOT_INSTALL_CURRENT_YOLOV5_REQUIREMENTS"
        ),
    }

    evidence_directory = os.path.join(PROJECT_DIRECTORY, "evidence")
    if not os.path.isdir(evidence_directory):
        os.makedirs(evidence_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    output_path = os.path.join(
        evidence_directory, "perception_preflight_{}.json".format(stamp)
    )
    with open(output_path, "w") as output_file:
        json.dump(result, output_file, indent=2, sort_keys=True)
        output_file.write("\n")

    for item in checks:
        if item["passed"]:
            status = "PASS"
        elif item["required"]:
            status = "FAIL"
        else:
            status = "WARN"
        print("{} {:28s} {}".format(
            status,
            item["name"],
            item["evidence"],
        ))
    print("Evidence: {}".format(output_path))
    print("PREFLIGHT: {}".format("PASS" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
