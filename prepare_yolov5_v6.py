#!/usr/bin/env python3
"""Provision pinned YOLOv5 v6.0 source and verified n/s weights safely.

This helper deliberately does not install requirements because replacing the
JetPack-provided CUDA PyTorch/OpenCV stack can break GPU support on a Nano.
Existing, unexpected checkouts or weights are never overwritten.
Compatible with Python 3.6.
"""

from __future__ import print_function

import argparse
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
try:
    from urllib.request import urlopen
except ImportError:  # pragma: no cover
    from urllib2 import urlopen

from source_integrity import git_source_state, source_tree_sha256


PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
YOLO_DIRECTORY = os.path.join(PROJECT_DIRECTORY, "yolov5_v6")
WEIGHTS_PATH = os.path.join(YOLO_DIRECTORY, "yolov5n.pt")
YOLOV5S_WEIGHTS_PATH = os.path.join(YOLO_DIRECTORY, "yolov5s.pt")
REPOSITORY_URL = "https://github.com/ultralytics/yolov5.git"
RELEASE_TAG = "v6.0"
EXPECTED_COMMIT = "956be8e642b5c10af4a1533e09084ca32ff4f21f"
WEIGHTS_URL = (
    "https://github.com/ultralytics/yolov5/releases/download/"
    "v6.0/yolov5n.pt"
)
EXPECTED_WEIGHTS_SHA256 = (
    "649e089f59b78ac021025de035b2d9c45dc26e544ea252955d0ffcefc1099e2f"
)
EXPECTED_WEIGHTS_BYTES = 3952441
YOLOV5S_WEIGHTS_URL = (
    "https://github.com/ultralytics/yolov5/releases/download/"
    "v6.0/yolov5s.pt"
)
EXPECTED_YOLOV5S_SHA256 = (
    "c3b140f32001a9eec4afa07120b3851eb1b6c2c7c7e7a4303af9eadfacbeb598"
)
EXPECTED_YOLOV5S_BYTES = 14698491


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


def remove_created_directory(path):
    project = os.path.realpath(PROJECT_DIRECTORY)
    target = os.path.realpath(path)
    if os.path.dirname(target) != project or not os.path.basename(target).startswith(
        ".yolov5_v6."
    ):
        raise RuntimeError("Refusing unsafe temporary-directory cleanup")
    if os.path.isdir(target):
        shutil.rmtree(target)


def clone_if_missing():
    if os.path.exists(YOLO_DIRECTORY):
        if not os.path.isdir(YOLO_DIRECTORY):
            raise RuntimeError("yolov5_v6 exists but is not a directory")
        print("PRESERVED existing YOLO directory: {}".format(YOLO_DIRECTORY))
        return False

    temporary = tempfile.mkdtemp(prefix=".yolov5_v6.", dir=PROJECT_DIRECTORY)
    try:
        subprocess.check_call(
            [
                "git",
                "clone",
                "--branch",
                RELEASE_TAG,
                "--depth",
                "1",
                REPOSITORY_URL,
                temporary,
            ]
        )
        state = git_source_state(temporary)
        if state["commit"] != EXPECTED_COMMIT or not state["source_tree_clean"]:
            raise RuntimeError(
                "Downloaded YOLO source did not match the pinned clean v6.0 tree"
            )
        os.replace(temporary, YOLO_DIRECTORY)
        print("INSTALLED pinned YOLOv5 v6.0 source: {}".format(YOLO_DIRECTORY))
        return True
    except Exception:
        remove_created_directory(temporary)
        raise


def verify_source():
    state = git_source_state(YOLO_DIRECTORY)
    if state["commit"] != EXPECTED_COMMIT:
        raise RuntimeError(
            "Existing yolov5_v6 is not commit {} (actual {}). Move it to a "
            "backup name and rerun this helper; it will not overwrite it."
            .format(EXPECTED_COMMIT, state["commit"] or "unavailable")
        )
    if not state["source_tree_clean"]:
        raise RuntimeError(
            "Existing yolov5_v6 has modified or untracked source files. "
            "Tracked status={!r}; untracked source={!r}. Preserve it as a "
            "backup and provision a clean tree."
            .format(
                state["tracked_status"],
                state["untracked_source_files"],
            )
        )
    source_hash, source_count = source_tree_sha256(YOLO_DIRECTORY)
    return state, source_hash, source_count


def download_weights_if_missing(
    weights_path,
    weights_url,
    expected_sha256,
    expected_bytes,
    model_label,
):
    if os.path.isfile(weights_path):
        actual = sha256_file(weights_path)
        actual_bytes = os.path.getsize(weights_path)
        if actual != expected_sha256 or actual_bytes != expected_bytes:
            raise RuntimeError(
                "Existing {} has bytes={} SHA-256={}, expected bytes={} "
                "SHA-256={}. It was not overwritten.".format(
                    os.path.basename(weights_path),
                    actual_bytes,
                    actual,
                    expected_bytes,
                    expected_sha256,
                )
            )
        print("PRESERVED verified {} weights: {}".format(
            model_label, weights_path
        ))
        return False
    if os.path.exists(weights_path):
        raise RuntimeError("{} exists but is not a regular file".format(
            os.path.basename(weights_path)
        ))

    temporary = weights_path + ".download"
    if os.path.exists(temporary):
        raise RuntimeError(
            "Temporary download already exists; inspect and remove it first: {}"
            .format(temporary)
        )
    digest = hashlib.sha256()
    try:
        with urlopen(weights_url, timeout=60) as response:
            byte_count = 0
            with open(temporary, "wb") as output_file:
                while True:
                    block = response.read(1024 * 1024)
                    if not block:
                        break
                    output_file.write(block)
                    digest.update(block)
                    byte_count += len(block)
                output_file.flush()
                os.fsync(output_file.fileno())
        actual = digest.hexdigest()
        if actual != expected_sha256 or byte_count != expected_bytes:
            raise RuntimeError(
                "Downloaded {} failed identity verification: bytes={} "
                "SHA-256={}".format(
                    os.path.basename(weights_path), byte_count, actual
                )
            )
        os.replace(temporary, weights_path)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise
    print("INSTALLED verified {} weights: {}".format(
        model_label, weights_path
    ))
    return True


def write_evidence(payload):
    evidence_directory = os.path.join(PROJECT_DIRECTORY, "evidence")
    if not os.path.isdir(evidence_directory):
        os.makedirs(evidence_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    path = os.path.join(
        evidence_directory, "yolov5_provisioning_{}.json".format(stamp)
    )
    with open(path, "w") as output_file:
        json.dump(payload, output_file, indent=2, sort_keys=True)
        output_file.write("\n")
    return path


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Provision the pinned YOLOv5 v6.0 source plus verified "
            "YOLOv5n integration and YOLOv5s report-baseline weights."
        )
    )
    parser.parse_args()
    try:
        clone_if_missing()
        state, source_hash, source_count = verify_source()
        download_weights_if_missing(
            WEIGHTS_PATH,
            WEIGHTS_URL,
            EXPECTED_WEIGHTS_SHA256,
            EXPECTED_WEIGHTS_BYTES,
            "YOLOv5n",
        )
        download_weights_if_missing(
            YOLOV5S_WEIGHTS_PATH,
            YOLOV5S_WEIGHTS_URL,
            EXPECTED_YOLOV5S_SHA256,
            EXPECTED_YOLOV5S_BYTES,
            "YOLOv5s",
        )
        weights_hash = sha256_file(WEIGHTS_PATH)
        yolov5s_hash = sha256_file(YOLOV5S_WEIGHTS_PATH)
        payload = {
            "schema_version": 2,
            "test": "PINNED_YOLOV5_V6_PROVISIONING",
            "passed": True,
            "repository_url": REPOSITORY_URL,
            "release_tag": RELEASE_TAG,
            "commit": state["commit"],
            "source_tree_clean": state["source_tree_clean"],
            "source_tree_sha256": source_hash,
            "source_file_count": source_count,
            "weights_url": WEIGHTS_URL,
            "weights_sha256": weights_hash,
            "model_assets": {
                "yolov5n": {
                    "role": "INTEGRATION_AND_MOTION_GATE",
                    "path": WEIGHTS_PATH,
                    "url": WEIGHTS_URL,
                    "size_bytes": os.path.getsize(WEIGHTS_PATH),
                    "sha256": weights_hash,
                },
                "yolov5s": {
                    "role": "REPORT_ONLY_RAW_PYTORCH_BASELINE",
                    "path": YOLOV5S_WEIGHTS_PATH,
                    "url": YOLOV5S_WEIGHTS_URL,
                    "size_bytes": os.path.getsize(YOLOV5S_WEIGHTS_PATH),
                    "sha256": yolov5s_hash,
                },
            },
            "requirements_installed": False,
            "completed_utc": utc_now(),
        }
        evidence_path = write_evidence(payload)
        print(json.dumps(payload, indent=2, sort_keys=True))
        print("Provisioning evidence: {}".format(evidence_path))
        print("YOLOV5 V6 PROVISIONING: PASS")
        return 0
    except (IOError, OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print("PROVISIONING REFUSED: {}".format(error))
        print("No pip requirements were installed and no existing file was overwritten.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
