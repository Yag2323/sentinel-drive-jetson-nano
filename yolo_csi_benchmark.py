#!/usr/bin/env python3
"""Measure pinned YOLOv5n/s performance on the corrected real CSI feed."""

from __future__ import print_function

import argparse
import csv
import datetime
import hashlib
import json
import os
import statistics
import sys
import time

import cv2

from gst_camera_bridge import DEFAULT_CSI_FLIP_METHOD, GstCamera
from source_integrity import git_source_state, source_tree_sha256
from validation_manager import capture_artifact_snapshot


EXPECTED_YOLOV5_V6_COMMIT = "956be8e642b5c10af4a1533e09084ca32ff4f21f"
EXPECTED_YOLOV5N_SHA256 = "649e089f59b78ac021025de035b2d9c45dc26e544ea252955d0ffcefc1099e2f"
EXPECTED_YOLOV5S_SHA256 = "c3b140f32001a9eec4afa07120b3851eb1b6c2c7c7e7a4303af9eadfacbeb598"
PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
YOLO_DIRECTORY = os.path.join(PROJECT_DIRECTORY, "yolov5_v6")
MODEL_SPECS = {
    "yolov5n": {
        "weights_path": os.path.join(YOLO_DIRECTORY, "yolov5n.pt"),
        "expected_sha256": EXPECTED_YOLOV5N_SHA256,
        "test": "YOLOV5N_REAL_CSI_BENCHMARK",
        "minimum_frames": 50,
        "motion_gate_eligible": True,
        "role": "INTEGRATION_AND_MOTION_GATE",
    },
    "yolov5s": {
        "weights_path": os.path.join(YOLO_DIRECTORY, "yolov5s.pt"),
        "expected_sha256": EXPECTED_YOLOV5S_SHA256,
        "test": "YOLOV5S_REAL_CSI_BASELINE",
        "minimum_frames": 20,
        "motion_gate_eligible": False,
        "role": "REPORT_ONLY_RAW_PYTORCH_BASELINE",
    },
}


def utc_now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * float(fraction)))
    return float(ordered[index])


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
    parser = argparse.ArgumentParser(description="CSI YOLOv5 n/s benchmark")
    parser.add_argument(
        "--model",
        choices=sorted(MODEL_SPECS.keys()),
        default="yolov5n",
        help="yolov5n is the integration gate; yolov5s is report-only.",
    )
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--img-size", type=int, default=320)
    parser.add_argument("--confidence", type=float, default=0.55)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument(
        "--camera-fps",
        type=int,
        choices=(21, 30),
        default=None,
        help=(
            "CSI source rate. Default: 30 Hz for the report-only yolov5s "
            "baseline and 21 Hz for the operational yolov5n gate."
        ),
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--allow-cpu",
        action="store_true",
        help="Permit a diagnostic CPU run; not valid GPU evidence.",
    )
    arguments = parser.parse_args()
    model_spec = MODEL_SPECS[arguments.model]
    camera_fps = (
        int(arguments.camera_fps)
        if arguments.camera_fps is not None
        else (30 if arguments.model == "yolov5s" else 21)
    )
    weights_path = model_spec["weights_path"]
    expected_weights_sha256 = model_spec["expected_sha256"]
    if not 5.0 <= arguments.seconds <= 300.0:
        print("REFUSED: --seconds must be between 5 and 300.")
        return 1
    if not 0 <= arguments.warmup_frames <= 60:
        print("REFUSED: --warmup-frames must be between 0 and 60.")
        return 1

    if not os.path.isfile(weights_path):
        print("REFUSED: {} weights are missing: {}".format(
            arguments.model, weights_path
        ))
        return 1
    try:
        source_hash_before, source_file_count_before = source_tree_sha256(
            YOLO_DIRECTORY
        )
    except Exception as error:
        print("REFUSED: YOLO source integrity failed: {}".format(error))
        return 1
    git_state_before = git_source_state(YOLO_DIRECTORY)
    if git_state_before["commit"] != EXPECTED_YOLOV5_V6_COMMIT:
        print(
            "REFUSED: YOLO checkout is not the pinned v6.0 commit. "
            "Expected {}, received {}.".format(
                EXPECTED_YOLOV5_V6_COMMIT,
                git_state_before["commit"] or "unavailable",
            )
        )
        return 1
    if not git_state_before["source_tree_clean"]:
        print(
            "REFUSED: yolov5_v6 source is modified or contains untracked "
            "source files: {} {}".format(
                git_state_before["tracked_status"]
                or git_state_before["tracked_status_error"]
                or "tracked files clean;",
                ", ".join(git_state_before["untracked_source_files"])
                or git_state_before["untracked_source_status_error"]
                or "no untracked source",
            )
        )
        return 1
    weights_hash_before = sha256_file(weights_path)
    if weights_hash_before != expected_weights_sha256:
        print(
            "REFUSED: {} is not the pinned official v6.0 weight. "
            "Expected {}, received {}.".format(
                os.path.basename(weights_path),
                expected_weights_sha256,
                weights_hash_before,
            )
        )
        return 1

    artifact_snapshot_start = capture_artifact_snapshot(
        "yolo_current_regression"
    )

    # Import only after the pinned/clean source check. This deliberately avoids
    # importing an unverified YOLO tree during a benchmark intended as evidence.
    from yolov5_runtime import (
        YoloV5Detector,
        draw_detections,
        format_detections,
    )

    evidence_directory = os.path.join(PROJECT_DIRECTORY, "evidence")
    if not os.path.isdir(evidence_directory):
        os.makedirs(evidence_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    file_prefix = "{}_csi_{}".format(arguments.model, stamp)
    csv_path = os.path.join(evidence_directory, file_prefix + ".csv")
    json_path = os.path.join(evidence_directory, file_prefix + ".json")
    image_path = os.path.join(evidence_directory, file_prefix + ".jpg")

    detector = YoloV5Detector(
        weights_path=weights_path,
        image_size=arguments.img_size,
        confidence=arguments.confidence,
        iou=arguments.iou,
    )
    if detector.device.type != "cuda" and not arguments.allow_cpu:
        print("REFUSED: CUDA is unavailable. Use --allow-cpu only for diagnosis.")
        return 1

    camera = GstCamera(
        sensor_id=0,
        capture_width=1280,
        capture_height=720,
        output_width=640,
        output_height=360,
        framerate=camera_fps,
    )
    csv_file = open(csv_path, "w", newline="")
    fields = (
        "timestamp_utc",
        "monotonic_s",
        "frame",
        "camera_sequence",
        "camera_pts_ns",
        "camera_age_ms",
        "camera_fresh",
        "tensor_height",
        "tensor_width",
        "capture_ms",
        "preprocess_ms",
        "inference_ms",
        "nms_ms",
        "detector_end_to_end_ms",
        "source_to_result_ms",
        "loop_ms",
        "detection_count",
        "objects",
    )
    writer = csv.DictWriter(csv_file, fieldnames=fields)
    writer.writeheader()

    frame_number = 0
    inference_values = []
    loop_values = []
    preprocess_values = []
    nms_values = []
    detector_values = []
    source_to_result_values = []
    tensor_shapes = set()
    capture_values = []
    latest_overlay = None
    start = None
    measurement_end = None
    error_text = None

    print("{} CSI BENCHMARK - MOTORS NOT IMPORTED".format(
        arguments.model.upper()
    ))
    print("Role: {}".format(model_spec["role"]))
    print("Camera correction: nvvidconv flip-method={}".format(
        DEFAULT_CSI_FLIP_METHOD
    ))
    print("Image size: {} | duration: {:.1f}s | CSI source: {} Hz".format(
        detector.image_size, arguments.seconds, camera_fps
    ))
    try:
        for warmup_index in range(arguments.warmup_frames):
            success, warmup_frame, camera_metadata = camera.read_with_metadata(
                timeout_seconds=2.0
            )
            if (
                not success
                or camera_metadata is None
                or not camera_metadata["timestamp_valid"]
                or not camera_metadata["source_age_trustworthy"]
                or not camera_metadata["fresh"]
            ):
                raise RuntimeError("CSI warm-up frame unavailable or stale.")
            detector.infer(warmup_frame)
            if (warmup_index + 1) % 10 == 0:
                print("Warm-up {}/{}".format(
                    warmup_index + 1, arguments.warmup_frames
                ))

        start = time.monotonic()
        while time.monotonic() - start < arguments.seconds:
            loop_start = time.monotonic()
            capture_start = time.monotonic()
            success, frame, camera_metadata = camera.read_with_metadata(
                timeout_seconds=2.0
            )
            capture_ms = (time.monotonic() - capture_start) * 1000.0
            if not success:
                raise RuntimeError("CSI frame unavailable.")
            if (
                camera_metadata is None
                or not camera_metadata["timestamp_valid"]
                or not camera_metadata["source_age_trustworthy"]
                or not camera_metadata["fresh"]
            ):
                raise RuntimeError("CSI timestamp is invalid or repeated.")
            camera_age = time.monotonic() - camera_metadata["capture_monotonic"]
            if camera_age > 0.25:
                raise RuntimeError(
                    "CSI frame is stale: {:.3f} seconds.".format(camera_age)
                )

            result = detector.infer(frame)
            source_to_result_ms = (
                time.monotonic() - camera_metadata["capture_monotonic"]
            ) * 1000.0
            loop_ms = (time.monotonic() - loop_start) * 1000.0
            frame_number += 1
            capture_values.append(capture_ms)
            preprocess_values.append(float(result["preprocess_ms"]))
            inference_values.append(float(result["inference_ms"]))
            nms_values.append(float(result["nms_ms"]))
            detector_values.append(float(result["end_to_end_ms"]))
            source_to_result_values.append(source_to_result_ms)
            tensor_shape = tuple(result["input_tensor_shape"])
            tensor_shapes.add(tensor_shape)
            loop_values.append(loop_ms)
            writer.writerow(
                {
                    "timestamp_utc": utc_now(),
                    "monotonic_s": round(time.monotonic(), 6),
                    "frame": frame_number,
                    "camera_sequence": camera_metadata["sequence"],
                    "camera_pts_ns": camera_metadata["pts_ns"],
                    "camera_age_ms": round(camera_age * 1000.0, 3),
                    "camera_fresh": 1,
                    "tensor_height": tensor_shape[2],
                    "tensor_width": tensor_shape[3],
                    "capture_ms": round(capture_ms, 3),
                    "preprocess_ms": round(result["preprocess_ms"], 3),
                    "inference_ms": round(result["inference_ms"], 3),
                    "nms_ms": round(result["nms_ms"], 3),
                    "detector_end_to_end_ms": round(result["end_to_end_ms"], 3),
                    "source_to_result_ms": round(source_to_result_ms, 3),
                    "loop_ms": round(loop_ms, 3),
                    "detection_count": len(result["detections"]),
                    "objects": format_detections(result["detections"]),
                }
            )
            if frame_number % 10 == 0:
                csv_file.flush()
                print(
                    "Frame {} | inference {:.1f} ms | capture+detector {:.2f} FPS".format(
                        frame_number,
                        result["inference_ms"],
                        1000.0 / loop_ms if loop_ms > 0 else 0.0,
                    )
                )

            latest_overlay = frame.copy()
            draw_detections(
                latest_overlay, result["detections"], result["corridor"]
            )
            cv2.putText(
                latest_overlay,
                "capture+detector {:.2f} FPS | {:.1f} ms model".format(
                    1000.0 / loop_ms if loop_ms > 0 else 0.0,
                    result["inference_ms"],
                ),
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 255),
                2,
            )
            if not arguments.headless:
                cv2.imshow(
                    "{} CSI benchmark".format(arguments.model),
                    latest_overlay,
                )
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    error_text = "OPERATOR_ENDED_BEFORE_FULL_WINDOW"
                    break
    except KeyboardInterrupt:
        error_text = "INTERRUPTED_BY_OPERATOR"
        print("Interrupted.")
    except Exception as error:
        error_text = repr(error)
        print("BENCHMARK STOPPED: {}".format(error))
    finally:
        measurement_end = time.monotonic()
        camera.close()
        csv_file.close()
        cv2.destroyAllWindows()

    elapsed = (
        max(0.000001, measurement_end - start)
        if start is not None
        else 0.0
    )
    if latest_overlay is not None:
        cv2.imwrite(image_path, latest_overlay)
    # Do not launch Git subprocesses after CUDA/GStreamer have initialized.
    # On memory-constrained Jetson Nano systems those subprocesses can time out
    # even though the checkout is unchanged.  Git identity/cleanliness was
    # verified before importing the detector.  Here we verify the exact source
    # and weight bytes again; validation_manager.py performs a fresh Git check
    # in its own process before it accepts motion-gate evidence.
    try:
        source_hash_after, source_file_count_after = source_tree_sha256(
            YOLO_DIRECTORY
        )
        weights_hash_after = sha256_file(weights_path)
    except Exception as integrity_error:
        source_hash_after = None
        source_file_count_after = 0
        weights_hash_after = None
        if error_text is None:
            error_text = "SOURCE_INTEGRITY_ERROR: {!r}".format(
                integrity_error
            )
    pre_measurement_git_verified = bool(
        git_state_before.get("commit") == EXPECTED_YOLOV5_V6_COMMIT
        and git_state_before.get("source_tree_clean") is True
    )
    byte_integrity_stable = bool(
        source_hash_before == source_hash_after
        and source_file_count_before == source_file_count_after
        and weights_hash_before == weights_hash_after
    )
    source_integrity_stable = bool(
        pre_measurement_git_verified and byte_integrity_stable
    )
    artifact_snapshot_end = capture_artifact_snapshot(
        "yolo_current_regression"
    )
    artifact_snapshot_stable = (
        artifact_snapshot_start == artifact_snapshot_end
    )
    measurement_window_completed = bool(
        start is not None and elapsed >= float(arguments.seconds)
    )
    summary = {
        "schema_version": 1,
        "test": model_spec["test"],
        "passed": (
            error_text is None
            and measurement_window_completed
            and frame_number >= model_spec["minimum_frames"]
            and detector.device.type == "cuda"
            and source_integrity_stable
            and artifact_snapshot_stable
        ),
        "error": error_text,
        "motor_commands": False,
        "model_variant": arguments.model,
        "model_role": model_spec["role"],
        "motion_gate_eligible": bool(model_spec["motion_gate_eligible"]),
        "camera_flip_method": DEFAULT_CSI_FLIP_METHOD,
        "camera_capture_fps_configured": camera_fps,
        "camera_capture_width": 1280,
        "camera_capture_height": 720,
        "camera_output_width": 640,
        "camera_output_height": 360,
        "camera_pipeline": camera.pipeline_description,
        "device": str(detector.device),
        "fp16": bool(detector.use_half),
        "image_size": int(detector.image_size),
        "confidence_threshold": float(arguments.confidence),
        "iou_threshold": float(arguments.iou),
        "observed_input_tensor_shapes": [
            list(shape) for shape in sorted(tensor_shapes)
        ],
        "warmup_frames": arguments.warmup_frames,
        "weights_path": weights_path,
        "weights_sha256": weights_hash_after,
        "expected_weights_sha256": expected_weights_sha256,
        "weights_sha256_before_measurement": weights_hash_before,
        "yolov5_repository_commit": git_state_before.get("commit"),
        "expected_yolov5_repository_commit": EXPECTED_YOLOV5_V6_COMMIT,
        "yolov5_tracked_source_clean": bool(
            git_state_before.get("tracked_source_clean")
        ),
        "yolov5_source_tree_clean": bool(
            git_state_before.get("source_tree_clean")
        ),
        "yolov5_untracked_source_files": git_state_before.get(
            "untracked_source_files", []
        ),
        "yolov5_tracked_status": git_state_before.get("tracked_status"),
        "git_identity_check_phase": "PRE_MEASUREMENT_BEFORE_CUDA_IMPORT",
        "gate_recorder_requires_fresh_git_identity_check": True,
        "source_integrity_basis": (
            "Pinned clean Git identity before CUDA import; deterministic "
            "source-tree and weight SHA-256 values unchanged after the full "
            "measurement; validation_manager performs an independent current "
            "Git check before accepting gate evidence."
        ),
        "yolov5_source_tree_sha256": source_hash_after,
        "yolov5_source_tree_sha256_before_measurement": source_hash_before,
        "yolov5_source_file_count": source_file_count_after,
        "source_and_weights_bytes_stable_during_measurement": (
            byte_integrity_stable
        ),
        "source_integrity_stable_during_measurement": source_integrity_stable,
        "artifact_snapshot_stable": artifact_snapshot_stable,
        "artifact_sha256_at_test": artifact_snapshot_end,
        "frames": frame_number,
        "measurement_elapsed_s": elapsed,
        "requested_measurement_window_s": float(arguments.seconds),
        "measurement_window_completed": measurement_window_completed,
        "complete_measurement_window_fps": (
            frame_number / elapsed if elapsed > 0.0 else 0.0
        ),
        "mean_capture_plus_detector_ms": statistics.mean(loop_values) if loop_values else None,
        "median_capture_plus_detector_ms": statistics.median(loop_values) if loop_values else None,
        "p95_capture_plus_detector_ms": percentile(loop_values, 0.95),
        "mean_capture_ms": statistics.mean(capture_values) if capture_values else None,
        "mean_preprocess_ms": statistics.mean(preprocess_values) if preprocess_values else None,
        "mean_inference_ms": statistics.mean(inference_values) if inference_values else None,
        "median_inference_ms": statistics.median(inference_values) if inference_values else None,
        "p95_inference_ms": percentile(inference_values, 0.95),
        "mean_nms_ms": statistics.mean(nms_values) if nms_values else None,
        "mean_detector_end_to_end_ms": statistics.mean(detector_values) if detector_values else None,
        "mean_source_to_result_ms": statistics.mean(source_to_result_values) if source_to_result_values else None,
        "p95_source_to_result_ms": percentile(source_to_result_values, 0.95),
        "inference_only_fps": (
            1000.0 / statistics.mean(inference_values) if inference_values else 0.0
        ),
        "csv": csv_path,
        "evidence_image": image_path if latest_overlay is not None else None,
        "completed_utc": utc_now(),
        "interpretation": (
            "complete_measurement_window_fps uses the timed measurement window, including "
            "CSI capture, preprocessing, inference, NMS, drawing and logging. "
            "The per-frame capture_plus_detector timings exclude drawing/logging."
        ),
        "pass_requirements": {
            "minimum_measured_frames": model_spec["minimum_frames"],
            "cuda_required": True,
            "pinned_yolov5_v6_commit": EXPECTED_YOLOV5_V6_COMMIT,
            "model_variant": arguments.model,
            "motion_gate_eligible": bool(model_spec["motion_gate_eligible"]),
            "pinned_weights_sha256": expected_weights_sha256,
            "tracked_yolov5_source_clean": True,
            "stable_source_tree_and_weights_during_measurement": True,
        },
    }
    with open(json_path, "w") as output_file:
        json.dump(summary, output_file, indent=2, sort_keys=True)
        output_file.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print("Evidence: {}".format(json_path))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
