#!/usr/bin/env python3
"""YOLOv5 v6 runtime proven compatible with the Jetson Nano environment."""

from __future__ import print_function

import os
import queue
import sys
import threading
import time

import cv2
import numpy as np
import torch


PROJECT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
YOLO_DIRECTORY = os.path.join(PROJECT_DIRECTORY, "yolov5_v6")
DEFAULT_WEIGHTS_PATH = os.path.join(YOLO_DIRECTORY, "yolov5n.pt")

if YOLO_DIRECTORY not in sys.path:
    sys.path.insert(0, YOLO_DIRECTORY)

from models.experimental import attempt_load  # noqa: E402
from utils.datasets import letterbox  # noqa: E402
from utils.general import (  # noqa: E402
    check_img_size,
    non_max_suppression,
    scale_coords,
)


CLASSES_OF_INTEREST = [0, 1, 2, 3, 5, 7, 9, 11, 56]
OBSTACLE_NAMES = {
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "bus",
    "truck",
    "chair",
}
SLOW_AREA_RATIO = 0.035
STOP_AREA_RATIO = 0.120
STOP_SIGN_AREA_RATIO = 0.008


def detection_colour(name):
    colours = {
        "person": (0, 0, 255),
        "bicycle": (255, 255, 0),
        "car": (0, 255, 0),
        "motorcycle": (255, 0, 255),
        "bus": (0, 165, 255),
        "truck": (255, 0, 0),
        "traffic light": (255, 255, 255),
        "stop sign": (0, 0, 255),
        "chair": (0, 255, 255),
    }
    return colours.get(name, (255, 255, 255))


class YoloV5Detector(object):
    def __init__(
        self,
        weights_path=DEFAULT_WEIGHTS_PATH,
        image_size=256,
        confidence=0.45,
        iou=0.45,
    ):
        if not os.path.isfile(weights_path):
            raise RuntimeError("YOLO weights not found: {}".format(weights_path))
        self.device = torch.device(
            "cuda:0" if torch.cuda.is_available() else "cpu"
        )
        self.use_half = self.device.type == "cuda"
        self.confidence = float(confidence)
        self.iou = float(iou)

        print("Loading YOLOv5 weights: {}".format(weights_path))
        print("Device: {} | FP16: {}".format(self.device, self.use_half))
        self.model = attempt_load(weights_path, map_location=self.device)
        self.model.eval()
        self.stride = int(self.model.stride.max())
        self.image_size = check_img_size(int(image_size), s=self.stride)
        if self.use_half:
            self.model.half()
        self.names = (
            self.model.module.names
            if hasattr(self.model, "module")
            else self.model.names
        )

        warmup = torch.zeros(
            1,
            3,
            self.image_size,
            self.image_size,
            device=self.device,
        )
        if self.use_half:
            warmup = warmup.half()
        with torch.no_grad():
            self.model(warmup)[0]

    def _prepare(self, frame):
        resized = letterbox(
            frame,
            new_shape=self.image_size,
            stride=self.stride,
            auto=True,
        )[0]
        resized = resized[:, :, ::-1].transpose(2, 0, 1)
        resized = np.ascontiguousarray(resized)
        tensor = torch.from_numpy(resized).to(self.device)
        tensor = tensor.half() if self.use_half else tensor.float()
        tensor /= 255.0
        if tensor.ndimension() == 3:
            tensor = tensor.unsqueeze(0)
        return tensor

    def infer(self, frame):
        total_start = time.perf_counter()
        tensor = self._prepare(frame)
        preprocess_ms = (time.perf_counter() - total_start) * 1000.0

        if self.device.type == "cuda":
            torch.cuda.synchronize()
        inference_start = time.perf_counter()
        with torch.no_grad():
            prediction = self.model(tensor, augment=False)[0]
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        inference_ms = (time.perf_counter() - inference_start) * 1000.0

        nms_start = time.perf_counter()
        with torch.no_grad():
            prediction = non_max_suppression(
                prediction,
                conf_thres=self.confidence,
                iou_thres=self.iou,
                classes=CLASSES_OF_INTEREST,
                agnostic=False,
            )
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        nms_ms = (time.perf_counter() - nms_start) * 1000.0

        frame_height, frame_width = frame.shape[:2]
        corridor = {
            "left": int(frame_width * 0.25),
            "right": int(frame_width * 0.75),
            "top": int(frame_height * 0.42),
            "bottom": frame_height - 1,
        }
        detections = []
        for detected_group in prediction:
            if len(detected_group) == 0:
                continue
            detected_group[:, :4] = scale_coords(
                tensor.shape[2:], detected_group[:, :4], frame.shape
            ).round()
            for row in reversed(detected_group):
                x1, y1, x2, y2 = [int(row[index].item()) for index in range(4)]
                confidence = float(row[4].item())
                class_id = int(row[5].item())
                name = self.names[class_id]
                box_width = max(0, x2 - x1)
                box_height = max(0, y2 - y1)
                area_ratio = float(box_width * box_height) / float(
                    frame_width * frame_height
                )
                centre_x = int((x1 + x2) / 2)
                inside_path = (
                    corridor["left"] <= centre_x <= corridor["right"]
                    and y2 >= corridor["top"]
                )
                detections.append(
                    {
                        "name": str(name),
                        "confidence": confidence,
                        "x1": x1,
                        "y1": y1,
                        "x2": x2,
                        "y2": y2,
                        "area_ratio": area_ratio,
                        "inside_path": inside_path,
                    }
                )
        return {
            "detections": detections,
            "corridor": corridor,
            "preprocess_ms": preprocess_ms,
            "inference_ms": inference_ms,
            "nms_ms": nms_ms,
            "end_to_end_ms": (time.perf_counter() - total_start) * 1000.0,
            "input_tensor_shape": [int(value) for value in tensor.shape],
        }


def mark_path_overlap(detections, path_polygon):
    """Reclassify path overlap against the physically calibrated IPM polygon."""
    polygon = np.float32(path_polygon)
    updated = []
    for original in detections:
        item = dict(original)
        rectangle = np.float32(
            [
                [item["x1"], item["y1"]],
                [item["x2"], item["y1"]],
                [item["x2"], item["y2"]],
                [item["x1"], item["y2"]],
            ]
        )
        try:
            overlap_area, _intersection = cv2.intersectConvexConvex(
                polygon, rectangle
            )
            item["inside_path"] = float(overlap_area) > 0.0
        except Exception:
            # Conservative fallback: any polygon/box vertex containment counts.
            box_points = [tuple(point) for point in rectangle]
            polygon_points = [tuple(point) for point in polygon]
            box_contains_polygon = any(
                item["x1"] <= point[0] <= item["x2"]
                and item["y1"] <= point[1] <= item["y2"]
                for point in polygon_points
            )
            polygon_contains_box = any(
                cv2.pointPolygonTest(polygon, point, False) >= 0
                for point in box_points
            )
            item["inside_path"] = bool(
                box_contains_polygon or polygon_contains_box
            )
        updated.append(item)
    return updated


def evaluate_obstacles(detections):
    """Conservative physical policy: any relevant detection means STOP.

    The previous area-ratio thresholds were not calibrated to stopping distance.
    They remain logged as descriptive image measurements but are not used to
    authorize physical motion.
    """
    for item in detections:
        normalized_name = item["name"].upper().replace(" ", "_")
        if item["name"] == "stop sign":
            return {"state": "STOP", "reason": "STOP_SIGN_DETECTED"}
        if item["name"] == "traffic light":
            return {"state": "STOP", "reason": "TRAFFIC_LIGHT_UNCLASSIFIED"}
        if item["name"] in OBSTACLE_NAMES:
            return {
                "state": "STOP",
                "reason": "OBJECT_DETECTED_{}".format(normalized_name),
            }
    return {"state": "DRIVE", "reason": "PATH_CLEAR"}


def format_detections(detections):
    return "|".join(
        "{}:{:.2f}:{:.3f}:{}".format(
            item["name"],
            item["confidence"],
            item["area_ratio"],
            "path" if item["inside_path"] else "outside",
        )
        for item in detections
    )


def draw_detections(image, detections, corridor, path_polygon=None):
    if path_polygon is not None:
        cv2.polylines(
            image,
            [np.int32(path_polygon)],
            True,
            (255, 255, 0),
            2,
        )
    else:
        cv2.rectangle(
            image,
            (corridor["left"], corridor["top"]),
            (corridor["right"], corridor["bottom"]),
            (255, 255, 0),
            1,
        )
    for item in detections:
        colour = detection_colour(item["name"])
        cv2.rectangle(
            image,
            (item["x1"], item["y1"]),
            (item["x2"], item["y2"]),
            colour,
            2,
        )
        label = "{} {:.2f} A:{:.3f}{}".format(
            item["name"],
            item["confidence"],
            item["area_ratio"],
            " PATH" if item["inside_path"] else "",
        )
        cv2.putText(
            image,
            label,
            (item["x1"], max(18, item["y1"] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            colour,
            2,
        )


class AsyncYoloWorker(object):
    """Single-frame asynchronous worker with no stale-frame backlog."""

    def __init__(self, detector):
        self.detector = detector
        self.frame_queue = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.busy = False
        self.latest = {
            "has_result": False,
            "detections": [],
            "corridor": {"left": 160, "right": 480, "top": 151, "bottom": 359},
            "preprocess_ms": 0.0,
            "inference_ms": 0.0,
            "nms_ms": 0.0,
            "end_to_end_ms": 0.0,
            "input_tensor_shape": None,
            "source_frame": 0,
            "source_capture_monotonic": None,
            "source_pts_ns": None,
            "result_time": 0.0,
            "error": None,
        }
        self.thread = threading.Thread(target=self._run, name="AsyncYoloWorker")
        self.thread.daemon = True

    def start(self):
        self.thread.start()

    def submit(
        self,
        frame,
        frame_number,
        capture_monotonic=None,
        source_pts_ns=None,
    ):
        if capture_monotonic is None:
            capture_monotonic = time.monotonic()
        with self.lock:
            if self.busy:
                return False
            self.busy = True
        try:
            self.frame_queue.put_nowait(
                (
                    int(frame_number),
                    float(capture_monotonic),
                    source_pts_ns,
                    frame.copy(),
                )
            )
            return True
        except queue.Full:
            with self.lock:
                self.busy = False
            return False

    def snapshot(self, current_frame_number=None):
        with self.lock:
            result = dict(self.latest)
            result["detections"] = [dict(item) for item in self.latest["detections"]]
            result["corridor"] = dict(self.latest["corridor"])
            result["busy"] = bool(self.busy)
            now = time.monotonic()
            source_time = self.latest.get("source_capture_monotonic")
            result["age_seconds"] = (
                now - float(source_time)
                if self.latest["has_result"] and source_time is not None
                else float("inf")
            )
            result["result_age_seconds"] = (
                now - float(self.latest["result_time"])
                if self.latest["has_result"]
                else float("inf")
            )
            result["source_frame_lag"] = (
                max(0, int(current_frame_number) - int(self.latest["source_frame"]))
                if self.latest["has_result"] and current_frame_number is not None
                else None
            )
            return result

    def _run(self):
        while not self.stop_event.is_set():
            try:
                item = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            frame_number, capture_monotonic, source_pts_ns, frame = item
            try:
                result = self.detector.infer(frame)
                result["has_result"] = True
                result["source_frame"] = frame_number
                result["source_capture_monotonic"] = capture_monotonic
                result["source_pts_ns"] = source_pts_ns
                result["result_time"] = time.monotonic()
                result["error"] = None
                with self.lock:
                    self.latest = result
            except Exception as error:
                with self.lock:
                    self.latest["error"] = repr(error)
            finally:
                with self.lock:
                    self.busy = False
                self.frame_queue.task_done()

    def stop(self):
        self.stop_event.set()
        try:
            self.frame_queue.put_nowait(None)
        except queue.Full:
            pass
        self.thread.join(timeout=5.0)
