#!/usr/bin/env python3
"""Motor-free CSI/YOLO stop-sign demonstration for a VNC desktop."""

from __future__ import print_function

import argparse
import datetime
import json
import os
import sys
import time

import cv2

from gst_camera_bridge import GstCamera
from yolov5_runtime import (
    YoloV5Detector,
    draw_detections,
    evaluate_obstacles,
    format_detections,
)


WINDOW_TITLE = "Stop sign demo - YOLOv5n - motors disabled"
FORBIDDEN_MOTOR_MODULES = ("manual_motor_control", "safe_motor_output")


def utc_now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def stop_sign_decision(detections):
    """Give the stop-sign class priority over simultaneous person detections."""
    stop_signs = [item for item in detections if item["name"] == "stop sign"]
    if stop_signs:
        strongest = max(stop_signs, key=lambda item: item["confidence"])
        return {
            "state": "STOP",
            "reason": "STOP_SIGN_DETECTED",
            "confidence": float(strongest["confidence"]),
        }
    decision = evaluate_obstacles(detections)
    decision["confidence"] = None
    return decision


def main():
    parser = argparse.ArgumentParser(
        description="Motor-disabled live stop-sign recognition window"
    )
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--img-size", type=int, default=320)
    parser.add_argument("--confidence", type=float, default=0.45)
    parser.add_argument("--iou", type=float, default=0.45)
    arguments = parser.parse_args()

    if not 5.0 <= arguments.seconds <= 600.0:
        print("REFUSED: --seconds must be between 5 and 600.")
        return 1
    if not 0.20 <= arguments.confidence <= 0.90:
        print("REFUSED: --confidence must be between 0.20 and 0.90.")
        return 1

    imported_motor_modules = [
        name for name in FORBIDDEN_MOTOR_MODULES if name in sys.modules
    ]
    if imported_motor_modules:
        print("REFUSED: a physical motor module is unexpectedly imported: {}".format(
            ", ".join(imported_motor_modules)
        ))
        return 1

    project_directory = os.path.dirname(os.path.abspath(__file__))
    evidence_directory = os.path.join(project_directory, "evidence")
    if not os.path.isdir(evidence_directory):
        os.makedirs(evidence_directory)
    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    image_path = os.path.join(
        evidence_directory, "stop_sign_detection_{}.jpg".format(stamp)
    )
    json_path = os.path.join(
        evidence_directory, "stop_sign_detection_{}.json".format(stamp)
    )

    camera = None
    frame_number = 0
    best_confidence = -1.0
    stop_sign_detected = False
    saved_image = None
    started = time.monotonic()
    previous_loop = started

    print("=" * 68)
    print("STOP-SIGN VNC DEMO - PHYSICAL MOTOR MODULES ARE NOT IMPORTED")
    print("The car remains idle. Keep the motor battery disconnected.")
    print("Show a printed stop sign to the CSI camera; press Q to finish.")
    print("A genuine stop-sign frame is saved automatically.")
    print("=" * 68)

    try:
        detector = YoloV5Detector(
            image_size=arguments.img_size,
            confidence=arguments.confidence,
            iou=arguments.iou,
        )
        if detector.device.type != "cuda":
            raise RuntimeError("CUDA is required for this Jetson demonstration.")

        camera = GstCamera(
            sensor_id=0,
            capture_width=1280,
            capture_height=720,
            output_width=640,
            output_height=360,
            framerate=21,
        )

        success, warmup_frame = camera.read(timeout_seconds=3.0)
        if not success:
            raise RuntimeError("No CSI frame was received for warm-up.")
        detector.warmup_for_frame_shape(
            warmup_frame.shape[0], warmup_frame.shape[1], iterations=3
        )

        while time.monotonic() - started < arguments.seconds:
            success, frame = camera.read(timeout_seconds=3.0)
            if not success:
                raise RuntimeError("CSI frame unavailable.")

            result = detector.infer(frame)
            frame_number += 1
            now = time.monotonic()
            loop_seconds = max(0.000001, now - previous_loop)
            previous_loop = now
            decision = stop_sign_decision(result["detections"])

            overlay = frame.copy()
            draw_detections(
                overlay, result["detections"], result["corridor"]
            )
            is_stop_sign = decision["reason"] == "STOP_SIGN_DETECTED"
            if is_stop_sign:
                banner = "STOP - STOP SIGN"
                colour = (0, 0, 255)
            elif decision["state"] == "STOP":
                banner = "STOP - {}".format(decision["reason"])
                colour = (0, 0, 255)
            else:
                banner = "MONITORING - NO STOP SIGN"
                colour = (0, 255, 0)

            cv2.rectangle(overlay, (0, 0), (640, 86), (0, 0, 0), -1)
            cv2.putText(
                overlay,
                "YOLOv5n - MOTORS DISABLED",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                overlay,
                banner,
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                colour,
                2,
            )
            cv2.putText(
                overlay,
                "objects {} | inference {:.1f} ms | {:.2f} FPS".format(
                    len(result["detections"]),
                    result["inference_ms"],
                    1.0 / loop_seconds,
                ),
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
            )

            if is_stop_sign:
                confidence = float(decision["confidence"])
                print(
                    "STOP | STOP_SIGN_DETECTED | confidence {:.3f}".format(
                        confidence
                    )
                )
                stop_sign_detected = True
                if confidence > best_confidence:
                    best_confidence = confidence
                    if not cv2.imwrite(image_path, overlay):
                        raise RuntimeError("Could not save stop-sign image.")
                    saved_image = image_path
                    evidence = {
                        "schema_version": 1,
                        "test": "MOTOR_DISABLED_STOP_SIGN_VNC_DEMO",
                        "completed_utc": utc_now(),
                        "physical_motor_commands": False,
                        "motor_modules_imported": False,
                        "stop_sign_detected": True,
                        "decision": "STOP",
                        "reason": "STOP_SIGN_DETECTED",
                        "confidence": confidence,
                        "frame": frame_number,
                        "objects": format_detections(result["detections"]),
                        "image": image_path,
                        "model": "yolov5n",
                        "device": str(detector.device),
                        "fp16": bool(detector.use_half),
                    }
                    with open(json_path, "w") as summary_file:
                        json.dump(evidence, summary_file, indent=2, sort_keys=True)
                        summary_file.write("\n")
                    print("SAVED: {}".format(image_path))

            cv2.imshow(WINDOW_TITLE, overlay)
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                break

    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception as error:
        print("DEMO STOPPED: {}".format(error))
        return 1
    finally:
        if camera is not None:
            camera.close()
        cv2.destroyAllWindows()

    if stop_sign_detected:
        print("RESULT: STOP SIGN DETECTED; car remained idle.")
        print("Evidence image: {}".format(saved_image))
        print("Evidence JSON:  {}".format(json_path))
        return 0

    print("RESULT: no stop-sign class was detected; no evidence was saved.")
    print("Try even lighting, a larger sign in frame, and confidence 0.35.")
    return 2


if __name__ == "__main__":
    sys.exit(main())
