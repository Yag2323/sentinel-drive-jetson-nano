#!/usr/bin/env python3
"""Port the dashboard CSI feed to the shared upright GI/GStreamer bridge.

Run this utility on the Jetson from the directory containing
``robot_dashboard.py``. It uses the same ``GstCamera`` implementation as the
perception stack, creates a timestamped backup, and refuses to edit an
unfamiliar implementation. USB/custom-pipeline capture remains unchanged.

Compatible with Python 3.6.
"""

from __future__ import print_function

import argparse
import datetime
import os
import re
import shutil
import stat
import sys
import tempfile


BRIDGE_IMPORT = (
    "from gst_camera_bridge import DEFAULT_CSI_FLIP_METHOD, GstCamera"
)


class PatchError(RuntimeError):
    pass


def transform_source(source):
    """Return ``(updated_source, changed)`` for a recognised dashboard."""

    original_source = source

    future_import = "from __future__ import print_function"
    if future_import not in source:
        raise PatchError(
            "The expected future import is missing; refusing an uncertain edit"
        )

    # The current monitor-only dashboard has a native, latest-frame CSI
    # implementation. Its camera imports remain deliberately lazy so camera-off
    # mode still works on machines without Jetson GI libraries. Validate that
    # complete architecture and report it as already configured.
    integrated_markers = (
        "class CameraMonitor(threading.Thread):",
        "from gst_camera_bridge import (",
        "DEFAULT_CSI_FLIP_METHOD,",
        "GstCamera,",
        "capture = GstCamera(",
        "flip_method=DEFAULT_CSI_FLIP_METHOD,",
        '"--camera"',
        '"--camera-source"',
        '"--allow-lan-camera"',
        'if path == "/camera.mjpg":',
        "server.camera_monitor = camera_monitor",
    )
    if (
        "class CameraMonitor(threading.Thread):" in source
        and "Single-producer, latest-frame CSI monitor" in source
    ):
        missing = [marker for marker in integrated_markers if marker not in source]
        if missing:
            raise PatchError(
                "The integrated dashboard camera implementation is incomplete: {}"
                .format(", ".join(missing))
            )
        if re.search(r"(?m)^DEFAULT_CSI_FLIP_METHOD\s*=", source):
            raise PatchError("A dashboard-local CSI flip constant still exists")
        if source.count("capture = GstCamera(") != 1:
            raise PatchError("The integrated dashboard must open exactly one GstCamera")
        if source.count("flip_method=DEFAULT_CSI_FLIP_METHOD,") != 1:
            raise PatchError(
                "The integrated dashboard does not use the shared flip constant"
            )
        if "cv2.VideoCapture" in source:
            raise PatchError(
                "The integrated dashboard must not open CSI through OpenCV"
            )
        compile(source, "robot_dashboard.py", "exec")
        return source, False

    # Remove the old patcher's duplicate constant, if present. The only source
    # of truth must be gst_camera_bridge.DEFAULT_CSI_FLIP_METHOD.
    source = re.sub(
        r"(?m)^DEFAULT_CSI_FLIP_METHOD\s*=\s*2\s*(?:#.*)?\n?",
        "",
        source,
    )
    if BRIDGE_IMPORT not in source:
        insertion = source.find(future_import) + len(future_import)
        source = source[:insertion] + "\n\n" + BRIDGE_IMPORT + source[insertion:]

    function_start = source.find("def jetson_csi_pipeline(")
    if function_start < 0:
        raise PatchError(
            "jetson_csi_pipeline() was not found; this dashboard does not "
            "match the expected CSI-camera version"
        )

    function_end = source.find("\nclass CameraMonitor", function_start)
    if function_end < 0:
        raise PatchError(
            "CameraMonitor was not found after jetson_csi_pipeline(); "
            "refusing an uncertain edit"
        )

    before = source[:function_start]
    function = source[function_start:function_end]
    after = source[function_end:]

    placeholder = '"nvvidconv flip-method={flip_method} ! "'
    literal_zero = '"nvvidconv flip-method=0 ! "'
    literal_two = '"nvvidconv flip-method=2 ! "'
    literal_missing = '"nvvidconv ! "'

    if placeholder not in function:
        if literal_zero in function:
            function = function.replace(literal_zero, placeholder, 1)
        elif literal_two in function:
            function = function.replace(literal_two, placeholder, 1)
        elif literal_missing in function:
            function = function.replace(literal_missing, placeholder, 1)
        else:
            raise PatchError(
                "The built-in CSI pipeline has no recognised nvvidconv "
                "element; inspect it manually"
            )

    flip_argument = re.compile(
        r"(?m)^(?P<indent>[ \t]+)flip_method\s*=\s*[^,\r\n]+,\s*$"
    )
    flip_arguments = list(flip_argument.finditer(function))
    if len(flip_arguments) > 1:
        raise PatchError(
            "The CSI pipeline .format() block contains multiple flip_method "
            "arguments; refusing an uncertain edit"
        )
    if len(flip_arguments) == 1:
        match = flip_arguments[0]
        replacement = "{}flip_method=DEFAULT_CSI_FLIP_METHOD,".format(
            match.group("indent")
        )
        function = (
            function[: match.start()]
            + replacement
            + function[match.end() :]
        )
    else:
        format_anchor = re.search(
            r"\)\.format\(\n(?P<indent>[ \t]+)sensor_id=int\(sensor_id\),",
            function,
        )
        if format_anchor is None:
            raise PatchError(
                "The CSI pipeline .format() block is unfamiliar; refusing "
                "an uncertain edit"
            )
        indent = format_anchor.group("indent")
        original = format_anchor.group(0)
        replacement = (
            ").format(\n"
            + indent
            + "flip_method=DEFAULT_CSI_FLIP_METHOD,\n"
            + indent
            + "sensor_id=int(sensor_id),"
        )
        function = function.replace(original, replacement, 1)

    source = before + function + after

    old_csi_capture = (
        '        elif self.mode == "csi":\n'
        "            pipeline = jetson_csi_pipeline(\n"
        "                self.source,\n"
        "                self.width,\n"
        "                self.height,\n"
        "                self.target_fps,\n"
        "            )\n"
        "            capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)"
    )
    new_csi_capture = (
        '        elif self.mode == "csi":\n'
        "            capture = GstCamera(\n"
        "                sensor_id=self.source,\n"
        "                capture_width=max(1280, self.width),\n"
        "                capture_height=max(720, self.height),\n"
        "                output_width=self.width,\n"
        "                output_height=self.height,\n"
        "                framerate=max(1, int(round(self.target_fps))),\n"
        "                flip_method=DEFAULT_CSI_FLIP_METHOD,\n"
        "            )"
    )
    if old_csi_capture in source:
        source = source.replace(old_csi_capture, new_csi_capture, 1)
    elif new_csi_capture not in source:
        raise PatchError(
            "The CameraMonitor CSI capture block is unfamiliar; refusing an "
            "uncertain edit"
        )

    if source.count(BRIDGE_IMPORT) != 1:
        raise PatchError("The shared CSI bridge import is missing or duplicated")
    if re.search(r"(?m)^DEFAULT_CSI_FLIP_METHOD\s*=", source):
        raise PatchError("A dashboard-local CSI flip constant still exists")
    if source.count("nvvidconv flip-method={flip_method} !") != 1:
        raise PatchError("The built-in CSI flip placeholder is missing or duplicated")
    if source.count("flip_method=DEFAULT_CSI_FLIP_METHOD,") != 2:
        raise PatchError(
            "The dashboard pipeline and GstCamera must both consume the "
            "shared flip constant"
        )
    if source.count("capture = GstCamera(") != 1:
        raise PatchError("The dashboard CSI mode does not use exactly one GstCamera")

    return source, source != original_source


def patch_file(path):
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        raise PatchError("Dashboard file not found: {}".format(path))

    with open(path, "r") as handle:
        original = handle.read()

    updated, _unused_changed = transform_source(original)
    if updated == original:
        print("Already configured: CSI uses the shared upright GI bridge.")
        print("Dashboard: {}".format(path))
        return False

    # Compile before touching the working dashboard.
    compile(updated, path, "exec")

    timestamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    backup = "{}.before-csi-bridge-{}.bak".format(path, timestamp)
    shutil.copy2(path, backup)

    original_mode = stat.S_IMODE(os.stat(path).st_mode)
    directory = os.path.dirname(path) or "."
    file_descriptor, temporary = tempfile.mkstemp(
        prefix=".robot_dashboard.", suffix=".tmp", dir=directory
    )
    try:
        with os.fdopen(file_descriptor, "w") as handle:
            handle.write(updated)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, original_mode)
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise

    print("Updated: {}".format(path))
    print("Backup:  {}".format(backup))
    print("CSI capture: shared GI/GStreamer bridge")
    print("CSI rotation: nvvidconv flip-method=2 (single shared constant)")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Port the dashboard CSI feed to the shared upright bridge."
    )
    parser.add_argument(
        "dashboard",
        nargs="?",
        default="robot_dashboard.py",
        help="Path to robot_dashboard.py (default: ./robot_dashboard.py)",
    )
    arguments = parser.parse_args()

    try:
        patch_file(arguments.dashboard)
        return 0
    except (PatchError, SyntaxError, OSError) as error:
        print("PATCH REFUSED: {}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
