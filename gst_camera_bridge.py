#!/usr/bin/env python3
"""Jetson CSI camera bridge using GStreamer and GI.

The camera is opened by GStreamer rather than OpenCV because the Jetson Nano's
OpenCV build does not have GStreamer enabled.  Every consumer imports this one
class so the physical 180-degree camera correction is defined in one place.
"""

from __future__ import print_function

import gi
import numpy as np
import time

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")

from gi.repository import Gst, GstApp  # noqa: E402,F401


DEFAULT_CSI_FLIP_METHOD = 2


def build_csi_pipeline(
    sensor_id=0,
    sensor_mode=None,
    capture_width=1280,
    capture_height=720,
    output_width=640,
    output_height=360,
    framerate=30,
    flip_method=DEFAULT_CSI_FLIP_METHOD,
):
    """Return the single canonical CSI pipeline used by this project."""
    sensor_mode_property = (
        " sensor-mode={}".format(int(sensor_mode))
        if sensor_mode is not None
        else ""
    )
    return (
        "nvarguscamerasrc sensor-id={sensor_id}{sensor_mode_property} ! "
        "video/x-raw(memory:NVMM), "
        "width=(int){capture_width}, "
        "height=(int){capture_height}, "
        "format=(string)NV12, "
        "framerate=(fraction){framerate}/1 ! "
        "nvvidconv flip-method={flip_method} ! "
        "video/x-raw, "
        "width=(int){output_width}, "
        "height=(int){output_height}, "
        "format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink name=sink emit-signals=false sync=false "
        "max-buffers=1 drop=true"
    ).format(
        sensor_id=int(sensor_id),
        sensor_mode_property=sensor_mode_property,
        capture_width=int(capture_width),
        capture_height=int(capture_height),
        output_width=int(output_width),
        output_height=int(output_height),
        framerate=int(framerate),
        flip_method=int(flip_method),
    )


class GstCamera(object):
    """Capture IMX219 CSI frames as BGR NumPy arrays."""

    def __init__(
        self,
        sensor_id=0,
        sensor_mode=None,
        capture_width=1280,
        capture_height=720,
        output_width=640,
        output_height=360,
        framerate=30,
        flip_method=DEFAULT_CSI_FLIP_METHOD,
    ):
        Gst.init(None)

        self.closed = False
        self._frame_sequence = 0
        self._last_pts_ns = None
        self._pts_origin_ns = None
        self._pts_origin_monotonic = None
        self.pipeline_description = build_csi_pipeline(
            sensor_id=sensor_id,
            sensor_mode=sensor_mode,
            capture_width=capture_width,
            capture_height=capture_height,
            output_width=output_width,
            output_height=output_height,
            framerate=framerate,
            flip_method=flip_method,
        )

        print("Starting GStreamer camera pipeline:")
        print(self.pipeline_description)

        self.pipeline = None
        self.appsink = None
        self.bus = None
        try:
            self.pipeline = Gst.parse_launch(self.pipeline_description)
            self.appsink = self.pipeline.get_by_name("sink")
            if self.appsink is None:
                raise RuntimeError(
                    "The GStreamer appsink element was not created."
                )

            self.bus = self.pipeline.get_bus()
            state_result = self.pipeline.set_state(Gst.State.PLAYING)
            if state_result == Gst.StateChangeReturn.FAILURE:
                self._raise_pipeline_error(
                    "Could not start the camera pipeline."
                )

            state_result, _current, _pending = self.pipeline.get_state(
                5 * Gst.SECOND
            )
            if state_result == Gst.StateChangeReturn.FAILURE:
                self._raise_pipeline_error(
                    "The camera pipeline failed during startup."
                )
        except Exception:
            if self.pipeline is not None:
                self.pipeline.set_state(Gst.State.NULL)
            self.closed = True
            raise

        print("GStreamer camera pipeline is running.")

    def _raise_pipeline_error(self, default_message):
        message = self.bus.timed_pop_filtered(
            500 * Gst.MSECOND,
            Gst.MessageType.ERROR,
        )
        if message is not None:
            error, debug_information = message.parse_error()
            raise RuntimeError(
                "{}\nGStreamer error: {}\nDebug: {}".format(
                    default_message,
                    error,
                    debug_information,
                )
            )
        raise RuntimeError(default_message)

    def _check_bus(self):
        while True:
            message = self.bus.pop()
            if message is None:
                return
            if message.type == Gst.MessageType.ERROR:
                error, debug_information = message.parse_error()
                raise RuntimeError(
                    "GStreamer error: {}\nDebug: {}".format(
                        error,
                        debug_information,
                    )
                )
            if message.type == Gst.MessageType.EOS:
                raise RuntimeError("The camera pipeline reached end-of-stream.")

    def read_with_metadata(self, timeout_seconds=2.0):
        """Return ``(success, frame, metadata)`` with source-age evidence."""
        if self.closed:
            return False, None, None

        sample = self.appsink.emit(
            "try-pull-sample",
            int(float(timeout_seconds) * Gst.SECOND),
        )
        if sample is None:
            self._check_bus()
            return False, None, None

        caps = sample.get_caps()
        if caps is None or caps.get_size() == 0:
            return False, None, None

        structure = caps.get_structure(0)
        width = int(structure.get_value("width"))
        height = int(structure.get_value("height"))
        buffer = sample.get_buffer()
        if buffer is None:
            return False, None, None

        pts_value = int(buffer.pts)
        pts_valid = (
            pts_value >= 0 and pts_value != int(Gst.CLOCK_TIME_NONE)
        )
        offset_value = int(buffer.offset)
        buffer_offset_none = int(
            getattr(Gst, "BUFFER_OFFSET_NONE", 18446744073709551615)
        )
        offset_valid = (
            offset_value >= 0 and offset_value != buffer_offset_none
        )

        success, map_information = buffer.map(Gst.MapFlags.READ)
        if not success:
            return False, None, None

        try:
            raw_data = np.frombuffer(map_information.data, dtype=np.uint8)
            expected_size = width * height * 3
            if raw_data.size < expected_size:
                raise RuntimeError(
                    "Camera buffer is too small. Expected {} bytes but received {}."
                    .format(expected_size, raw_data.size)
                )
            frame = raw_data[:expected_size].reshape((height, width, 3)).copy()
        finally:
            buffer.unmap(map_information)

        arrival_monotonic = time.monotonic()
        self._frame_sequence += 1
        capture_monotonic = None
        source_age_method = None
        increasing_pts = False
        if pts_valid:
            # Prefer the pipeline clock. PTS is converted through the sample's
            # segment into running time, then compared with the pipeline's
            # current running time. This preserves acquisition/backlog age;
            # timestamping after ``read()`` would incorrectly report it as 0.
            try:
                clock = self.pipeline.get_clock()
                base_time = int(self.pipeline.get_base_time())
                segment = sample.get_segment()
                buffer_running_ns = (
                    int(segment.to_running_time(Gst.Format.TIME, pts_value))
                    if segment is not None
                    else pts_value
                )
                clock_time_none = int(Gst.CLOCK_TIME_NONE)
                if (
                    clock is not None
                    and base_time != clock_time_none
                    and buffer_running_ns != clock_time_none
                ):
                    current_running_ns = int(clock.get_time()) - base_time
                    source_age_s = (
                        current_running_ns - buffer_running_ns
                    ) / 1000000000.0
                    # A tiny negative value is possible because clocks are read
                    # at different instants; a large negative value is invalid.
                    if source_age_s >= -0.050:
                        capture_monotonic = arrival_monotonic - max(
                            0.0, source_age_s
                        )
                        source_age_method = "GSTREAMER_RUNNING_TIME"
            except Exception:
                capture_monotonic = None

            # Compatibility fallback for older GI builds. It detects repeats
            # and accumulating delay, but cannot prove initial-frame age.
            if capture_monotonic is None:
                if self._pts_origin_ns is None:
                    self._pts_origin_ns = pts_value
                    self._pts_origin_monotonic = arrival_monotonic
                capture_monotonic = self._pts_origin_monotonic + (
                    pts_value - self._pts_origin_ns
                ) / 1000000000.0
                source_age_method = "PTS_ORIGIN_FALLBACK"
            increasing_pts = (
                self._last_pts_ns is None or pts_value > self._last_pts_ns
            )
            if increasing_pts:
                self._last_pts_ns = pts_value

        metadata = {
            "sequence": self._frame_sequence,
            "pts_ns": pts_value if pts_valid else None,
            "buffer_offset": offset_value if offset_valid else None,
            "arrival_monotonic": arrival_monotonic,
            "capture_monotonic": capture_monotonic,
            "timestamp_valid": bool(
                pts_valid and capture_monotonic is not None
            ),
            "source_age_method": source_age_method,
            "source_age_trustworthy": bool(
                source_age_method == "GSTREAMER_RUNNING_TIME"
            ),
            "fresh": bool(
                pts_valid and capture_monotonic is not None and increasing_pts
            ),
            "age_seconds_at_arrival": (
                max(0.0, arrival_monotonic - capture_monotonic)
                if capture_monotonic is not None
                else None
            ),
        }
        return True, frame, metadata

    def read(self, timeout_seconds=2.0):
        """Backward-compatible ``(success, frame)`` camera read."""
        success, frame, _metadata = self.read_with_metadata(timeout_seconds)
        return success, frame

    def isOpened(self):
        """OpenCV-compatible state query used by the dashboard sampler."""
        return bool(not self.closed and self.pipeline is not None)

    def set(self, _property, _value):
        """OpenCV-compatible no-op; CSI geometry is fixed in the pipeline."""
        return False

    def release(self):
        """OpenCV-compatible alias used by the dashboard sampler."""
        self.close()

    def close(self):
        if self.closed:
            return
        self.pipeline.set_state(Gst.State.NULL)
        self.closed = True
        print("GStreamer camera pipeline stopped.")

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.close()


if __name__ == "__main__":
    print(build_csi_pipeline())
