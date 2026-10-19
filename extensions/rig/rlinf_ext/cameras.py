# Copyright 2026 The RLinf-Franka Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""ZED 2i UVC camera backend.

The ZED SDK (``pyzed``) needs an NVIDIA GPU, so it cannot run on this
collection host. The camera still exposes a plain UVC stream, which this
driver captures through OpenCV: no SDK, no GPU, no depth.

A ZED UVC frame is the two eyes side by side, so one eye is cropped out and
scaled to the resolution the environment declares.
"""

from __future__ import annotations

import glob
import os
from typing import Any, Optional, Union

import numpy as np

from rlinf.robotics.parts.cameras.base import BaseCamera, Camera, CameraInfo
from rlinf.utils.logging import get_logger

_logger = get_logger()


@Camera.register("zed_uvc")
class ZEDUVCCamera(BaseCamera):
    """Capture one eye of a ZED camera's UVC stream through OpenCV.

    ``camera_info.serial_number`` may be:

    * a ``/dev/v4l/by-id/`` filename, which is stable across reboots
    * a ``"videoN"`` shorthand resolved to ``/dev/videoN``
    * a numeric string or int interpreted as a V4L2 device index

    The frame the device hands back is the stereo pair, so
    :pyattr:`CAPTURE_WIDTH` is the full side-by-side width. Only
    :pyattr:`EYE` is kept, then scaled to ``camera_info.resolution``.
    """

    SDK = "cv2"

    #: Side-by-side capture size. The ZED 2i offers 1344x376, 2560x720,
    #: 3840x1080 and 4416x1242. They are not just scales of one another:
    #: each mode frames the scene differently, and 1344x376 is the only one
    #: measured here that keeps the whole arm and the work table in view.
    #: It is also the cheapest, which leaves USB and CPU headroom.
    CAPTURE_WIDTH = 1344
    CAPTURE_HEIGHT = 376
    #: Which eye of the stereo pair to publish.
    EYE = "left"
    #: Frames discarded on open while auto-exposure settles.
    WARMUP_FRAMES = 15

    def __init__(self, camera_info: CameraInfo) -> None:
        super().__init__(camera_info)
        if camera_info.enable_depth:
            raise ValueError(
                "ZEDUVCCamera captures the UVC stream, which has no depth. "
                "Set enable_camera_depth: false or use the ZED SDK backend."
            )
        self._out_w, self._out_h = camera_info.resolution
        self._cv2: Any = None

    def _open(self) -> Any:
        """Open the V4L2 device and put it in YUYV at the capture size."""
        import cv2

        self._cv2 = cv2
        info = self._camera_info
        dev_path: Union[str, int] = self._resolve_device_path(info.serial_number)

        capture = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(
                f"Failed to open ZED UVC camera (serial={info.serial_number}, "
                f"dev_path={dev_path})."
            )

        try:
            self._configure(capture, dev_path)
        except Exception:
            capture.release()
            raise
        return capture

    def _configure(self, capture: Any, dev_path: Union[str, int]) -> None:
        """Select YUYV and the capture size, or say what the device gave back."""
        cv2 = self._cv2
        info = self._camera_info

        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.CAPTURE_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.CAPTURE_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, info.fps)
        try:
            # One buffered frame keeps a stale image from being read as live.
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception as exc:
            _logger.warning(
                "Failed to set ZED UVC buffer size (serial=%s): %s",
                info.serial_number,
                exc,
            )

        actual_w = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (actual_w, actual_h) != (self.CAPTURE_WIDTH, self.CAPTURE_HEIGHT):
            raise RuntimeError(
                f"ZED UVC camera (serial={info.serial_number}, dev_path={dev_path}) "
                f"returned {actual_w}x{actual_h}; expected "
                f"{self.CAPTURE_WIDTH}x{self.CAPTURE_HEIGHT}."
            )

        # The camera resets auto-exposure on open and the first frames come
        # back blown out. Discarding them here keeps an unusable frame out of
        # the buffer the environment reads its first observation from.
        for _ in range(self.WARMUP_FRAMES):
            capture.read()

    @staticmethod
    def _resolve_device_path(serial_number: Union[str, int]) -> Union[str, int]:
        """Map a configured identifier to a path OpenCV can open."""
        if isinstance(serial_number, int):
            return serial_number
        if serial_number.startswith("video"):
            return f"/dev/{serial_number}"
        by_id = f"/dev/v4l/by-id/{serial_number}"
        if os.path.exists(by_id):
            return by_id
        try:
            return int(serial_number)
        except ValueError as exc:
            raise ValueError(
                f"Could not resolve ZED UVC serial_number={serial_number!r} to a "
                f"V4L2 device. Pass a /dev/v4l/by-id name, 'videoN', or an index."
            ) from exc

    def _read_frame(self) -> tuple[bool, Optional[np.ndarray]]:
        """Crop the selected eye, then aspect-fill it to the declared size.

        The eye is 16:9 while the environment's frame is 4:3, so a plain
        resize would stretch the scene. Cropping to the target aspect first
        keeps geometry intact, which matters for the policy that trains on
        these frames.
        """
        ok, frame = self._device.read()
        if not ok or frame is None:
            return False, None

        width = frame.shape[1]
        half = width // 2
        if half == 0:
            _logger.warning(
                "Dropping undersized ZED UVC frame (serial=%s): %s",
                self.camera_info.serial_number,
                frame.shape,
            )
            return False, None
        frame = frame[:, :half] if self.EYE == "left" else frame[:, half:]

        frame = self._center_crop_to_aspect(frame, self._out_w, self._out_h)
        if (frame.shape[1], frame.shape[0]) != (self._out_w, self._out_h):
            frame = self._cv2.resize(
                frame, (self._out_w, self._out_h), interpolation=self._cv2.INTER_AREA
            )
        return True, frame

    @staticmethod
    def _center_crop_to_aspect(frame: np.ndarray, out_w: int, out_h: int) -> np.ndarray:
        """Crop ``frame`` to ``out_w:out_h`` around its centre."""
        height, width = frame.shape[:2]
        target = out_w / out_h
        current = width / height
        if current > target:
            new_w = max(1, int(round(height * target)))
            left = (width - new_w) // 2
            return frame[:, left : left + new_w]
        if current < target:
            new_h = max(1, int(round(width / target)))
            top = (height - new_h) // 2
            return frame[top : top + new_h, :]
        return frame

    def _release(self, device: Any) -> None:
        """Release the V4L2 handle."""
        if device is not None:
            device.release()

    @classmethod
    def discover(cls) -> set[str]:
        """Stable ``by-id`` names of the V4L2 cameras attached to this node."""
        devices = glob.glob("/dev/v4l/by-id/*")
        if not devices:
            devices = glob.glob("/dev/video*")
        return {os.path.basename(device) for device in devices}
