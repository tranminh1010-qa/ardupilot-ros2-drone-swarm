#!/usr/bin/env python3
"""OpenCV capture pipeline for the drone's down-facing Gazebo camera.

Each drone model streams H.264/RTP over UDP via GstCameraPlugin on port
5600 + instance (see src/custom_gz/models/base_drone*/model.sdf). This module
decodes that stream with OpenCV's GStreamer backend and provides
waypoint-triggered, geotagged frame capture plus ArUco marker detection.

Design constraints:
 - MUST be non-fatal: a missing camera, cv2, or GStreamer support must never
   break the flight mission. Every public method degrades to a no-op.
 - Frames are saved under /root/logs (docker-mounted to ./logs on the host) as
   logs/drone_<id>/wp_<n>.jpg plus a JSON sidecar with the geotag, so the
   host-side mosaic script can place them on the field map.
"""
import json
import os
import threading
import time

try:
    import cv2
    _CV2_OK = True
except ImportError:  # pragma: no cover - depends on container image
    cv2 = None
    _CV2_OK = False


class CameraProcessor:
    """Grabs frames from a GstCameraPlugin UDP stream and geotags them."""

    def __init__(self, drone_id, udp_port, out_dir="/root/logs", logger=None):
        self.drone_id = drone_id
        self.udp_port = udp_port
        self.out_dir = os.path.join(out_dir, f"drone_{drone_id}")
        self._log = logger
        self._cap = None
        self._lock = threading.Lock()
        self._latest_frame = None
        self._reader = None
        self._running = False

        if not _CV2_OK:
            self._warn("cv2 not available - camera capture disabled")
            return
        if not udp_port:
            self._info("no camera_port configured - camera capture disabled")
            return

        os.makedirs(self.out_dir, exist_ok=True)
        self._open_stream()

    # ------------------------------------------------------------------ util
    def _info(self, msg):
        if self._log:
            self._log.info(f"[camera] {msg}")

    def _warn(self, msg):
        if self._log:
            self._log.warning(f"[camera] {msg}")

    @property
    def active(self):
        return self._cap is not None and self._running

    # ---------------------------------------------------------------- stream
    def _open_stream(self):
        """Open the RTP/H.264 stream via OpenCV's GStreamer backend."""
        pipeline = (
            f"udpsrc port={self.udp_port} "
            "caps=\"application/x-rtp, media=video, encoding-name=H264\" "
            "! rtph264depay ! h264parse ! avdec_h264 "
            "! videoconvert ! video/x-raw, format=BGR "
            "! appsink drop=true max-buffers=1 sync=false"
        )
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        except Exception as e:  # GStreamer backend missing, etc.
            self._warn(f"failed to open stream on udp:{self.udp_port}: {e}")
            return
        if not cap.isOpened():
            self._warn(
                f"could not open GStreamer pipeline on udp:{self.udp_port} "
                "(stream not enabled yet, or cv2 built without GStreamer)")
            return

        self._cap = cap
        self._running = True
        # Dedicated reader thread keeps the appsink drained so capture()
        # always returns the freshest frame instead of a stale buffered one.
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._info(f"camera stream open on udp:{self.udp_port}")

    def _read_loop(self):
        while self._running:
            ok, frame = self._cap.read()
            if ok:
                with self._lock:
                    self._latest_frame = frame
            else:
                time.sleep(0.05)

    # --------------------------------------------------------------- capture
    def capture(self, wp_index, lat=None, lon=None, alt=None, extra=None):
        """Save the freshest frame as wp_<n>.jpg with a JSON geotag sidecar.

        Also runs ArUco detection on the frame; detections land in the
        sidecar so downstream tooling can georeference targets.
        Returns the image path, or None if no frame was available.
        """
        if not self.active:
            return None

        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
        if frame is None:
            self._warn(f"wp {wp_index}: no frame received yet")
            return None

        detections = self._detect_aruco(frame)

        img_path = os.path.join(self.out_dir, f"wp_{wp_index:03d}.jpg")
        cv2.imwrite(img_path, frame)

        meta = {
            "drone_id": self.drone_id,
            "wp_index": wp_index,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "unix_time": time.time(),
            "aruco_detections": detections,
        }
        if extra:
            meta.update(extra)
        with open(os.path.join(self.out_dir, f"wp_{wp_index:03d}.json"), "w") as f:
            json.dump(meta, f, indent=2)

        self._info(
            f"wp {wp_index}: saved {os.path.basename(img_path)}"
            + (f" ({len(detections)} aruco)" if detections else ""))
        return img_path

    def _detect_aruco(self, frame):
        """Detect ArUco markers (DICT_4X4_50); returns [{'id', 'corners'}]."""
        try:
            aruco = cv2.aruco
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            try:  # OpenCV >= 4.7 API
                detector = aruco.ArucoDetector(dictionary)
                corners, ids, _ = detector.detectMarkers(frame)
            except AttributeError:  # OpenCV 4.5.x API (Ubuntu 22.04)
                corners, ids, _ = aruco.detectMarkers(frame, dictionary)
        except Exception:
            return []
        if ids is None:
            return []
        return [
            {"id": int(i[0]), "corners": c.reshape(-1, 2).tolist()}
            for i, c in zip(ids, corners)
        ]

    # ---------------------------------------------------------------- close
    def close(self):
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None
