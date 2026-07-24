#!/usr/bin/env python3
"""OpenCV capture pipeline for the drone's down-facing Gazebo camera.

Each drone model streams H.264/RTP over UDP via GstCameraPlugin on port
5600 + instance (see src/custom_gz/models/base_drone*/model.sdf). This module
decodes that stream with OpenCV's GStreamer backend and provides
waypoint-triggered, geotagged frame capture plus ArUco marker detection and
weed detection.

Design constraints:
 - MUST be non-fatal: a missing camera, cv2, or GStreamer support must never
   break the flight mission. Every public method degrades to a no-op.
 - Frames are saved under /root/logs (docker-mounted to ./logs on the host) as
   logs/drone_<id>/wp_<n>.jpg plus a JSON sidecar with the geotag, so the
   host-side mosaic script can place them on the field map.
"""
import json
import math
import os
import threading
import time

try:
    import cv2
    _CV2_OK = True
except ImportError:  # pragma: no cover - depends on container image
    cv2 = None
    _CV2_OK = False

# Weed detection: the sim's weeds (model weed_field) are pure magenta blobs,
# so a tight HSV band + area filter is a reliable detector. Camera intrinsics
# from base_drone_cam/model.sdf, used for nadir georeferencing.
CAMERA_HFOV_RAD = 1.2
WEED_HSV_LOW = (140, 100, 80)
WEED_HSV_HIGH = (170, 255, 255)
WEED_MIN_AREA_PX = 50


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
        weeds = self._detect_weeds(frame, lat=lat, lon=lon, alt=alt)

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
            "weed_detections": weeds,
        }
        if extra:
            meta.update(extra)
        with open(os.path.join(self.out_dir, f"wp_{wp_index:03d}.json"), "w") as f:
            json.dump(meta, f, indent=2)

        self._info(
            f"wp {wp_index}: saved {os.path.basename(img_path)}"
            + (f" ({len(detections)} aruco)" if detections else "")
            + (f" ({len(weeds)} weeds)" if weeds else ""))
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

    def _detect_weeds(self, frame, lat=None, lon=None, alt=None):
        """Detect magenta weed blobs; returns [{'centroid_px', 'bbox',
        'area_px', 'lat', 'lon'}]. lat/lon are georeferenced from the drone
        pose assuming a north-aligned nadir camera (approximate — vehicle yaw
        is not compensated), or None when the pose is unknown."""
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, WEED_HSV_LOW, WEED_HSV_HIGH)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            h, w = frame.shape[:2]
            f_px = (w / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
            detections = []
            for c in contours:
                area = cv2.contourArea(c)
                if area < WEED_MIN_AREA_PX:
                    continue
                bx, by, bw, bh = cv2.boundingRect(c)
                cx = bx + bw / 2.0
                cy = by + bh / 2.0
                det = {
                    "centroid_px": [cx, cy],
                    "bbox": [bx, by, bw, bh],
                    "area_px": area,
                    "lat": None,
                    "lon": None,
                }
                if lat is not None and lon is not None and alt is not None:
                    east = (cx - w / 2.0) / f_px * alt
                    north = -(cy - h / 2.0) / f_px * alt
                    det["lat"] = lat + north / 111111.0
                    det["lon"] = lon + east / (
                        111111.0 * math.cos(math.radians(lat)))
                detections.append(det)
            return detections
        except Exception:
            return []

    # ---------------------------------------------------------------- close
    def close(self):
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None
