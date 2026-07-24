#!/usr/bin/env python3
"""Tests for CameraProcessor._detect_weeds. Run directly:
    python3 tests/test_weed_detection.py
Needs cv2 + numpy on the host (venv-ardupilot has both)."""
import math
import os
import sys

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "swarm_control", "scripts"))
from camera_processor import CameraProcessor, CAMERA_HFOV_RAD  # noqa: E402

# udp_port=0 -> no stream is opened; detection methods still usable.
CP = CameraProcessor(0, 0)

GREEN_BGR = (40, 120, 60)     # like the crop fields
MAGENTA_BGR = (255, 0, 255)   # weed blob color


def _field_frame():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = GREEN_BGR
    return img


def test_detects_two_blobs():
    img = _field_frame()
    cv2.circle(img, (200, 150), 15, MAGENTA_BGR, -1)
    cv2.circle(img, (500, 400), 25, MAGENTA_BGR, -1)
    dets = CP._detect_weeds(img)
    assert len(dets) == 2, dets
    centroids = sorted((d["centroid_px"][0], d["centroid_px"][1]) for d in dets)
    assert abs(centroids[0][0] - 200) < 3 and abs(centroids[0][1] - 150) < 3
    assert abs(centroids[1][0] - 500) < 3 and abs(centroids[1][1] - 400) < 3
    for d in dets:
        assert d["area_px"] >= 50
        assert d["lat"] is None and d["lon"] is None  # no pose given


def test_ignores_weed_free_frame_and_speckle():
    img = _field_frame()
    img[100, 100] = MAGENTA_BGR  # single-pixel noise, killed by morphology/area
    assert CP._detect_weeds(img) == []


def test_georeference_nadir():
    lat, lon, alt = 40.072842, -105.230575, 30.0
    img = _field_frame()
    cv2.circle(img, (320 + 100, 240), 15, MAGENTA_BGR, -1)  # 100 px east of center
    dets = CP._detect_weeds(img, lat=lat, lon=lon, alt=alt)
    assert len(dets) == 1
    f_px = (640 / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
    east_m = 100.0 / f_px * alt
    exp_lon = lon + east_m / (111111.0 * math.cos(math.radians(lat)))
    assert abs(dets[0]["lon"] - exp_lon) < 1e-6, (dets[0]["lon"], exp_lon)
    assert abs(dets[0]["lat"] - lat) < 1e-6  # centered vertically -> no north offset


def test_never_raises_on_garbage():
    assert CP._detect_weeds(np.zeros((4, 4), dtype=np.uint8)) == []  # grayscale


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"PASS {name}")
    print("ALL PASS")
