#!/usr/bin/env python3
"""Combine the per-drone geotagged mapping frames into one field map.

Run on the HOST after a mission (needs python3-opencv or opencv-python):

    python3 src/scripts/stitch_field_map.py            # reads ./logs
    python3 src/scripts/stitch_field_map.py --logs ./logs --out field_map.jpg

Each CameraDrone saves logs/drone_<id>/wp_<n>.jpg plus a JSON sidecar with
the waypoint's lat/lon/alt. This script projects every frame onto a common
ground canvas by position (position-based mosaic — more robust for nadir
imagery with sparse overlap than feature-based cv2.Stitcher), using the
camera's ground footprint (2 * alt * tan(hfov/2)) for scale.

Assumption: frames are pasted axis-aligned (no per-frame yaw rotation);
ArduCopter GUIDED flight keeps yaw roughly constant so seams are minor.
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("OpenCV required: pip install opencv-python (or apt python3-opencv)")

# Must match the camera in src/custom_gz/models/base_drone_cam*/model.sdf
HFOV_RAD = 1.2
IMG_W, IMG_H = 640, 480

# Shared field origin (see compose_decentralized.launch.py)
ORIGIN_LAT = 40.072842
ORIGIN_LON = -105.230575

M_PER_DEG_LAT = 111_320.0


def latlon_to_xy(lat, lon):
    """Field-local metres (x=east, y=north) from the shared origin."""
    x = (lon - ORIGIN_LON) * M_PER_DEG_LAT * math.cos(math.radians(ORIGIN_LAT))
    y = (lat - ORIGIN_LAT) * M_PER_DEG_LAT
    return x, y


def load_frames(logs_dir):
    frames = []
    for meta_path in sorted(glob.glob(os.path.join(logs_dir, "drone_*", "wp_*.json"))):
        img_path = meta_path.replace(".json", ".jpg")
        if not os.path.exists(img_path):
            continue
        with open(meta_path) as f:
            meta = json.load(f)
        if meta.get("lat") is None or meta.get("lon") is None:
            continue
        frames.append((img_path, meta))
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", default="logs", help="logs dir (default ./logs)")
    ap.add_argument("--out", default="field_map.jpg", help="output mosaic")
    args = ap.parse_args()

    frames = load_frames(args.logs)
    if not frames:
        sys.exit(f"no geotagged frames found under {args.logs}/drone_*/")

    # Ground footprint and scale from the first frame's altitude
    alts = [m["alt"] for _, m in frames if m.get("alt")]
    alt = float(np.median(alts)) if alts else 30.0
    footprint_w = 2.0 * alt * math.tan(HFOV_RAD / 2.0)        # metres
    m_per_px = footprint_w / IMG_W

    xys = [latlon_to_xy(m["lat"], m["lon"]) for _, m in frames]
    xs = [p[0] for p in xys]
    ys = [p[1] for p in xys]
    margin = footprint_w / 2.0 + 5.0
    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin

    canvas_w = int((max_x - min_x) / m_per_px)
    canvas_h = int((max_y - min_y) / m_per_px)
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    n_pasted = 0
    n_marks = 0
    for (img_path, meta), (x, y) in zip(frames, xys):
        img = cv2.imread(img_path)
        if img is None:
            continue
        img = cv2.resize(img, (int(footprint_w / m_per_px),
                               int((footprint_w * IMG_H / IMG_W) / m_per_px)))
        h, w = img.shape[:2]
        # canvas: +x right (east), +y DOWN — so north must be flipped
        cx = int((x - min_x) / m_per_px)
        cy = canvas_h - int((y - min_y) / m_per_px)
        x0, y0 = cx - w // 2, cy - h // 2
        x1, y1 = x0 + w, y0 + h
        sx0, sy0 = max(0, -x0), max(0, -y0)
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(canvas_w, x1), min(canvas_h, y1)
        if x1 <= x0 or y1 <= y0:
            continue
        canvas[y0:y1, x0:x1] = img[sy0:sy0 + (y1 - y0), sx0:sx0 + (x1 - x0)]
        n_pasted += 1

        # Mark georeferenced ArUco detections on the mosaic
        for det in meta.get("aruco_detections", []):
            cv2.circle(canvas, (cx, cy), 12, (0, 0, 255), 2)
            cv2.putText(canvas, f"aruco {det['id']}", (cx + 15, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            n_marks += 1

    cv2.imwrite(args.out, canvas)
    print(f"mosaic: {args.out} ({canvas_w}x{canvas_h}px, {m_per_px:.3f} m/px) "
          f"from {n_pasted} frames, {n_marks} aruco marks")


if __name__ == "__main__":
    main()
