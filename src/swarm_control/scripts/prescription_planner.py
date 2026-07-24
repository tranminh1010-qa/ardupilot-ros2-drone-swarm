#!/usr/bin/env python3
"""Prescription planner: turn mapper weed detections into targeted spray plans.

Runs inside the swarm_controller container alongside the drone nodes (started
by compose_decentralized.launch.py). File-based by design — the mappers'
CameraProcessor writes geotagged sidecars under /root/logs/drone_<id>/, and
this planner turns them into per-sprayer prescription files that the
SprayerDrone nodes poll for before arming:

  1. WAIT for the mapping survey: poll each mapper's sidecar JSONs until the
     expected frame count arrives (or --timeout expires — then plan with
     whatever was mapped).
  2. CLUSTER all georeferenced weed detections (greedy union within
     --cluster-radius metres) — adjacent frames/drones re-detect the same
     weed, clustering dedupes them into one spray target.
  3. SPLIT clusters among the sprayers: sorted along the field, contiguous
     balanced cut (like the mapping split), nearest-neighbour ordered within
     each share.
  4. WRITE logs/prescriptions/sprayer_<instance>.json with field-local
     waypoints (north, east, alt) — the same frame BaseDrone flies — plus a
     combined plan.geojson for GIS tooling.

Coordinates: detections carry lat/lon (nadir approximation). We convert to
field-local metres around the shared field origin with the same equirectangular
approximation the detector used, so the round trip is consistent.
"""
import argparse
import glob
import json
import math
import os
import time

FIELD_ORIGIN_LAT = 40.072842
FIELD_ORIGIN_LON = -105.230575
M_PER_DEG_LAT = 111111.0


def latlon_to_ne(lat, lon):
    north = (lat - FIELD_ORIGIN_LAT) * M_PER_DEG_LAT
    east = (lon - FIELD_ORIGIN_LON) * M_PER_DEG_LAT * math.cos(
        math.radians(FIELD_ORIGIN_LAT))
    return north, east


def ne_to_latlon(north, east):
    lat = FIELD_ORIGIN_LAT + north / M_PER_DEG_LAT
    lon = FIELD_ORIGIN_LON + east / (
        M_PER_DEG_LAT * math.cos(math.radians(FIELD_ORIGIN_LAT)))
    return lat, lon


def load_detections(logs_dir, mapper_ids):
    """All georeferenced weed detections from the mappers' sidecars."""
    dets = []
    frames = 0
    for mid in mapper_ids:
        for meta_path in sorted(
                glob.glob(os.path.join(logs_dir, f"drone_{mid}", "wp_*.json"))):
            try:
                with open(meta_path) as f:
                    meta = json.load(f)
            except (json.JSONDecodeError, OSError):
                continue  # sidecar mid-write; next poll gets it
            frames += 1
            for d in meta.get("weed_detections", []):
                if d.get("lat") is not None and d.get("lon") is not None:
                    n, e = latlon_to_ne(d["lat"], d["lon"])
                    dets.append({"n": n, "e": e, "area_px": d.get("area_px", 0)})
    return dets, frames


def cluster(dets, radius):
    """Greedy distance clustering; returns cluster dicts with centroids."""
    clusters = []
    for d in dets:
        best = None
        for c in clusters:
            dist = math.hypot(c["n"] - d["n"], c["e"] - d["e"])
            if dist <= radius and (best is None or dist < best[0]):
                best = (dist, c)
        if best is None:
            clusters.append({"n": d["n"], "e": d["e"],
                             "num_detections": 1,
                             "total_area_px": d["area_px"]})
        else:
            c = best[1]
            k = c["num_detections"]
            c["n"] = (c["n"] * k + d["n"]) / (k + 1)
            c["e"] = (c["e"] * k + d["e"]) / (k + 1)
            c["num_detections"] = k + 1
            c["total_area_px"] += d["area_px"]
    return clusters


def nn_order(points):
    """Nearest-neighbour ordering starting from the first point."""
    if not points:
        return []
    rem = points[:]
    path = [rem.pop(0)]
    while rem:
        cur = path[-1]
        best = min(range(len(rem)), key=lambda i: math.hypot(
            rem[i]["n"] - cur["n"], rem[i]["e"] - cur["e"]))
        path.append(rem.pop(best))
    return path


def split_balanced(clusters, n):
    """Sort along the field, contiguous balanced cut into n shares."""
    ordered = sorted(clusters, key=lambda c: (c["n"], c["e"]))
    base, extra = divmod(len(ordered), n)
    shares, start = [], 0
    for i in range(n):
        size = base + (1 if i < extra else 0)
        shares.append(nn_order(ordered[start:start + size]))
        start += size
    return shares


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", default="/root/logs")
    ap.add_argument("--sprayers", nargs="+", type=int, required=True,
                    help="0-indexed sprayer instances, e.g. 2 3 4 5")
    ap.add_argument("--expect", nargs="+", default=[],
                    help="mapper drone_id:frames pairs, e.g. 1:13 2:12")
    ap.add_argument("--timeout", type=float, default=420.0,
                    help="max seconds to wait for the survey to finish")
    ap.add_argument("--cluster-radius", type=float, default=3.0)
    ap.add_argument("--spray-alt", type=float, default=15.0)
    args = ap.parse_args()

    expect = {}
    for pair in args.expect:
        drone_id, frames = pair.split(":")
        expect[int(drone_id)] = int(frames)
    mapper_ids = sorted(expect)
    total_expected = sum(expect.values())

    print(f"[planner] waiting for survey: {expect} "
          f"(timeout {args.timeout:.0f}s)", flush=True)
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        dets, frames = load_detections(args.logs, mapper_ids)
        if total_expected and frames >= total_expected:
            print(f"[planner] survey complete: {frames} frames", flush=True)
            break
        time.sleep(10)
    else:
        print(f"[planner] TIMEOUT - planning with {frames}/{total_expected} "
              f"frames", flush=True)

    dets, frames = load_detections(args.logs, mapper_ids)
    clusters = cluster(dets, args.cluster_radius)
    print(f"[planner] {len(dets)} detections in {frames} frames -> "
          f"{len(clusters)} spray targets", flush=True)

    out_dir = os.path.join(args.logs, "prescriptions")
    os.makedirs(out_dir, exist_ok=True)

    shares = split_balanced(clusters, len(args.sprayers))
    now = time.time()
    for inst, share in zip(args.sprayers, shares):
        plan = {
            "generated_unix": now,
            "sprayer_instance": inst,
            "field_origin": [FIELD_ORIGIN_LAT, FIELD_ORIGIN_LON],
            "num_targets": len(share),
            # BaseDrone waypoint frame: (north, east, alt) around field origin
            "waypoints": [[round(c["n"], 2), round(c["e"], 2), args.spray_alt]
                          for c in share],
            "clusters": [
                {**{k: round(v, 3) if isinstance(v, float) else v
                    for k, v in c.items()},
                 "lat": round(ne_to_latlon(c["n"], c["e"])[0], 8),
                 "lon": round(ne_to_latlon(c["n"], c["e"])[1], 8)}
                for c in share],
        }
        path = os.path.join(out_dir, f"sprayer_{inst}.json")
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(plan, f, indent=2)
        os.replace(tmp, path)  # atomic: sprayers never see a partial file
        print(f"[planner] wrote {path}: {len(share)} targets", flush=True)

    # Combined GeoJSON for GIS tooling (QGIS / PostGIS import)
    features = []
    for inst, share in zip(args.sprayers, shares):
        for c in share:
            lat, lon = ne_to_latlon(c["n"], c["e"])
            features.append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [lon, lat]},
                "properties": {"sprayer_instance": inst,
                               "num_detections": c["num_detections"],
                               "total_area_px": c["total_area_px"]},
            })
    with open(os.path.join(out_dir, "plan.geojson"), "w") as f:
        json.dump({"type": "FeatureCollection", "features": features}, f,
                  indent=2)
    print(f"[planner] wrote plan.geojson ({len(features)} targets). Done.",
          flush=True)


if __name__ == "__main__":
    main()
