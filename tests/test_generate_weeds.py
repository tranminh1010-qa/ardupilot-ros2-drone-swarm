#!/usr/bin/env python3
"""Tests for the weed_field generator. Run directly: python3 tests/test_generate_weeds.py"""
import importlib.util
import json
import os
import tempfile
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location(
    "generate_weeds", os.path.join(REPO, "src", "custom_gz", "worlds", "generate_weeds.py"))
gw = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gw)

CENTER = (45.0, 50.0)


def test_deterministic_for_same_seed():
    a = gw.generate_weeds(12, 42, CENTER, 80.0)
    b = gw.generate_weeds(12, 42, CENTER, 80.0)
    assert a == b, "same seed must give identical weeds"
    c = gw.generate_weeds(12, 43, CENTER, 80.0)
    assert a != c, "different seed must give different weeds"


def test_count_area_and_diameter_bounds():
    weeds = gw.generate_weeds(12, 42, CENTER, 80.0)
    assert len(weeds) == 12
    for w in weeds:
        assert 45.0 - 40.0 <= w["x"] <= 45.0 + 40.0
        assert 50.0 - 40.0 <= w["y"] <= 50.0 + 40.0
        assert 1.2 <= w["diameter"] <= 2.5


def test_latlon_offsets():
    # A weed exactly at the field origin (Gazebo 45,50) must map to LAT/LON_BASE.
    lat, lon = gw.gazebo_to_latlon(45.0, 50.0, CENTER[0], CENTER[1])
    assert abs(lat - 40.072842) < 1e-9
    assert abs(lon - (-105.230575)) < 1e-9
    # 111.111 m north ≈ +0.001 deg latitude.
    lat2, _ = gw.gazebo_to_latlon(45.0, 50.0 + 111.111, CENTER[0], CENTER[1])
    assert abs((lat2 - lat) - 0.001) < 1e-5


def test_model_sdf_is_valid_and_visual_only():
    weeds = gw.generate_weeds(12, 42, CENTER, 80.0)
    root = ET.fromstring(gw.model_sdf(weeds))
    model = root.find("model")
    assert model.get("name") == "weed_field"
    assert model.find("static").text == "true"
    links = model.findall("link")
    assert len(links) == 12
    assert not list(root.iter("collision")), "weeds must be visual-only"
    for link in links:
        vis = link.find("visual")
        mat = vis.find("material")
        for tag in ("ambient", "diffuse", "emissive"):
            assert mat.find(tag).text.split() == ["1", "0", "1", "1"]


def test_write_model_outputs():
    weeds = gw.generate_weeds(12, 42, CENTER, 80.0)
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "weed_field")
        gw.write_model(weeds, out, {"seed": 42, "count": 12, "center": list(CENTER), "area": 80.0})
        for f in ("model.sdf", "model.config", "ground_truth.json"):
            assert os.path.isfile(os.path.join(out, f)), f
        with open(os.path.join(out, "ground_truth.json")) as fh:
            gt = json.load(fh)
        assert gt["seed"] == 42
        assert len(gt["weeds"]) == 12
        assert gt["weeds"] == weeds


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"PASS {name}")
    print("ALL PASS")
