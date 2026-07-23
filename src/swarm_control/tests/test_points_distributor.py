#!/usr/bin/env python3
"""Tests for the waypoint splitter: every drone must get a balanced 1/n share
of the field with zero dropped waypoints, for ANY drone count.

Run directly:  python3 src/swarm_control/tests/test_points_distributor.py
"""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from points_distributor import generate_grid_waypoints, split_serpentine


def check(num_drones, waypoints):
    chunks = split_serpentine(waypoints, num_drones)
    n_wps = len(waypoints)

    # Always exactly num_drones chunks (launch file indexes chunks[i])
    assert len(chunks) == num_drones, \
        f"n={num_drones}: expected {num_drones} chunks, got {len(chunks)}"

    # No waypoint dropped, none duplicated
    flat = [wp for c in chunks for wp in c]
    assert len(flat) == n_wps, \
        f"n={num_drones}: {n_wps - len(flat)} waypoints dropped"
    assert len(set(flat)) == len(set(waypoints)), \
        f"n={num_drones}: duplicated waypoints"

    # Balanced 1/n: chunk sizes differ by at most 1
    sizes = [len(c) for c in chunks]
    assert max(sizes) - min(sizes) <= 1, \
        f"n={num_drones}: unbalanced sizes {sizes}"


def test_balance_and_coverage():
    wps = generate_grid_waypoints(field_size=80.0, grid_points=5, height=30.0)
    assert len(wps) == 25
    for n in (1, 2, 3, 4, 5, 6, 7, 8, 12, 24, 25):
        check(n, wps)


def test_more_drones_than_waypoints():
    wps = generate_grid_waypoints(field_size=20.0, grid_points=2, height=10.0)  # 4 wps
    chunks = split_serpentine(wps, 6)
    assert len(chunks) == 6                       # indexable for every drone
    assert sum(len(c) for c in chunks) == 4       # all waypoints assigned
    assert all(len(c) <= 1 for c in chunks)


def test_serpentine_order_is_efficient():
    """Within a chunk, consecutive waypoints must be adjacent grid cells
    (sweep pattern) — no diagonal jumps across the field."""
    wps = generate_grid_waypoints(field_size=80.0, grid_points=5, height=30.0)
    spacing = 80.0 / 4  # 20 m between grid points
    for chunk in split_serpentine(wps, 4):
        for a, b in zip(chunk, chunk[1:]):
            d = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
            assert d <= spacing * 1.01, \
                f"non-adjacent hop {d:.1f} m between {a} and {b}"


if __name__ == '__main__':
    test_balance_and_coverage()
    test_more_drones_than_waypoints()
    test_serpentine_order_is_efficient()
    print("ALL SPLITTER TESTS PASSED")
