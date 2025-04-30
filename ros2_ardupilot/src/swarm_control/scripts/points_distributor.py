import numpy as np

def split_waypoints(waypoints, num_drones):
    chunks = []
    for i in range(num_drones):
        start = i * (len(waypoints) // num_drones)
        end = start + (len(waypoints) // num_drones) + (1 if i < len(waypoints) % num_drones else 0)
        if not is_odd(start):
            start -= 1
        if is_odd(end) and end != len(waypoints):
            end += 1
        chunks.append(waypoints[start:end])
    return chunks

def is_odd(num):
    return num % 2 != 0

def generate_circular_waypoints():
    radius = 30.0
    height = 8.0
    points = 12  # Number of points in the circle
    waypoints = []
    for i in range(points):
        angle = 2 * np.pi * i / points
        x = float(radius * np.cos(angle))
        y = float(radius * np.sin(angle))
        waypoints.append((x, y, height))
    return waypoints