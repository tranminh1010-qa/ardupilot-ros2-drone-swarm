import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def generate_circular_waypoints():
    radius = 30.0
    height = 8.0
    points = 12  # Number of points in the circle
    wps = []
    for i in range(points):
        angle = 2 * np.pi * i / points
        x = float(radius * np.cos(angle))
        y = float(radius * np.sin(angle))
        wps.append((x, y, height))
    return wps

def generate_grid_waypoints(field_size=60.0, grid_points=6, height=8.0):
    """Generate a grid of waypoints"""
    all_waypoints = []
    spacing = field_size / (grid_points - 1)

    for i in range(grid_points):
        for j in range(grid_points):
            x = (i * spacing) - (field_size / 2)
            y = (j * spacing) - (field_size / 2)
            all_waypoints.append((float(x), float(y), height))

    return all_waypoints


def distance_between(wp1, wp2):
    """Calculate Euclidean distance between two waypoints"""
    return np.sqrt((wp2[0] - wp1[0]) ** 2 + (wp2[1] - wp1[1]) ** 2 + (wp2[2] - wp1[2]) ** 2)



def sort_waypoints_by_path(waypoints):
    """Sort waypoints to minimize path length (greedy nearest neighbor approach)"""
    if not waypoints:
        return []

    remaining = waypoints.copy()
    path = [remaining.pop(0)]  # Start with the first waypoint

    while remaining:
        current = path[-1]

        # Find the closest remaining waypoint
        best_idx = 0
        best_distance = distance_between(current, remaining[0])

        for i in range(1, len(remaining)):
            dist = distance_between(current, remaining[i])
            if dist < best_distance:
                best_distance = dist
                best_idx = i

        # Add the closest waypoint to the path
        path.append(remaining.pop(best_idx))

    return path


def split_by_grid(waypoints, num_drones, grid_dims=None):
    """
    Split waypoints by dividing the area into a grid and assigning
    each cell to a drone.
    """
    if num_drones >= len(waypoints):
        return [[wp] for wp in waypoints[:num_drones]]

    # Determine grid dimensions if not provided
    if grid_dims is None:
        # Try to make a square-ish grid
        grid_size = int(np.ceil(np.sqrt(num_drones)))
        grid_dims = (grid_size, grid_size)

    rows, cols = grid_dims

    # Find min and max coordinates
    x_coords = [wp[0] for wp in waypoints]
    y_coords = [wp[1] for wp in waypoints]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)

    # Add a small buffer to ensure all points are included
    buffer = 0.001
    width = max_x - min_x + buffer
    height = max_y - min_y + buffer

    # Calculate cell sizes
    cell_width = width / cols
    cell_height = height / rows

    # Initialize drone waypoints
    drone_waypoints = [[] for _ in range(num_drones)]

    # Assign waypoints to grid cells
    for wp in waypoints:
        # Determine which cell this waypoint belongs to
        col = min(cols - 1, int((wp[0] - min_x) / cell_width))
        row = min(rows - 1, int((wp[1] - min_y) / cell_height))

        # Convert 2D cell index to drone index
        drone_idx = row * cols + col

        # Handle case where we have more cells than drones
        if drone_idx < num_drones:
            drone_waypoints[drone_idx].append(wp)

    # Remove any empty assignments
    drone_waypoints = [wps for wps in drone_waypoints if wps]

    # Sort each drone's waypoints for efficient paths
    for i in range(len(drone_waypoints)):
        if drone_waypoints[i]:
            drone_waypoints[i] = sort_waypoints_by_path(drone_waypoints[i])

    return drone_waypoints


def plot_waypoints(waypoints_list, title):
    """Plot the waypoints for visualization"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']

    for drone_idx, waypoints in enumerate(waypoints_list):
        color = colors[drone_idx % len(colors)]

        x_coords = [point[0] for point in waypoints]
        y_coords = [point[1] for point in waypoints]
        z_coords = [point[2] for point in waypoints]

        ax.plot(x_coords, y_coords, z_coords, color=color, marker='o',
                linestyle='-', linewidth=2, markersize=5,
                label=f'Drone {drone_idx + 1}')

        # Add arrows to show direction
        for i in range(len(waypoints) - 1):
            x1, y1, z1 = waypoints[i]
            x2, y2, z2 = waypoints[i + 1]

            dx = x2 - x1
            dy = y2 - y1
            dz = z2 - z1

            length = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            dx, dy, dz = dx / length, dy / length, dz / length

            mid_x, mid_y, mid_z = (x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2
            ax.quiver(mid_x, mid_y, mid_z, dx, dy, dz, color=color,
                      length=5, arrow_length_ratio=0.2, normalize=True)

        # Mark start and end points
        ax.scatter(x_coords[0], y_coords[0], z_coords[0], color=color, s=100, marker='^',
                   label=f'Start {drone_idx + 1}')
        ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], color=color, s=100, marker='s',
                   label=f'End {drone_idx + 1}')

    # Set plot parameters
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title(title)

    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper right')

    plt.tight_layout()
    plt.show()


# Run the visualization to compare different splitting methods
if __name__ == "__main__":
    num_drones = 4
    waypoints = generate_grid_waypoints(field_size=60.0, grid_points=5, height=8.0)

    grid_split = split_by_grid(waypoints, num_drones)
    plot_waypoints(grid_split, "Grid-Based Split")