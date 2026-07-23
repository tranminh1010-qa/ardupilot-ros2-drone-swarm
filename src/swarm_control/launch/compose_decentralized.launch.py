#!/usr/bin/env python3
import json
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

from scripts.points_distributor import generate_grid_waypoints, split_serpentine

# Shared field origin — must match LAT_BASE/LON_BASE in
# src/scripts/start_drone_container.sh so the mapped field is centred on the
# swarm's home area. All drones convert waypoints with this ONE origin so the
# 1/n chunks tile a single field instead of shifting with each drone's home.
FIELD_ORIGIN_LAT = 40.072842
FIELD_ORIGIN_LON = -105.230575

# GstCameraPlugin stream ports: drone instance i streams H.264/RTP on
# CAMERA_PORT_BASE + i (see src/custom_gz/models/base_drone*/model.sdf).
CAMERA_PORT_BASE = 5600


def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='2',
        description='Number of drones to launch'
    )

    rmw_arg = DeclareLaunchArgument(
        'rmw_implementation',
        default_value='rmw_fastrtps_cpp',
        description='ROS MiddleWare Implementation'
    )

    # Docker Compose environment - SITL instances are already running
    def launch_setup(context, *args, **kwargs):
        try:
            num_drones = int(context.launch_configurations['num_drones'])
            LogInfo(msg = f"Launching ROS2 Node for num_drones:{num_drones}")
            rmw_implementation = str(context.launch_configurations.get('rmw_implementation','rmw_fastrtps_cpp'))
            base_mavlink_port = 14550
            base_ros_port = 14551

            action_list = []

            # Generate waypoints and distribute to drones: balanced serpentine
            # split — every drone gets a contiguous 1/n strip of the sweep,
            # sizes differ by at most one waypoint, nothing is dropped.
            wps = generate_grid_waypoints(field_size=80.0, grid_points=5, height=30.0)
            chunks = split_serpentine(wps, num_drones)
            action_list.append(LogInfo(msg=f"Generated {len(wps)} waypoints for {num_drones} drones."))
            # Create ROS nodes for each drone
            drone_nodes = []
            for i in range(num_drones):
                drone_id = i + 1
                instance = i
                # Each drone gets its own ROS_DOMAIN_ID (INSTANCE + 1)
                ros_domain_id = str(drone_id)
                mavlink_port = base_mavlink_port + (i * 10)
                ros_port = base_ros_port + (i * 10)
                wp = chunks[i]
                wp_json = json.dumps(wp)
                # Create drone node - connects to existing SITL instance

                # CameraDrone = BaseDrone extension with the mapping camera;
                # switch back to base_drone_dds.py to fly camera-less.
                drone_node = Node(
                    package='swarm_control',
                    executable='camera_drone_dds.py',
                    name=f'drone{drone_id}',
                    output='screen',
                    # Self-heal against EKF/GPS warmup races: a node that crashes
                    # during startup (arm/position/GUIDED wait) is relaunched and
                    # retries once the SITL instance's EKF is ready. Successful nodes
                    # call rclpy.spin() forever, so respawn only re-runs failures.
                    respawn=True,
                    respawn_delay=5.0,
                    parameters=[{
                        'drone_id': drone_id,
                        'mavlink_connection': f'udp:localhost:{ros_port}',
                        'assigned_waypoints': wp_json,
                        'instance': instance,
                        'mavlink_port': mavlink_port,
                        'ros_port': ros_port,
                        'field_origin_lat': FIELD_ORIGIN_LAT,
                        'field_origin_lon': FIELD_ORIGIN_LON,
                        'camera_port': CAMERA_PORT_BASE + instance,
                    }],
                    additional_env={
                        'ROS_DOMAIN_ID': ros_domain_id,
                        'RMW_IMPLEMENTATION': rmw_implementation
                    }
                )

                drone_nodes.append(drone_node)

            for i, drone_node in enumerate(drone_nodes):
                startup_delay = (i * 5.0)
                action_list.append(
                    TimerAction(
                        period=startup_delay,
                        actions=[
                            LogInfo(msg=f"Starting ROS node for drone {i + 1}..."),
                            drone_node
                        ]
                    )
                )

            return action_list

        except Exception as e:
            print(f"Error in launch setup: {str(e)}")
            return [LogInfo(msg=f"Error in launch setup: {str(e)}")]

    launch_actions = [num_drones_arg, rmw_arg,  OpaqueFunction(function=launch_setup)]

    return LaunchDescription(launch_actions)