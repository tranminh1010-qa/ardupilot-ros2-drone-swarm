#!/usr/bin/env python3
import json
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

from scripts.points_distributor import generate_grid_waypoints, split_by_sector, split_by_grid


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
            ros_domain_id = '0'
            rmw_implementation = str(context.launch_configurations.get('rmw_implementation','rmw_fastrtps_cpp'))
            base_mavlink_port = 14550
            base_ros_port = 14551

            action_list = []

            # Generate waypoints and distribute to drones
            wps = generate_grid_waypoints(field_size=80.0, num_points=12, height=30.0)
            action_list.append(LogInfo(msg=f"Generated {len(wps)} waypoints for {num_drones} drones."))
            chunks = split_by_sector(wps, num_drones)
            # Create ROS nodes for each drone
            drone_nodes = []
            for i in range(num_drones):
                drone_id = i + 1
                instance = i
                mavlink_port = base_mavlink_port + (i * 10)
                ros_port = base_ros_port + (i * 10)
                wp = chunks[i] if i < len(chunks) else wps[:4]  # Fallback waypoints
                wp_json = json.dumps(wp)
                # Create drone node - connects to existing SITL instance

                drone_node = Node(
                    package='swarm_control',
                    executable='base_drone_dds.py',
                    name=f'drone{drone_id}',
                    output='screen',
                    parameters=[{
                        'drone_id': drone_id,
                        'mavlink_connection': f'udp:localhost:{ros_port}',
                        'assigned_waypoints': wp_json,
                        'instance': instance,
                        'mavlink_port': mavlink_port,
                        'ros_port': ros_port,
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