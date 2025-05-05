#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from ament_index_python.packages import get_package_share_directory

from scripts.points_distributor import generate_grid_waypoints, split_by_sector
from utility.launch_utils import LaunchUtils
from scripts.command_generator import CommandGenerator


def generate_launch_description():
    # Declare launch arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones to launch'
    )

    # Launch Configuration
    LaunchConfiguration('num_drones')

    def launch_setup(context, *args, **kwargs):
        try:
            # Convert to integer
            num_drones = int(context.launch_configurations['num_drones'])

            bringup_dir = get_package_share_directory('ardupilot_gazebo')
            sdf_path = os.path.join(bringup_dir, 'models', 'iris_with_ardupilot', 'model.sdf')
            param_file = "/root/ardu_ws/src/swarm_control/parameters/ardu_gps_noise.parm"
            host_address = '172.17.0.1'
            ros_domain_id = '1'

            # Initialize utility classes
            utils = LaunchUtils(sdf_path, param_file)
            cmd_gen = CommandGenerator(host_address, param_file)

            utils.verify_paths()
            print(f"Using parameter file: {param_file}")
            common_env = cmd_gen.get_common_env(ros_domain_id)

            action_list = []

            build_action = ExecuteProcess(
                cmd=cmd_gen.get_build_cmd(),
                output='screen',
                shell=True
            )

            action_list.append(LogInfo(msg=f"Building Ardupilot for {num_drones} drones..."))
            action_list.append(build_action)

            # Generate waypoints
            wps = generate_grid_waypoints(field_size=60.0, num_points=6, height=8.0)
            action_list.append(LogInfo(msg=f"Generated {len(wps)} waypoints for {num_drones} drones."))
            chunks = split_by_sector(wps, num_drones)

            # Setup notification for build completion
            action_list.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=build_action,
                        on_exit=[LogInfo(msg=f"Build complete. Starting {num_drones} SITL instances...")]
                    )
                )
            )

            sitl_actions = []
            drone_nodes = []

            # Start SITL instances
            for i in range(num_drones):
                drone_id = i + 1
                instance = i
                ros_port = 14551 + (i * 10)
                gazebo_port = 14550 + (i * 10)
                wp = chunks[i]

                sitl_cmd = cmd_gen.get_sitl_cmd(instance, drone_id, ros_port, gazebo_port)

                sitl_action = ExecuteProcess(
                    cmd=sitl_cmd,
                    output='screen',
                    shell=True,
                    additional_env=common_env
                )

                if i == 0:
                    action_list.append(
                        RegisterEventHandler(
                            OnProcessExit(
                                target_action=build_action,
                                on_exit=[sitl_action]
                            )
                        )
                    )
                else:
                    action_list.append(
                        TimerAction(
                            period=3.0 * i,
                            actions=[sitl_action]
                        )
                    )
                sitl_actions.append(sitl_action)
                drone_node = Node(
                    package='swarm_control',
                    executable='base_drone.py',
                    name=f'drone{drone_id}',
                    output='screen',
                    parameters=[{
                        'drone_id': drone_id,
                        'mavlink_connection': f'udp:localhost:{ros_port}',
                        'assigned_waypoints': str(wp),
                    }]
                )

                drone_nodes.append(drone_node)

            # Start ROS nodes with delay
            for i, drone_node in enumerate(drone_nodes):
                action_list.append(
                    TimerAction(
                        period=30.0 + (i * 2.0),
                        actions=[
                            LogInfo(msg=f"Starting ROS node for drone {i + 1}..."),
                            drone_node
                        ]
                    )
                )

            monitor_action = ExecuteProcess(
                cmd=cmd_gen.get_monitor_cmd(),
                output='screen',
                shell=True
            )

            action_list.append(
                TimerAction(
                    period=35.0,
                    actions=[monitor_action]
                )
            )

            return action_list

        except Exception as e:
            print(f"Error in launch setup: {str(e)}")
            return [LogInfo(msg=f"Error in launch setup: {str(e)}")]
    launch_actions = [num_drones_arg, OpaqueFunction(function=launch_setup)]

    return LaunchDescription(launch_actions)