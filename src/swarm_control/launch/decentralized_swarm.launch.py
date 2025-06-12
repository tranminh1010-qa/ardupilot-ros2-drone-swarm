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
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones to launch'
    )

    LaunchConfiguration('num_drones')

    def launch_setup(context, *args, **kwargs):
        try:
            num_drones = int(context.launch_configurations['num_drones'])

            bringup_dir = get_package_share_directory('ardupilot_gazebo')
            sdf_path = os.path.join(bringup_dir, 'models', 'iris_with_ardupilot', 'model.sdf')
            param_file = "/root/ardu_ws/src/swarm_control/parameters/multi_swarm.parm"
            host_address = '172.17.0.1'
            ros_domain_id = '1'

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

            wps = generate_grid_waypoints(field_size=60.0, num_points=6, height=8.0)
            action_list.append(LogInfo(msg=f"Generated {len(wps)} waypoints for {num_drones} drones."))
            chunks = split_by_sector(wps, num_drones)

            action_list.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=build_action,
                        on_exit=[LogInfo(msg=f"Build complete. Starting {num_drones} SITL instances...")]
                    )
                )
            )

            setup_dirs_script = f"""
            for i in $(seq 0 {num_drones - 1}); do
                mkdir -p /tmp/sitl_$i
                rm -rf /tmp/sitl_$i/*
            done
            """

            setup_dirs_action = ExecuteProcess(
                cmd=['bash', '-c', setup_dirs_script],
                output='screen'
            )
            action_list.append(setup_dirs_action)

            sitl_actions = []
            drone_nodes = []

            for i in range(num_drones):
                drone_id = i + 1
                instance = i
                ros_port = 14551 + (i * 10)
                gazebo_port = 9002 + (i * 10)
                mavlink_port = 5760 + (i * 10)
                wp = chunks[i]

                sitl_cmd = [
                    'sim_vehicle.py',
                    '-v', 'ArduCopter',
                    '-f', 'gazebo-iris',
                    '--model', 'JSON',
                    f'--instance={instance}',
                    f'--sysid={drone_id}',
                    '--no-rebuild',
                    '-w',
                    '--speedup=1',
                    f'--sim-address={host_address}:{gazebo_port}',
                    '--custom-location=40.072842,-105.230575,1586,0',
                    f'--out=udp:127.0.0.1:{ros_port}',
                    f'--out=udp:{host_address}:{gazebo_port}',
                    '--add-param-file', param_file,
                    #'--console',
                    #'--map',
                ]

                sitl_env = common_env.copy()
                sitl_env.update({
                    'HOME': f'/tmp/sitl_{instance}',
                    'SITL_INSTANCE': str(instance)
                })

                sitl_action = ExecuteProcess(
                    cmd=sitl_cmd,
                    output='screen',
                    additional_env=sitl_env
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
                            period=8.0 * i,
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

            for i, drone_node in enumerate(drone_nodes):
                action_list.append(
                    TimerAction(
                        period=25.0 + (i * 5.0),
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
                    period=30.0,
                    actions=[monitor_action]
                )
            )

            return action_list

        except Exception as e:
            print(f"Error in launch setup: {str(e)}")
            return [LogInfo(msg=f"Error in launch setup: {str(e)}")]

    launch_actions = [num_drones_arg, OpaqueFunction(function=launch_setup)]

    return LaunchDescription(launch_actions)