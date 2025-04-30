#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from scripts.points_distributor import split_waypoints, generate_circular_waypoints


def generate_launch_description():
    try:
        __num_drones = 3 #Modifying this value requires modifying the gazebo.sdf
        # Verify paths
        bringup_dir = get_package_share_directory('ardupilot_gazebo')
        sdf_path = os.path.join(bringup_dir, 'models', 'iris_with_ardupilot', 'model.sdf')
        param_file = "/root/ardu_ws/src/swarm_control/parameters/ardu_gps_noise.parm"
        host_address = '172.17.0.1'

        # Check if required files exist
        for path in [sdf_path, param_file]:
            if not os.path.exists(path):
                raise FileNotFoundError(f"Required file not found: {path}")

        print(f"Using parameter file: {param_file}")

        # Common environment variables for SITL instances
        common_env = {
            'ARDU_SIM_PROVIDED': 'gz',
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/root/ardu_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo/',
            'GZ_SIM_RESOURCE_PATH': '/root/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/',
        }

        # Launch components with appropriate delays
        launch_sequence = [LogInfo(msg="Starting SITL instances...")]
        wps = generate_circular_waypoints()
        chunks = split_waypoints(wps, __num_drones)
        # Create SITL instances and drone nodes for each drone
        for i in range(__num_drones):
            drone_id = i + 1
            instance = i
            ros_port = 14551 + (i * 10)
            gazebo_port = 14550 + (i * 10)
            wp = chunks[i]

            # SITL instance
            sitl_cmd = [
                'sim_vehicle.py',
                '-v', 'ArduCopter',
                '-f', 'gazebo-iris',
                '--model', 'JSON',
                '--console',
                f'--instance', str(instance),
                f'--sysid', str(drone_id),
                '--speedup', '2',
                f'--sim-address={host_address}',
                '--custom-location=40.072842,-105.230575,1586,0',
                f'--out=udp:127.0.0.1:{ros_port}',
                f'--out=udp:172.17.0.1:{gazebo_port}',
                '--add-param-file', param_file,
            ]

            sitl_action = ExecuteProcess(
                cmd=sitl_cmd,
                output='screen',
                shell=True,
                additional_env=common_env
            )

            # Add SITL instance with delay
            launch_sequence.append(TimerAction(
                period=5.0 + (i * 5.0),
                actions=[sitl_action]
            ))

            # Drone ROS node
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

            # Add drone node with delay
            launch_sequence.append(TimerAction(
                period=35.0 + (i * 5.0),
                actions=[drone_node]
            ))

        # Add log message between SITL and ROS node launches
        launch_sequence.insert(5, LogInfo(msg="Starting ROS nodes..."))

        return LaunchDescription(launch_sequence)

    except Exception as e:
        print(f"Error in launch file: {str(e)}")
        return LaunchDescription([])