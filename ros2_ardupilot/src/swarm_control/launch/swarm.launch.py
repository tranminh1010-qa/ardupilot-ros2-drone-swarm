#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    try:
        # Verify the correct path for the iris_quadcopter model
        bringup_dir = get_package_share_directory('ardupilot_gazebo')
        sdf_path = os.path.join(bringup_dir, 'models', 'iris_with_standoffs', 'model.sdf')
        swarm_control_share = get_package_share_directory('swarm_control')
        param_file = os.path.join(swarm_control_share, 'parameters', 'ardu_gps_noise.param')

        if not os.path.exists(sdf_path):
            raise FileNotFoundError(f"SDF file not found: {sdf_path}")

        if not os.path.exists(param_file):
            raise FileNotFoundError(f"Parameter file not found: {param_file}")

        print(f"Using parameter file: {param_file}")

        # Launch Gazebo
        gz_sim = ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        )

        # Launch ArduPilot SITL instances
        ardupilot_sitl_leader = ExecuteProcess(
            cmd=[
                'sim_vehicle.py',
                '-v', 'ArduCopter',
                '--model', 'JSON',
                '--console',
                '--instance', '0',
                '--custom-location=40.072842,-105.230575,1586,0',
                '--out=udp:127.0.0.1:14550',
                '--out=udp:127.0.0.1:14551',
                '--add-param-file', param_file,
            ],
            output='screen',
            shell=True,
            additional_env={
                'ARDU_SIM_PROVIDED': 'gz',
                'GZ_SIM_SYSTEM_PLUGIN_PATH': '/root/ardu_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo/',
                'GZ_SIM_RESOURCE_PATH': '/root/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/',
            }
        )

        ardupilot_sitl_follower1 = ExecuteProcess(
            cmd=[
                'sim_vehicle.py',
                '-v', 'ArduCopter',
                '--model', 'JSON',
                '--console',
                '--instance', '1',
                '--out=udp:127.0.0.1:14560',
                '--out=udp:127.0.0.1:14561',
                '--add-param-file', param_file,
            ],
            output='screen',
            shell=True,
            additional_env={
                'ARDU_SIM_PROVIDED': 'gz',
                'GZ_SIM_SYSTEM_PLUGIN_PATH': '/root/ardu_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo/',
                'GZ_SIM_RESOURCE_PATH': '/root/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/',
            }
        )

        ardupilot_sitl_follower2 = ExecuteProcess(
            cmd=[
                'sim_vehicle.py',
                '-v', 'ArduCopter',
                '--model', 'JSON',
                '--console',
                '--instance', '2',
                '--out=udp:localhost:14570',
                '--out=udp:localhost:14571',
                '--add-param-file', param_file,
            ],
            output='screen',
            shell=True,
            additional_env={
                'ARDU_SIM_PROVIDED': 'gz',
                'GZ_SIM_SYSTEM_PLUGIN_PATH': '/root/ardu_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo/',
                'GZ_SIM_RESOURCE_PATH': '/root/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/',
            }
        )

        # Spawn drones in Gazebo
        spawn_leader = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', sdf_path, '-name', 'leader_drone', '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        )

        spawn_follower1 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', sdf_path, '-name', 'follower_drone1', '-x', '-2', '-y', '0', '-z', '0.1'],
            output='screen'
        )

        spawn_follower2 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', sdf_path, '-name', 'follower_drone2', '-x', '-4', '-y', '0', '-z', '0.1'],
            output='screen'
        )

        # ROS-Gazebo bridge
        bridge = GroupAction([
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/model/leader_drone/pose@geometry_msgs/msg/Pose@gz.msgs.Pose',
                    '/model/leader_drone/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/follower_drone1/pose@geometry_msgs/msg/Pose@gz.msgs.Pose',
                    '/model/follower_drone1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/follower_drone2/pose@geometry_msgs/msg/Pose@gz.msgs.Pose',
                    '/model/follower_drone2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                ],
                output='screen'
            )
        ])

        # Leader drone node
        leader_node = Node(
            package='swarm_control',
            executable='leader_swarm.py',
            name='leader_drone_node',
            output='screen',
            parameters=[{
                'mavlink_connection': 'udp:localhost:14550',
            }]
        )

        # Follower drone nodes
        follower_node1 = Node(
            package='swarm_control',
            executable='follower_drone.py',
            name='follower_drone_node1',
            output='screen',
            parameters=[{
                'mavlink_connection': 'udp:localhost:14560',
                'leader_pos_topic': '/leader_drone_node/position',
                'offset': [-2.0, 0.0, 0.0]
            }]
        )

        follower_node2 = Node(
            package='swarm_control',
            executable='follower_drone.py',
            name='follower_drone_node2',
            output='screen',
            parameters=[{
                'mavlink_connection': 'udp:localhost:14570',
                'leader_pos_topic': '/leader_drone_node/position',
                'offset': [-4.0, 0.0, 0.0]
            }]
        )

        return LaunchDescription([
            gz_sim,
            LogInfo(msg="Starting SITL instances..."),
            TimerAction(period=5.0, actions=[ardupilot_sitl_leader]),
            TimerAction(period=15.0, actions=[ardupilot_sitl_follower1]),
            TimerAction(period=25.0, actions=[ardupilot_sitl_follower2]),
            LogInfo(msg="Spawning drones in Gazebo..."),
            TimerAction(period=35.0, actions=[spawn_leader, spawn_follower1, spawn_follower2]),
            TimerAction(period=40.0, actions=[bridge]),
            LogInfo(msg="Starting ROS nodes..."),
            TimerAction(period=180.0, actions=[leader_node, follower_node1, follower_node2])
        ])
    except Exception as e:
        print(f"Error in launch file: {str(e)}")
        return LaunchDescription([])