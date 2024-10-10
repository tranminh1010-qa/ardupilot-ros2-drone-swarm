#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def generate_launch_description():
    try:
        # Verify the correct path for the iris_quadcopter model
        bringup_dir = get_package_share_directory('ardupilot_gazebo')
        sdf_path = os.path.join(bringup_dir, 'models', 'iris_with_ardupilot', 'model.sdf')
        swarm_control_share = get_package_share_directory('swarm_control')
        param_file = "/root/ardu_ws/src/swarm_control/parameters/ardu_gps_noise.parm"

        if not os.path.exists(sdf_path):
            raise FileNotFoundError(f"SDF file not found: {sdf_path}")

        if not os.path.exists(param_file):
             raise FileNotFoundError(f"Parameter file not found: {param_file}")

        print(f"Using parameter file: {param_file}")

        # Launch Gazebo
        gz_sim = ExecuteProcess(
            cmd=['gz', 'sim', '-v4', '-r', 'swarm_drone.sdf'],
            output='screen'
        )

        # Launch ArduPilot SITL instances
        ardupilot_sitl_leader = ExecuteProcess(
            cmd=[
                'sim_vehicle.py',
                '-v', 'ArduCopter',
                '-f', 'gazebo-iris',
                '--model', 'JSON',
                '--console',
                '--instance', '0',
                '--sysid', '1',
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
                '-f', 'gazebo-iris',
                '--model', 'JSON',
                '--console',
                '--instance', '1',
                '--sysid', '2',
                '--custom-location=40.072842,-105.230575,1586,0',
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
                '-f', 'gazebo-iris',
                '--model', 'JSON',
                '--console',
                '--instance', '2',
                '--sysid', '3',
                '--custom-location=40.072842,-105.230575,1586,0',
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
                'drone_id': 0,
                'mavlink_connection': 'udp:localhost:14551',
            }],
            remappings=[('/leader_drone_node/position', '/leader_position')],
        )

        follower_node1 = Node(
            package='swarm_control',
            executable='follower_drone.py',
            name='follower_drone_node_1',
            output='screen',
            parameters=[{
                'drone_id': 1,
                'mavlink_connection': 'udp:localhost:14561',
                'leader_pos_topic': '/leader_position',
                'offset': [-2.0, -2.0, 0.0],
                'follow_distance': 7.0
            }],
        )

        follower_node2 = Node(
            package='swarm_control',
            executable='follower_drone.py',
            name='follower_drone_node_2',
            output='screen',
            parameters=[{
                'drone_id': 2,
                'mavlink_connection': 'udp:localhost:14571',
                'leader_pos_topic': '/leader_position',
                'offset': [-2.0, 2.0, 0.0],
                'follow_distance': 7.0
            }],
        )

        return LaunchDescription([
            gz_sim,
            LogInfo(msg="Starting SITL instances..."),
            TimerAction(period=5.0, actions=[ardupilot_sitl_leader]),
            TimerAction(period=7.0, actions=[ardupilot_sitl_follower1]),
            TimerAction(period=10.0, actions=[ardupilot_sitl_follower2]),
            LogInfo(msg="Spawning drones in Gazebo..."),
            TimerAction(period=12.0, actions=[bridge]),
            LogInfo(msg="Starting ROS nodes..."),
            TimerAction(period=30.0, actions=[leader_node]),
            TimerAction(period=32.0, actions=[follower_node1]),
            TimerAction(period=34.0, actions=[follower_node2])
        ])
    except Exception as e:
        print(f"Error in launch file: {str(e)}")
        return LaunchDescription([])