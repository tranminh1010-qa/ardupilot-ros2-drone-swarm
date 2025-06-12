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
        host_address = '172.17.0.1'

        if not os.path.exists(sdf_path):
            raise FileNotFoundError(f"SDF file not found: {sdf_path}")

        if not os.path.exists(param_file):
            raise FileNotFoundError(f"Parameter file not found: {param_file}")

        print(f"Using parameter file: {param_file}")

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
                '--speedup', '2',
                '--sim-address=' + host_address,
                '--custom-location=40.072842,-105.230575,1586,0',
                '--out=udp:127.0.0.1:14551',  # Local connection for ROS
                '--out=udp:172.17.0.1:14550',  # Connection to Gazebo
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
                '--speedup', '2',
                '--sim-address=' + host_address,
                '--custom-location=40.072842,-105.230575,1586,0',
                '--out=udp:127.0.0.1:14561',  # Local connection for ROS
                '--out=udp:172.17.0.1:14560',  # Connection to Gazebo
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
                '--speedup', '2',
                '--sim-address=' + host_address,
                '--custom-location=40.072842,-105.230575,1586,0',
                '--out=udp:127.0.0.1:14571',  # Changed to unique port for ROS
                '--out=udp:172.17.0.1:14570',  # Connection to Gazebo
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

        # ROS-Gazebo bridge with reliable QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        bridge = GroupAction([
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
                parameters=[{'qos_overrides./clock.reliability': 'reliable'}]
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
                output='screen',
                parameters=[{'qos_override.*/reliability': 'reliable'}]
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
                'connection_retry_attempts': 5,
                'connection_timeout': 10.0
            }],
            remappings=[('/leader_drone_node/position', '/leader_position')],
        )

        # Follower drone nodes with corrected ports
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
                'follow_distance': 7.0,
                'connection_retry_attempts': 5,
                'connection_timeout': 10.0
            }],
        )

        follower_node2 = Node(
            package='swarm_control',
            executable='follower_drone.py',
            name='follower_drone_node_2',
            output='screen',
            parameters=[{
                'drone_id': 2,
                'mavlink_connection': 'udp:localhost:14571',  # Updated to match SITL port
                'leader_pos_topic': '/leader_position',
                'offset': [-2.0, 2.0, 0.0],
                'follow_distance': 7.0,
                'connection_retry_attempts': 5,
                'connection_timeout': 10.0
            }],
        )

        # Construct launch description with appropriate delays
        return LaunchDescription([
            LogInfo(msg="Starting SITL instances..."),
            TimerAction(period=5.0, actions=[ardupilot_sitl_leader]),
            TimerAction(period=10.0, actions=[ardupilot_sitl_follower1]),
            TimerAction(period=15.0, actions=[ardupilot_sitl_follower2]),
            LogInfo(msg="Starting ROS-Gazebo bridge..."),
            TimerAction(period=20.0, actions=[bridge]),
            LogInfo(msg="Starting ROS nodes..."),
            TimerAction(period=35.0, actions=[leader_node]),
            TimerAction(period=40.0, actions=[follower_node1]),
            TimerAction(period=45.0, actions=[follower_node2])
        ])
    except Exception as e:
        print(f"Error in launch file: {str(e)}")
        return LaunchDescription([])