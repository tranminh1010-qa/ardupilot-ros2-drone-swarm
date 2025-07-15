#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directories
    pkg_ardupilot_sitl = get_package_share_directory('ardupilot_sitl')
    pkg_ardupilot_gz_gazebo = get_package_share_directory('ardupilot_gz_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_swarm_control = get_package_share_directory('swarm_control')

    # Define drone-specific parameters
    drones = [
        {
            'instance': '0',
            'mavlink_connection': 'udp:localhost:14550',
            'sysid': '1',
            'name': 'drone0',
        },
        {
            'instance': '1',
            'mavlink_connection': 'udp:localhost:14560',
            'sysid': '2',
            'name': 'drone1',
        },
        {
            'instance': '2',
            'mavlink_connection': 'udp:localhost:14570',
            'sysid': '3',
            'name': 'drone2',
        }
    ]

    # Gazebo
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f"-v4 -s -r {os.path.join(pkg_ardupilot_gz_gazebo, 'worlds', 'iris_runway.sdf')}"
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': "-v4 -g"}.items(),
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_swarm_control, "rviz", "swarm.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # SITL instances
    sitl_instances = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_ardupilot_sitl, '/launch/sitl_dds_udp.launch.py']),
            launch_arguments={
                'instance': drone['instance'],
                'sysid': drone['sysid'],
                'mavlink_connection': drone['mavlink_connection'],
            }.items(),
        ) for drone in drones
    ]

    # Swarm control nodes
    def create_drone_node(node_type, drone_params):
        return Node(
            package='swarm_control',
            executable=f'{node_type}_swarm.py',
            name=f'{node_type}_{drone_params["name"]}',
            output='screen',
            parameters=[{
                'mavlink_connection': drone_params['mavlink_connection'],
                'sysid': drone_params['sysid']
            }],
            respawn=True,
            respawn_delay=5.0
        )

    leader_node = create_drone_node('leader', drones[0])
    follower_nodes = [create_drone_node('follower', drone) for drone in drones[1:]]

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        gz_sim_server,
        gz_sim_gui,
        rviz,
        GroupAction([
            TimerAction(period=(i+1)*10.0, actions=[sitl]) for i, sitl in enumerate(sitl_instances)
        ]),
        TimerAction(period=60.0, actions=[leader_node] + follower_nodes),
    ])