import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('swarm_control')

    # Construct the correct path to the world file
    world_path = os.path.join(bringup_dir, 'config', 'models', 'multi_drone_world.world')

    # Set environment variables
    env_vars = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', '/usr/share/gz/gz-sim7/worlds:/usr/share/gz/gz-sim7'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins'),
        SetEnvironmentVariable('LD_LIBRARY_PATH', '/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins:' + os.environ.get('LD_LIBRARY_PATH', ''))
    ]

    return LaunchDescription(env_vars + [
        # Start Gazebo Harmonic with the specified world file in headless mode
        ExecuteProcess(
            cmd=['gz', 'sim', '--headless'],
            output='screen'
        ),

        # Start the MAVLink node
        Node(
            package='swarm_control',
            executable='mavlink_node.py',
            name='mavlink_node',
            output='screen'
        ),

        # Start the drone control node
        Node(
            package='swarm_control',
            executable='drone_control_node.py',
            name='drone_control_node',
            output='screen',
            parameters=[{'publish_rate': 0.2}]  # Set to publish every 5 seconds
        )
    ])