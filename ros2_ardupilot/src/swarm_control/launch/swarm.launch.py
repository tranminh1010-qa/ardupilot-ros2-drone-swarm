import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('my_ardupilot_gazebo')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Configure Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': os.path.join(get_package_share_directory(
            'mavlink_sitl_gazebo'), 'worlds', 'iris.world')}.items()
    )

    # Spawn Iris model
    spawn_iris = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'iris',
             '-file', os.path.join(get_package_share_directory('mavlink_sitl_gazebo'),
                                   'models', 'iris', 'iris.sdf'),
             '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    # Launch MAVROS
    mavros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mavros'), 'launch'), '/px4.launch.py']),
        launch_arguments={
            'fcu_url': 'udp://:14550@localhost:14555',
            'gcs_url': '',
            'tgt_system': '1',
            'tgt_component': '1'
        }.items()
    )

    # Launch ArduPilot SITL
    ardupilot_sitl = ExecuteProcess(
        cmd=['sim_vehicle.py', '-v', 'ArduCopter', '-f', 'gazebo-iris',
             '--model', 'x', '--speedup', '1',
             '--defaults', os.path.join(get_package_share_directory('ardupilot_gazebo'),
                                        'config', 'iris_defaults.parm')],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_iris,
        mavros,
        ardupilot_sitl
    ])