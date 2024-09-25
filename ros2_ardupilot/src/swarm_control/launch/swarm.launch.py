from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('swarm_control')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Configure Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Launch ArduPilot SITL for multiple drones
    def create_drone_sitl(drone_id):
        return ExecuteProcess(
            cmd=[
                'sim_vehicle.py',
                '-v', 'ArduCopter',
                '-f', 'gazebo-iris',
                '--model', 'x',
                '--speedup', '1',
                '-I' + str(drone_id),
                '--sysid', str(drone_id + 1),
                '--out', f'udp:127.0.0.1:{14550 + drone_id}',
                '--mavproxy-args', f'--out=udp:127.0.0.1:{14560 + drone_id}'
            ],
            name=f'ardupilot_sitl_drone_{drone_id}',
            output='screen'
        )

    # Spawn drone models in Gazebo
    def spawn_drone(drone_id, x, y, z):
        return Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', f'drone_{drone_id}',
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
                '-file', os.path.join(get_package_share_directory('ardupilot_gz_description'), 'models', 'iris', 'model.sdf')
            ],
            output='screen'
        )

    # Create actions for multiple drones
    num_drones = 3
    drone_actions = []
    for i in range(num_drones):
        drone_actions.extend([
            create_drone_sitl(i),
            spawn_drone(i, i * 2.0, 0.0, 0.1)  # Adjust positions as needed
        ])

    # Launch leader and follower nodes
    leader_node = Node(
        package='swarm_control',
        executable='leader_swarm.py',
        name='leader_drone_node',
        parameters=[{
            'num_drones': num_drones,
            'mavlink_connection': 'udp:localhost:14551'
        }],
        output='screen'
    )

    follower_nodes = [
        Node(
            package='swarm_control',
            executable='follower_drone.py',
            name=f'follower_drone_node_{i}',
            parameters=[{
                'node_name': f'follower_drone_node_{i}',
                'mavlink_connection': f'udp:localhost:{14551 + i + 1}',
                'leader_pos_topic': '/leader_drone_node/position',
                'offset': [2.0 * (i + 1), 0.0, 0.0]  # Adjust offsets as needed
            }],
            output='screen'
        ) for i in range(num_drones - 1)
    ]

    # ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        GroupAction(drone_actions),
        leader_node,
        *follower_nodes
    ])