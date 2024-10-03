import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('ardupilot_sitl_models')
    sdf_path = os.path.join(bringup_dir, 'models', 'skywalker_x8_quad', 'model.sdf')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    def create_drone_sitl(drone_id):
        return ExecuteProcess(
            cmd=[
                'sim_vehicle.py',
                '-v', 'ArduPlane',
                '-f', 'gazebo-zephyr',
                '--model', 'JSON',
                '-I' + str(drone_id),
                '--custom-location=40.072842,-105.230575,1586,0',
                '--speedup', '1',
                '--instance', str(drone_id),
            ],
            output='screen',
            shell=True
        )

    def spawn_drone(drone_id, x, y, z):
        return ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/world/empty/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'sdf_filename: "{sdf_path}", name: "drone_{drone_id}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
            ],
            output='screen'
        )

    num_drones = 3
    drone_actions = []
    for i in range(num_drones):
        drone_actions.extend([
            create_drone_sitl(i),
            TimerAction(
                period=5.0,
                actions=[spawn_drone(i, i * 5.0, 0.0, 0.1)]
            )
        ])

    leader_node = Node(
        package='swarm_control',
        executable='leader_swarm.py',
        name='leader_drone_node',
        parameters=[{
            'num_drones': num_drones,
            'mavlink_connection': 'udp:localhost:14550',
        }],
        output='screen'
    )

    follower_nodes = [
        Node(
            package='swarm_control',
            executable='follower_drone.py',
            name=f'follower_drone_node_{i}',
            parameters=[{
                'mavlink_connection': f'udp:localhost:{14560 + i * 10}',
                'leader_pos_topic': '/leader_drone_node/position',
                'offset': [5.0 * (i + 1), 0.0, 0.0]
            }],
            output='screen'
        ) for i in range(num_drones - 1)
    ]

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
                '/gazebo/default/skywalker_x8_quad_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            output='screen'
        )
    ])

    return LaunchDescription([
        gz_sim,
        TimerAction(
            period=5.0,
            actions=[bridge]
        ),
        GroupAction(drone_actions),
        TimerAction(
            period=15.0,
            actions=[leader_node, *follower_nodes]
        )
    ])