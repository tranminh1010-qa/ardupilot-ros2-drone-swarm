#!/usr/bin/env python3
import json
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

from scripts.points_distributor import generate_grid_waypoints, split_serpentine

# Shared field origin — must match LAT_BASE/LON_BASE in
# src/scripts/start_drone_container.sh so the mapped field is centred on the
# swarm's home area. All drones convert waypoints with this ONE origin so the
# 1/n chunks tile a single field instead of shifting with each drone's home.
FIELD_ORIGIN_LAT = 40.072842
FIELD_ORIGIN_LON = -105.230575

# GstCameraPlugin stream ports: drone instance i streams H.264/RTP on
# CAMERA_PORT_BASE + i (see src/custom_gz/models/base_drone*/model.sdf).
CAMERA_PORT_BASE = 5600


def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='2',
        description='Number of drones to launch'
    )

    rmw_arg = DeclareLaunchArgument(
        'rmw_implementation',
        default_value='rmw_fastrtps_cpp',
        description='ROS MiddleWare Implementation'
    )

    sprayer_instances_arg = DeclareLaunchArgument(
        'sprayer_instances',
        default_value='',
        description='Space-separated 0-indexed instances that fly the sprayer '
                    'role (e.g. "2 3 4 5"); all other instances are mappers. '
                    'Must match SPRAYER_INSTANCES given to start_swarm.sh and '
                    'the sprayer models in the Gazebo world.'
    )

    # Docker Compose environment - SITL instances are already running
    def launch_setup(context, *args, **kwargs):
        try:
            num_drones = int(context.launch_configurations['num_drones'])
            LogInfo(msg = f"Launching ROS2 Node for num_drones:{num_drones}")
            rmw_implementation = str(context.launch_configurations.get('rmw_implementation','rmw_fastrtps_cpp'))
            sprayer_instances = {
                int(s) for s in
                context.launch_configurations.get('sprayer_instances', '').split()
            }
            base_mavlink_port = 14550
            base_ros_port = 14551

            action_list = []

            # Role split: mappers survey at 30 m with cameras, sprayers treat
            # at 15 m with AC_Sprayer. EACH ROLE covers the whole field split
            # 1/n among its own members (balanced serpentine strips).
            mappers = [i for i in range(num_drones) if i not in sprayer_instances]
            sprayers = [i for i in range(num_drones) if i in sprayer_instances]

            wps = generate_grid_waypoints(field_size=80.0, grid_points=5, height=30.0)
            mapper_chunks = split_serpentine(wps, len(mappers)) if mappers else []
            sprayer_chunks = split_serpentine(wps, len(sprayers)) if sprayers else []
            SPRAY_ALT = 15.0

            chunk_for = {}
            for k, i in enumerate(mappers):
                chunk_for[i] = mapper_chunks[k]
            for k, i in enumerate(sprayers):
                chunk_for[i] = [(x, y, SPRAY_ALT) for x, y, _ in sprayer_chunks[k]]

            action_list.append(LogInfo(
                msg=f"Fleet: {len(mappers)} mappers {mappers} + "
                    f"{len(sprayers)} sprayers {sprayers}; "
                    f"{len(wps)} waypoints per role."))

            # Create ROS nodes for each drone
            drone_nodes = []
            for i in range(num_drones):
                drone_id = i + 1
                instance = i
                is_sprayer = i in sprayer_instances
                # Each drone gets its own ROS_DOMAIN_ID (INSTANCE + 1)
                ros_domain_id = str(drone_id)
                mavlink_port = base_mavlink_port + (i * 10)
                ros_port = base_ros_port + (i * 10)
                wp_json = json.dumps(chunk_for[i])

                # Role extensions of BaseDrone: CameraDrone maps with the
                # down-facing camera; SprayerDrone flies with AC_Sprayer on
                # (toggled via SERIAL1 MAVLink TCP 5762 + 10*instance).
                params = {
                    'drone_id': drone_id,
                    'mavlink_connection': f'udp:localhost:{ros_port}',
                    'assigned_waypoints': wp_json,
                    'instance': instance,
                    'mavlink_port': mavlink_port,
                    'ros_port': ros_port,
                    'field_origin_lat': FIELD_ORIGIN_LAT,
                    'field_origin_lon': FIELD_ORIGIN_LON,
                }
                if is_sprayer:
                    executable = 'sprayer_drone_dds.py'
                    params['mavlink_tcp_port'] = 5762 + 10 * instance
                    # Targeted spraying: wait on the ground for the planner's
                    # prescription (weed clusters from the mappers' survey);
                    # assigned_waypoints stays as the blanket-strip fallback.
                    params['prescription_file'] = \
                        f'/root/logs/prescriptions/sprayer_{instance}.json'
                    params['prescription_timeout'] = 600.0
                else:
                    executable = 'camera_drone_dds.py'
                    params['camera_port'] = CAMERA_PORT_BASE + instance

                drone_node = Node(
                    package='swarm_control',
                    executable=executable,
                    name=f'drone{drone_id}',
                    output='screen',
                    # Self-heal against EKF/GPS warmup races: a node that crashes
                    # during startup (arm/position/GUIDED wait) is relaunched and
                    # retries once the SITL instance's EKF is ready. Successful nodes
                    # call rclpy.spin() forever, so respawn only re-runs failures.
                    respawn=True,
                    respawn_delay=5.0,
                    parameters=[params],
                    additional_env={
                        'ROS_DOMAIN_ID': ros_domain_id,
                        'RMW_IMPLEMENTATION': rmw_implementation
                    }
                )

                drone_nodes.append(drone_node)

            for i, drone_node in enumerate(drone_nodes):
                startup_delay = (i * 5.0)
                action_list.append(
                    TimerAction(
                        period=startup_delay,
                        actions=[
                            LogInfo(msg=f"Starting ROS node for drone {i + 1}..."),
                            drone_node
                        ]
                    )
                )

            # Prescription planner: waits for the mappers' survey, clusters
            # the weed detections, and writes per-sprayer spray plans that the
            # SprayerDrone nodes are polling for. File-based (shares
            # /root/logs with the CameraDrone nodes in this container).
            if sprayers and mappers:
                expect = [f"{i + 1}:{len(chunk_for[i])}" for i in mappers]
                action_list.append(ExecuteProcess(
                    cmd=['/ros2_ws/install/swarm_control/lib/swarm_control/'
                         'prescription_planner.py',
                         '--logs', '/root/logs',
                         '--sprayers', *[str(s) for s in sprayers],
                         '--expect', *expect,
                         '--timeout', '420',
                         # 6 m merges duplicate detections of one weed seen
                         # from different frames (georeferencing is nadir-
                         # approximate, vehicle yaw uncompensated) without
                         # merging distinct weeds (typically >10 m apart).
                         '--cluster-radius', '6.0',
                         '--spray-alt', str(SPRAY_ALT)],
                    output='screen',
                    name='prescription_planner',
                ))

            return action_list

        except Exception as e:
            print(f"Error in launch setup: {str(e)}")
            return [LogInfo(msg=f"Error in launch setup: {str(e)}")]

    launch_actions = [num_drones_arg, rmw_arg, sprayer_instances_arg,
                      OpaqueFunction(function=launch_setup)]

    return LaunchDescription(launch_actions)