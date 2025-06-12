#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

from scripts.points_distributor import generate_grid_waypoints, split_by_sector


def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='4',
        description='Number of drones to launch'
    )

    def launch_setup(context, *args, **kwargs):
        try:
            num_drones = int(context.launch_configurations['num_drones'])

            # Docker Compose environment - SITL instances are already running
            ros_domain_id = '1'  # Match docker-compose.yaml ROS_DOMAIN_ID
            base_mavlink_port = 14550
            base_ros_port = 14551

            action_list = []

            # Generate waypoints and distribute to drones
            wps = generate_grid_waypoints(field_size=80.0, num_points=12, height=10.0)
            action_list.append(LogInfo(msg=f"Generated {len(wps)} waypoints for {num_drones} drones."))
            chunks = split_by_sector(wps, num_drones)

            # Wait for SITL instances to be ready
            action_list.append(LogInfo(msg="Waiting for SITL instances to initialize..."))

            # Check SITL readiness
            check_sitl_action = ExecuteProcess(
                cmd=['bash', '-c', f'''
                    echo "Checking SITL readiness for {num_drones} drones..."
                    for i in $(seq 0 {num_drones - 1}); do
                        port=$((14550 + i * 10))
                        echo "Checking drone $((i+1)) on port $port..."
                        timeout=60
                        counter=0
                        while ! nc -z localhost $port && [ $counter -lt $timeout ]; do
                            echo "Waiting for SITL drone $((i+1))... ($counter/$timeout)"
                            sleep 2
                            counter=$((counter + 2))
                        done
                        if [ $counter -ge $timeout ]; then
                            echo "Warning: Timeout waiting for drone $((i+1)) on port $port"
                        else
                            echo "Drone $((i+1)) SITL ready on port $port"
                        fi
                    done
                    echo "SITL readiness check completed"
                '''],
                output='screen',
                shell=True
            )

            action_list.append(
                TimerAction(
                    period=10.0,  # Give containers time to start
                    actions=[check_sitl_action]
                )
            )

            # Create ROS nodes for each drone
            drone_nodes = []

            for i in range(num_drones):
                drone_id = i + 1
                instance = i
                mavlink_port = base_mavlink_port + (i * 10)
                ros_port = base_ros_port + (i * 10)
                wp = chunks[i] if i < len(chunks) else wps[:4]  # Fallback waypoints

                # Create drone node - connects to existing SITL instance
                drone_node = Node(
                    package='swarm_control',
                    executable='base_drone.py',
                    name=f'drone{drone_id}',
                    output='screen',
                    parameters=[{
                        'drone_id': drone_id,
                        'mavlink_connection': f'udp:localhost:{ros_port}',
                        'assigned_waypoints': str(wp),
                        'instance': instance,
                        'mavlink_port': mavlink_port,
                        'ros_port': ros_port,
                    }],
                    additional_env={
                        'ROS_DOMAIN_ID': ros_domain_id,
                        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'
                    }
                )

                drone_nodes.append(drone_node)

            # Start drone nodes with staggered timing after SITL check
            for i, drone_node in enumerate(drone_nodes):
                startup_delay = 75.0 + (i * 5.0)  # After SITL check + stagger
                action_list.append(
                    TimerAction(
                        period=startup_delay,
                        actions=[
                            LogInfo(msg=f"Starting ROS node for drone {i + 1}..."),
                            drone_node
                        ]
                    )
                )

            # Add swarm coordinator node
            swarm_coordinator = Node(
                package='swarm_control',
                executable='swarm_coordinator.py',
                name='swarm_coordinator',
                output='screen',
                parameters=[{
                    'num_drones': num_drones,
                    'coordination_mode': 'decentralized',
                    'formation_type': 'grid',
                }],
                additional_env={
                    'ROS_DOMAIN_ID': ros_domain_id,
                    'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'
                }
            )

            action_list.append(
                TimerAction(
                    period=15.0,
                    actions=[
                        LogInfo(msg="Starting swarm coordinator..."),
                        swarm_coordinator
                    ]
                )
            )

            # Add system monitoring
            monitor_action = ExecuteProcess(
                cmd=['bash', '-c', f'''
                    echo "=== Docker Compose Swarm Monitor ==="
                    while true; do
                        echo "=== $(date) ==="
                        echo "SITL Container Status:"
                        docker ps --filter "name=swarm_ardupilot-drone" --format "table {{{{.Names}}}}\\t{{{{.Status}}}}" 2>/dev/null || echo "Cannot access Docker (may be running inside container)"

                        echo "Port Status:"
                        for i in $(seq 0 {num_drones - 1}); do
                            mavlink_port=$((14550 + i * 10))
                            ros_port=$((14551 + i * 10))
                            if netstat -tuln 2>/dev/null | grep -q ":$mavlink_port "; then
                                echo "Drone $((i+1)): MAVLink $mavlink_port ✓, ROS $ros_port"
                            else
                                echo "Drone $((i+1)): MAVLink $mavlink_port ✗, ROS $ros_port"
                            fi
                        done

                        echo "ROS Topics:"
                        timeout 5 ros2 topic list 2>/dev/null | grep -E "(drone|swarm)" | head -10 || echo "No ROS topics detected"
                        echo "---"
                        sleep 15
                    done
                '''],
                output='screen',
                shell=True
            )

            action_list.append(
                TimerAction(
                    period=90.0,  # Start monitoring after nodes are up
                    actions=[monitor_action]
                )
            )

            # Add connection diagnostics
            diagnostic_action = ExecuteProcess(
                cmd=['bash', '-c', f'''
                    echo "=== Connection Diagnostics ==="
                    echo "Environment:"
                    echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
                    echo "Container Network: $(cat /proc/net/route | grep '^00000000' | awk '{{print $2}}' | xargs printf '%d.%d.%d.%d\\n' $(echo 'obase=10; ibase=16;' | bc -l))"

                    echo "Expected connections:"
                    for i in $(seq 1 {num_drones}); do
                        mavlink_port=$((14550 + (i-1) * 10))
                        ros_port=$((14551 + (i-1) * 10))
                        echo "Drone $i: MAVLink=udp:127.0.0.1:$mavlink_port, ROS=udp:127.0.0.1:$ros_port"
                    done
                '''],
                output='screen',
                shell=True
            )

            action_list.append(
                TimerAction(
                    period=5.0,
                    actions=[diagnostic_action]
                )
            )

            return action_list

        except Exception as e:
            print(f"Error in launch setup: {str(e)}")
            return [LogInfo(msg=f"Error in launch setup: {str(e)}")]

    launch_actions = [num_drones_arg, OpaqueFunction(function=launch_setup)]

    return LaunchDescription(launch_actions)