#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ardupilot_msgs.msg import Status  # Use DDS messages
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
import time
import enum
import os
from noise_injector import NoiseInjector
from points_distributor import generate_circular_waypoints
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy



class DroneState(enum.Enum):
    INITIALIZING = 0
    CONNECTING = 1
    CONNECTED = 2
    ARMING = 3
    ARMED = 4
    TAKING_OFF = 5
    FLYING = 6
    LANDING = 7
    ERROR = 8


class BaseDrone(Node):
    def __init__(self, node_name, drone_id=None, assigned_waypoints=None):
        super().__init__(node_name)
        self.waypoints = assigned_waypoints
        self.drone_id = drone_id
        self.state = DroneState.INITIALIZING
        self.current_position = None
        self.target_position = None
        self.armed = False

        # Create ArduPilot-compatible QoS profile
        ardupilot_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )


        # DDS subscribers and service clients
        self.status_subscription = self.create_subscription(
            Status,
            '/ap/status',
            self.status_callback,
            ardupilot_qos)  #

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            ardupilot_qos)

        # Service clients for DDS
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')

        # Command publisher
        self.cmd_pose_pub = self.create_publisher(
            PoseStamped,
            '/ap/cmd_gps_pose',
            10)

        self.get_logger().info(f"Initializing drone {self.drone_id} with DDS interface")

        # Check DDS connection
        self.connection_timer = self.create_timer(2.0, self.check_dds_connection)
        self.get_logger().info(f"Subscribed to: {self.status_subscription.topic_name}")
        self.get_logger().info(f"QoS: {self.status_subscription.qos_profile}")


    def status_callback(self, msg):
        """Handle ArduPilot status messages"""
        self.armed = msg.armed
        if self.state == DroneState.CONNECTING:
            self.state = DroneState.CONNECTED
            self.get_logger().info(f"DDS connection established for drone {self.drone_id}")

    def pose_callback(self, msg):
        """Handle pose updates"""
        self.current_position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

    def check_dds_connection(self):
        """Check if DDS topics are available"""
        if self.state == DroneState.INITIALIZING:
            self.state = DroneState.CONNECTING
            self.get_logger().info(f"Waiting for DDS connection for drone {self.drone_id}")
        else:
            self.connection_timer.cancel()
            self.start_mission_setup()

    def start_mission_setup(self):
        """Start the mission sequence"""
        self.get_logger().info(f"Starting mission setup for drone {self.drone_id}")
        self.arm_drone_dds()

    def arm_drone_dds(self):
        """Arm drone using DDS service"""
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arm service not available')
            return

        request = ArmMotors.Request()
        request.arm = True

        future = self.arm_client.call_async(request)
        future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        """Handle arm service response"""
        try:
            response = future.result()
            if response.result:
                self.get_logger().info(f"Drone {self.drone_id} armed successfully via DDS")
                self.set_guided_mode_dds()
            else:
                self.get_logger().error(f"Failed to arm drone {self.drone_id}")
        except Exception as e:
            self.get_logger().error(f'Arm service call failed: {e}')

    def set_guided_mode_dds(self):
        """Set guided mode using DDS"""
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Mode service not available')
            return

        request = ModeSwitch.Request()
        request.mode = 4  # GUIDED mode

        future = self.mode_client.call_async(request)
        future.add_done_callback(self.mode_response_callback)

    def mode_response_callback(self, future):
        """Handle mode service response"""
        try:
            response = future.result()
            if response.status:
                self.get_logger().info(f"Drone {self.drone_id} in GUIDED mode")
                self.start_mission()
            else:
                self.get_logger().error(f"Failed to set GUIDED mode for drone {self.drone_id}")
        except Exception as e:
            self.get_logger().error(f'Mode service call failed: {e}')

    def start_mission(self):
        """Start waypoint mission using DDS"""
        self.get_logger().info(f"Starting mission for drone {self.drone_id}")
        self.state = DroneState.FLYING

        # Simple waypoint following
        for i, wp in enumerate(self.waypoints):
            self.send_position_command_dds(wp[0], wp[1], wp[2])
            self.get_logger().info(f"Drone {self.drone_id} heading to waypoint {i + 1}: {wp}")
            time.sleep(10)  # Wait between waypoints

    def send_position_command_dds(self, x, y, z):
        """Send position command via DDS"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        self.cmd_pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = BaseDrone('base_drone', 1)

    try:
        # Get parameters
        drone_id_param = node.declare_parameter('drone_id', 1).value
        assigned_waypoints_param = node.declare_parameter('assigned_waypoints', '').value

        node.drone_id = drone_id_param

        # Handle waypoints
        if not assigned_waypoints_param:
            node.waypoints = generate_circular_waypoints()
        else:
            node.waypoints = eval(assigned_waypoints_param)

        node.get_logger().info(f"Drone {node.drone_id} initialized with DDS")

    except Exception as e:
        node.get_logger().error(f"Parameter initialization error: {e}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()