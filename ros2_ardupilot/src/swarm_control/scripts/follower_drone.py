#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import sys
import time


class FollowerDrone(Node):
    def __init__(self):
        super().__init__('follower_drone_node')
        self.get_logger().info("FollowerDrone __init__ started")

        self.declare_parameter('mavlink_connection', 'udp:localhost:14551')
        self.declare_parameter('leader_pos_topic', '/leader_drone_node/position')
        self.declare_parameter('offset', [1.0, 0.0, 0.0])

        self.mavlink_connection = self.get_parameter('mavlink_connection').value
        self.leader_pos_topic = self.get_parameter('leader_pos_topic').value
        self.offset = self.get_parameter('offset').value

        self.get_logger().info(f"Attempting to establish mavlink connection: {self.mavlink_connection}")

        self.mav_connection = None
        self.connect_timer = self.create_timer(5.0, self.attempt_connect)

        self.create_subscription(PoseStamped, self.leader_pos_topic, self.leader_position_callback, 10)

        self.get_logger().info("Follower drone initialized")

    def attempt_connect(self):
        if self.mav_connection is None or not self.mav_connection.target_system:
            try:
                self.get_logger().info(f"Attempting to connect to {self.mavlink_connection}")
                self.mav_connection = mavutil.mavlink_connection(self.mavlink_connection, source_system=2, timeout=60)
                self.get_logger().info(f"Connection established: {self.mavlink_connection}")
                self.get_logger().info("Waiting for heartbeat...")
                self.mav_connection.wait_heartbeat(timeout=10)
                self.get_logger().info("Heartbeat received!")
                self.connect_timer.cancel()
                self.set_guided_mode()
                self.arm_drone()
            except Exception as e:
                self.get_logger().error(f"Error establishing mavlink connection: {str(e)}")

    def set_guided_mode(self):
        for i in range(5):  # Try 5 times
            self.mav_connection.set_mode('GUIDED')
            time.sleep(1)  # Wait for mode change
            mode = self.mav_connection.flightmode
            self.get_logger().info(f"Attempt {i+1}: Current flight mode: {mode}")
            if mode == 'GUIDED':
                self.get_logger().info(f"Successfully set GUIDED mode")
                return True
        self.get_logger().error(f"Failed to set GUIDED mode")
        return False

    def arm_drone(self):
        self.mav_connection.arducopter_arm()
        self.get_logger().info("Attempting to arm drone")
        for i in range(10):
            if self.mav_connection.motors_armed():
                self.get_logger().info("Drone armed successfully.")
                return True
            time.sleep(1)
        self.get_logger().error("Failed to arm drone")
        return False

    def leader_position_callback(self, msg):
        self.get_logger().debug("leader_position_callback called")

        target_x = msg.pose.position.x + self.offset[0]
        target_y = msg.pose.position.y + self.offset[1]
        target_z = msg.pose.position.z + self.offset[2]

        self.get_logger().info(f"Following leader at offset: {target_x}, {target_y}, {target_z}")

        if self.mav_connection and self.mav_connection.target_system:
            try:
                self.mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                    0,
                    self.mav_connection.target_system,
                    self.mav_connection.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b110111111000,
                    int(target_x * 1e7),
                    int(target_y * 1e7),
                    int(target_z * 1000),
                    0, 0, 0,  # Velocity
                    0, 0, 0,  # Acceleration
                    0, 0))
            except Exception as e:
                self.get_logger().error(f"Error sending MAVLink message: {str(e)}")
        else:
            self.get_logger().warning("MAVLink connection not established. Cannot send command.")

    def shutdown(self):
        if self.mav_connection and self.mav_connection.target_system:
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
            self.get_logger().info("Landing command sent to follower drone")


def main(args=None):
    rclpy.init(args=args)
    follower = FollowerDrone()
    try:
        rclpy.spin(follower)
    finally:
        follower.shutdown()
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()