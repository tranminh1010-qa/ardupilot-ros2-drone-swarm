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
        print("FollowerDrone __init__ started", file=sys.stderr)
        self.get_logger().info("FollowerDrone __init__ started")

        self.declare_parameter('mavlink_connection', 'udp:localhost:14551')
        self.declare_parameter('leader_pos_topic', '/leader_drone_node/position')
        self.declare_parameter('offset', [1.0, 0.0, 0.0])

        self.mavlink_connection = self.get_parameter('mavlink_connection').value
        self.leader_pos_topic = self.get_parameter('leader_pos_topic').value
        self.offset = self.get_parameter('offset').value

        print(f"Attempting to establish mavlink connection: {self.mavlink_connection}", file=sys.stderr)
        self.get_logger().info(f"Attempting to establish mavlink connection: {self.mavlink_connection}")

        self.mav_connection = None
        self.connect_timer = self.create_timer(5.0, self.attempt_connect)

        self.create_subscription(PoseStamped, self.leader_pos_topic, self.leader_position_callback, 10)

        print("Follower drone initialized", file=sys.stderr)
        self.get_logger().info("Follower drone initialized")

    def attempt_connect(self):
        if self.mav_connection is None or not self.mav_connection.target_system:
            try:
                print(f"Attempting to connect to {self.mavlink_connection}", file=sys.stderr)
                self.get_logger().info(f"Attempting to connect to {self.mavlink_connection}")
                self.mav_connection = mavutil.mavlink_connection(self.mavlink_connection)
                heartbeat = self.mav_connection.wait_heartbeat(timeout=10)
                if heartbeat:
                    print(f"Heartbeat received. Mavlink connection established: {self.mavlink_connection}", file=sys.stderr)
                    self.get_logger().info(f"Heartbeat received. Mavlink connection established: {self.mavlink_connection}")
                    self.connect_timer.cancel()
                else:
                    print("Timeout waiting for heartbeat. Will try again.", file=sys.stderr)
                    self.get_logger().warning("Timeout waiting for heartbeat. Will try again.")
            except Exception as e:
                print(f"Error establishing mavlink connection: {str(e)}", file=sys.stderr)
                self.get_logger().error(f"Error establishing mavlink connection: {str(e)}")

    def leader_position_callback(self, msg):
        print("leader_position_callback called", file=sys.stderr)
        self.get_logger().info("leader_position_callback called")

        target_x = msg.pose.position.x + self.offset[0]
        target_y = msg.pose.position.y + self.offset[1]
        target_z = msg.pose.position.z + self.offset[2]

        self.get_logger().info(f"Following leader at offset: {target_x}, {target_y}, {target_z}")

        if self.mav_connection and self.mav_connection.target_system:
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
        else:
            self.get_logger().warning("MAVLink connection not established. Cannot send command.")

def main(args=None):
    print("main function started", file=sys.stderr)
    rclpy.init(args=args)
    print("rclpy initialized", file=sys.stderr)
    follower = FollowerDrone()
    print("FollowerDrone instance created", file=sys.stderr)
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()