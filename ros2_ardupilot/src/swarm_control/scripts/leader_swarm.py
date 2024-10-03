#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import math
import sys


class LeaderDrone(Node):
    def __init__(self):
        super().__init__('leader_drone_node')
        print("LeaderDrone __init__ started", file=sys.stderr)
        self.get_logger().info("LeaderDrone __init__ started")

        self.declare_parameter('num_drones', 3)
        self.declare_parameter('mavlink_connection', 'udp:localhost:14550')

        self.num_drones = self.get_parameter('num_drones').value
        mavlink_connection = self.get_parameter('mavlink_connection').value

        print(f"Attempting to establish mavlink connection: {mavlink_connection}", file=sys.stderr)
        self.get_logger().info(f"Attempting to establish mavlink connection: {mavlink_connection}")

        try:
            self.mavlink_connection = mavutil.mavlink_connection(mavlink_connection)
            print("Waiting for heartbeat...", file=sys.stderr)
            self.get_logger().info("Waiting for heartbeat...")
            heartbeat = self.mavlink_connection.wait_heartbeat(timeout=30)
            if heartbeat:
                print(f"Heartbeat received. Mavlink connection established: {mavlink_connection}", file=sys.stderr)
                self.get_logger().info(f"Heartbeat received. Mavlink connection established: {mavlink_connection}")
            else:
                print("Timeout waiting for heartbeat. Check SITL instance.", file=sys.stderr)
                self.get_logger().warning("Timeout waiting for heartbeat. Check SITL instance.")
        except Exception as e:
            print(f"Error establishing mavlink connection: {str(e)}", file=sys.stderr)
            self.get_logger().error(f"Error establishing mavlink connection: {str(e)}")
            raise

        self.position_pub = self.create_publisher(PoseStamped, 'position', 10)
        self.timer = self.create_timer(0.1, self.publish_position)

        self.start_time = time.time()
        print("Leader drone initialized", file=sys.stderr)
        self.get_logger().info("Leader drone initialized")

    def publish_position(self):
        print("publish_position called", file=sys.stderr)
        self.get_logger().info("publish_position called")
        # ... rest of the method remains the same


def main(args=None):
    print("main function started", file=sys.stderr)
    rclpy.init(args=args)
    print("rclpy initialized", file=sys.stderr)
    leader = LeaderDrone()
    print("LeaderDrone instance created", file=sys.stderr)
    rclpy.spin(leader)
    leader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()