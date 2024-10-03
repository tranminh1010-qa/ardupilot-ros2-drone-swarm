#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import sys


class FollowerDrone(Node):
    def __init__(self):
        super().__init__('follower_drone_node')
        print("FollowerDrone __init__ started", file=sys.stderr)
        self.get_logger().info("FollowerDrone __init__ started")

        self.declare_parameter('mavlink_connection', 'udp:localhost:14551')
        self.declare_parameter('leader_pos_topic', '/leader_drone_node/position')
        self.declare_parameter('offset', [1.0, 0.0, 0.0])

        mavlink_connection = self.get_parameter('mavlink_connection').value
        self.leader_pos_topic = self.get_parameter('leader_pos_topic').value
        self.offset = self.get_parameter('offset').value

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

        self.create_subscription(PoseStamped, self.leader_pos_topic, self.leader_position_callback, 10)

        print("Follower drone initialized", file=sys.stderr)
        self.get_logger().info("Follower drone initialized")

    def leader_position_callback(self, msg):
        print("leader_position_callback called", file=sys.stderr)
        self.get_logger().info("leader_position_callback called")
        # ... rest of the method remains the same


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