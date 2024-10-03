#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time

class LeaderDrone(Node):
    def __init__(self):
        super().__init__('leader_drone_node')
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('mavlink_connection', 'udp:localhost:14550')

        self.num_drones = self.get_parameter('num_drones').value
        mavlink_connection = self.get_parameter('mavlink_connection').value

        self.mavlink_connection = mavutil.mavlink_connection(mavlink_connection)
        self.mavlink_connection.wait_heartbeat()

        self.position_pub = self.create_publisher(PoseStamped, 'position', 10)
        self.timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info("Leader drone initialized")

    def publish_position(self):
        msg = self.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = msg.lat / 1e7
            pose.pose.position.y = msg.lon / 1e7
            pose.pose.position.z = msg.alt / 1000.0
            self.position_pub.publish(pose)
            self.get_logger().info(f"Published position: {pose.pose.position}")

        # Send command to move forward
        self.mavlink_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            0,
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,
            int(pose.pose.position.x * 1e7),
            int(pose.pose.position.y * 1e7),
            100,  # Altitude
            0, 0, 0,  # Velocity
            0, 0, 0,  # Acceleration
            0, 0))

def main(args=None):
    rclpy.init(args=args)
    leader = LeaderDrone()
    rclpy.spin(leader)
    leader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()