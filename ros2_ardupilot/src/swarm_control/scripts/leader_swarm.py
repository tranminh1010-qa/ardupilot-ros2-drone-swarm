#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil


class LeaderDrone(Node):
    def __init__(self):
        super().__init__('leader_drone_node')
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('mavlink_connection', 'udp:localhost:14551')

        self.num_drones = self.get_parameter('num_drones').value
        mavlink_connection = self.get_parameter('mavlink_connection').value

        self.mavlink_connection = mavutil.mavlink_connection(mavlink_connection)
        self.mavlink_connection.wait_heartbeat()

        self.position_pub = self.create_publisher(PoseStamped, 'position', 10)
        self.timer = self.create_timer(0.1, self.publish_position)

    def publish_position(self):
        msg = self.mavlink_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = msg.x
            pose.pose.position.y = msg.y
            pose.pose.position.z = -msg.z  # NED to ENU conversion
            self.position_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    leader = LeaderDrone()
    rclpy.spin(leader)
    leader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()