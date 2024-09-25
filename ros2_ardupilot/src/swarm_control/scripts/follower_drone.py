#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil

class FollowerDrone(Node):
    def __init__(self):
        super().__init__('follower_drone_node')
        self.declare_parameter('mavlink_connection', 'udp:localhost:14552')
        self.declare_parameter('leader_pos_topic', 'leader_position')
        self.declare_parameter('offset', [1.0, 0.0, 0.0])

        mavlink_connection = self.get_parameter('mavlink_connection').value
        self.leader_pos_topic = self.get_parameter('leader_pos_topic').value
        self.offset = self.get_parameter('offset').value

        self.mavlink_connection = mavutil.mavlink_connection(mavlink_connection)
        self.mavlink_connection.wait_heartbeat()

        self.create_subscription(PoseStamped, self.leader_pos_topic, self.leader_position_callback, 10)

    def leader_position_callback(self, msg):
        target_x = msg.pose.position.x + self.offset[0]
        target_y = msg.pose.position.y + self.offset[1]
        target_z = msg.pose.position.z + self.offset[2]

        self.mavlink_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,  # type_mask (only positions enabled)
            target_x, target_y, target_z,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity
            0, 0, 0,  # x, y, z acceleration
            0, 0  # yaw, yaw_rate
        )

def main(args=None):
    rclpy.init(args=args)
    follower = FollowerDrone()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()