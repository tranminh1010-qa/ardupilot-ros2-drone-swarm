#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class FollowerDrone(Node):
    def __init__(self, node_name, mavros_prefix, leader_pos_topic, offset):
        super().__init__(node_name)
        self.mavros_prefix = mavros_prefix
        self.leader_pos_topic = leader_pos_topic
        self.offset = offset
        self.current_state = State()
        self.offboard_mode_set = False

        self.local_pos_pub = self.create_publisher(PoseStamped, f'{self.mavros_prefix}/setpoint_position/local', 10)
        self.create_subscription(PoseStamped, self.leader_pos_topic, self.leader_position_callback, 10)
        self.arming_client = self.create_client(CommandBool, f'{self.mavros_prefix}/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, f'{self.mavros_prefix}/set_mode')

        self.arm_and_takeoff_timer = self.create_timer(1.0, self.arm_and_takeoff)

    def leader_position_callback(self, leader_pos):
        self.target_position = PoseStamped()
        self.target_position.pose.position.x = leader_pos.pose.position.x + self.offset[0]
        self.target_position.pose.position.y = leader_pos.pose.position.y + self.offset[1]
        self.target_position.pose.position.z = leader_pos.pose.position.z + self.offset[2]
        self.local_pos_pub.publish(self.target_position)

    async def arm_and_takeoff(self):
        if self.current_state is None:
            self.get_logger().error('Current state is not initialized')
            return
        if not self.current_state.armed:
            await self.arm_drone()
        elif not self.offboard_mode_set:
            await self.set_offboard_mode()
        else:
            self.get_logger().info(f"{self.get_name()} is armed and in OFFBOARD mode")
            self.destroy_timer(self.arm_and_takeoff_timer)

    async def arm_drone(self):
        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request(value=True)
            await self.arming_client.call_async(req)
        else:
            self.get_logger().warn("Arming service not available")

    async def set_offboard_mode(self):
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request(custom_mode="OFFBOARD")
            await self.set_mode_client.call_async(req)
            if self.current_state.mode == "OFFBOARD":
                self.offboard_mode_set = True
                self.get_logger().info(f"{self.get_name()} OFFBOARD mode set")
        else:
            self.get_logger().warn("Set mode service not available")

def main(args=None):
    rclpy.init(args=args)
    follower_drone = FollowerDrone(
        node_name='follower_drone_node',
        mavros_prefix='/mavros',
        leader_pos_topic='/leader/position',
        offset=(0.0, 0.0, 0.0)
    )
    rclpy.spin(follower_drone)
    follower_drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()