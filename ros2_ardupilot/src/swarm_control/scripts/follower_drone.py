#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class FollowerDrone(Node):
    def __init__(self, node_name, mavros_prefix, leader_pos_topic, offset):
        super().__init__(node_name)
        self.mavros_prefix = mavros_prefix
        self.offset = offset

        # Subscriptions
        self.create_subscription(State, f'{mavros_prefix}/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, leader_pos_topic, self.leader_pos_cb, 10)

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, f'{mavros_prefix}/setpoint_position/local', 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, f'{mavros_prefix}/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, f'{mavros_prefix}/set_mode')

        # State variables
        self.current_state = State()
        self.target_position = PoseStamped()
        self.offboard_mode_set = False

        # Timer
        self.create_timer(0.05, self.arm_and_takeoff)

    def state_cb(self, state):
        self.current_state = state

    def leader_pos_cb(self, leader_pos):
        self.target_position.header = leader_pos.header
        self.target_position.pose.position.x = leader_pos.pose.position.x + self.offset[0]
        self.target_position.pose.position.y = leader_pos.pose.position.y + self.offset[1]
        self.target_position.pose.position.z = leader_pos.pose.position.z + self.offset[2]
        self.local_pos_pub.publish(self.target_position)

    async def arm_and_takeoff(self):
        if not self.current_state.armed:
            await self.arm_drone()
        elif not self.offboard_mode_set:
            await self.set_offboard_mode()
        else:
            self.get_logger().info(f"{self.get_name()} is armed and in OFFBOARD mode")
            self.destroy_timer(self.arm_and_takeoff)

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

    follower1 = FollowerDrone(
        node_name='follower_drone1_node',
        mavros_prefix='/mavros2',
        leader_pos_topic='/leader/position',
        offset=(1.0, 0.0, 0.0)
    )

    follower2 = FollowerDrone(
        node_name='follower_drone2_node',
        mavros_prefix='/mavros3',
        leader_pos_topic='/leader/position',
        offset=(0.0, 1.0, 0.0)
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(follower1)
    executor.add_node(follower2)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        follower1.destroy_node()
        follower2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()