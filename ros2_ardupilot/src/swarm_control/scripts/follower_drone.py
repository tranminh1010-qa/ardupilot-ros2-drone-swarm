#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class FollowerDrone(Node):
    def __init__(self, node_name, state_topic, leader_pos_topic, local_pos_topic, arming_service, set_mode_service, offset_x, offset_y, offset_z):
        super().__init__(node_name)
        self.state_sub = self.create_subscription(State, state_topic, self.state_cb, 10)
        self.leader_pos_sub = self.create_subscription(PoseStamped, leader_pos_topic, self.leader_pos_cb, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, local_pos_topic, 10)

        self.arming_client = self.create_client(CommandBool, arming_service)
        self.set_mode_client = self.create_client(SetMode, set_mode_service)

        self.current_state = State()
        self.target_position = PoseStamped()
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z

        self.offboard_mode_set = False
        self.timer = self.create_timer(0.05, self.arm_and_takeoff)

    def state_cb(self, state):
        self.current_state = state

    def leader_pos_cb(self, leader_pos):
        self.target_position.pose.position.x = leader_pos.pose.position.x + self.offset_x
        self.target_position.pose.position.y = leader_pos.pose.position.y + self.offset_y
        self.target_position.pose.position.z = leader_pos.pose.position.z + self.offset_z
        self.local_pos_pub.publish(self.target_position)

    def arm_and_takeoff(self):
        if not self.current_state.armed:
            if self.arming_client.wait_for_service(timeout_sec=1.0):
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
            return

        if not self.offboard_mode_set:
            if self.set_mode_client.wait_for_service(timeout_sec=1.0):
                req = SetMode.Request()
                req.custom_mode = "OFFBOARD"
                self.set_mode_client.call_async(req)
                if self.current_state.mode == "OFFBOARD":
                    self.offboard_mode_set = True
            return

        self.get_logger().info(f"{self.get_name()} taking off with offset")
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)

    follower1 = FollowerDrone(
        node_name='follower_drone1_node',
        state_topic='/mavros2/state',
        leader_pos_topic='/leader/position',
        local_pos_topic='/mavros2/setpoint_position/local',
        arming_service='/mavros2/cmd/arming',
        set_mode_service='/mavros2/set_mode',
        offset_x=1.0,
        offset_y=0.0,
        offset_z=0.0
    )

    follower2 = FollowerDrone(
        node_name='follower_drone2_node',
        state_topic='/mavros3/state',
        leader_pos_topic='/leader/position',
        local_pos_topic='/mavros3/setpoint_position/local',
        arming_service='/mavros3/cmd/arming',
        set_mode_service='/mavros3/set_mode',
        offset_x=0.0,
        offset_y=1.0,
        offset_z=0.0
    )

    rclpy.spin(follower1)
    rclpy.spin(follower2)

    follower1.destroy_node()
    follower2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()