#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class LeaderDrone(Node):
    def __init__(self):
        super().__init__('leader_drone_node')
        self.state_sub1 = self.create_subscription(State, '/d1/mavros/state', self.state_cb1, 10)
        self.local_pos_pub1 = self.create_publisher(PoseStamped, '/d1/mavros/setpoint_position/local', 10)

        self.arming_client1 = self.create_client(CommandBool, '/d1/mavros/cmd/arming')
        self.set_mode_client1 = self.create_client(SetMode, '/d1/mavros/set_mode')
        self.set_takeoff_client1 = self.create_client(CommandTOL, '/d1/mavros/cmd/takeoff')

        self.state_sub2 = self.create_subscription(State, '/d2/mavros/state', self.state_cb2, 10)
        self.local_pos_pub2 = self.create_publisher(PoseStamped, '/d2/mavros/setpoint_position/local', 10)

        self.arming_client2 = self.create_client(CommandBool, '/d2/mavros/cmd/arming')
        self.set_mode_client2 = self.create_client(SetMode, '/d2/mavros/set_mode')
        self.set_takeoff_client2 = self.create_client(CommandTOL, '/d2/mavros/cmd/takeoff')

        self.state_sub3 = self.create_subscription(State, '/d3/mavros/state', self.state_cb3, 10)
        self.local_pos_pub3 = self.create_publisher(PoseStamped, '/d3/mavros/setpoint_position/local', 10)

        self.arming_client3 = self.create_client(CommandBool, '/d3/mavros/cmd/arming')
        self.set_mode_client3 = self.create_client(SetMode, '/d3/mavros/set_mode')
        self.set_takeoff_client3 = self.create_client(CommandTOL, '/d3/mavros/cmd/takeoff')

        self.current_state1 = State()
        self.current_state2 = State()
        self.current_state3 = State()

        self.current_pose1 = PoseStamped()

        self.target_position1 = PoseStamped()
        self.target_position1.pose.position.x = 0
        self.target_position1.pose.position.y = 0
        self.target_position1.pose.position.z = 5  # Set initial altitude to 5m

        self.target_position2 = PoseStamped()
        self.target_position2.pose.position.x = self.target_position1.pose.position.x
        self.target_position2.pose.position.y = self.target_position1.pose.position.y
        self.target_position2.pose.position.z = self.target_position1.pose.position.z  # Set initial altitude to 5m

        self.target_position3 = PoseStamped()
        self.target_position3.pose.position.x = self.target_position1.pose.position.x
        self.target_position3.pose.position.y = self.target_position1.pose.position.y
        self.target_position3.pose.position.z = self.target_position1.pose.position.z # Set initial altitude to 5m

        self.takeoff_complete = False
        self.timer = self.create_timer(0.05, self.arm_and_takeoff)
        self.guided_mode_set1 = False
        self.guided_mode_set2 = False
        self.guided_mode_set3 = False

        self.pose_sub1 = self.create_subscription(PoseStamped, '/d1/mavros/local_position/pose', self.pose_cb1, 10)
        self.lx = 0.0
        self.ly = 0.0
        self.lz = 0.0

    def state_cb1(self, state):
        self.current_state1 = state

    def state_cb2(self, state):
        self.current_state2 = state

    def state_cb3(self, state):
        self.current_state3 = state

    def arm_and_takeoff(self):
        if not (self.guided_mode_set1 and self.guided_mode_set2 and self.guided_mode_set3):
            if self.set_mode_client1.wait_for_service(timeout_sec=1.0):
                req = SetMode.Request()
                req.custom_mode = "GUIDED"
                self.set_mode_client1.call_async(req)
            if self.set_mode_client2.wait_for_service(timeout_sec=1.0):
                req = SetMode.Request()
                req.custom_mode = "GUIDED"
                self.set_mode_client2.call_async(req)
            if self.set_mode_client3.wait_for_service(timeout_sec=1.0):
                req = SetMode.Request()
                req.custom_mode = "GUIDED"
                self.set_mode_client3.call_async(req)
            if self.current_state1.mode == "GUIDED":
                self.guided_mode_set1 = True
            if self.current_state2.mode == "GUIDED":
                self.guided_mode_set2 = True
            if self.current_state3.mode == "GUIDED":
                self.guided_mode_set3 = True
            return

        if not (self.current_state1.armed and self.current_state2.armed and self.current_state3.armed):
            if self.arming_client1.wait_for_service(timeout_sec=1.0):
                req = CommandBool.Request()
                req.value = True
                self.arming_client1.call_async(req)
            if self.arming_client2.wait_for_service(timeout_sec=1.0):
                req = CommandBool.Request()
                req.value = True
                self.arming_client2.call_async(req)
            if self.arming_client3.wait_for_service(timeout_sec=1.0):
                req = CommandBool.Request()
                req.value = True
                self.arming_client3.call_async(req)
            return

        if self.set_takeoff_client1.wait_for_service(timeout_sec=1.0):
            req = CommandTOL.Request()
            req.altitude = 5.0
            self.set_takeoff_client1.call_async(req)
        if self.set_takeoff_client2.wait_for_service(timeout_sec=1.0):
            req = CommandTOL.Request()
            req.altitude = 5.0
            self.set_takeoff_client2.call_async(req)
        if self.set_takeoff_client3.wait_for_service(timeout_sec=1.0):
            req = CommandTOL.Request()
            req.altitude = 5.0
            self.set_takeoff_client3.call_async(req)

        self.get_logger().info("Leader drone taking off to 5m altitude")

        # Wait until the drones reach the desired altitude
        if self.current_pose1.pose.position.z < 4.9:  # Adding a small margin to ensure stability at the altitude
            self.get_logger().info("Waiting for the drone to reach takeoff altitude...")
            return

        self.get_logger().info("Drones have reached takeoff altitude")
        self.takeoff_complete = True
        self.timer.cancel()

    def pose_cb1(self, pose):
        # Only update positions after takeoff is complete
        self.current_pose1 = pose
        self.lx = pose.pose.position.x
        self.ly = pose.pose.position.y
        self.lz = pose.pose.position.z

        if self.takeoff_complete:
            self.target_position2.pose.position.x = self.lx
            self.target_position2.pose.position.y = self.ly
            self.target_position2.pose.position.z = self.lz
            self.local_pos_pub2.publish(self.target_position2)

            self.target_position3.pose.position.x = self.lx
            self.target_position3.pose.position.y = self.ly
            self.target_position3.pose.position.z = self.lz
            self.local_pos_pub3.publish(self.target_position3)

    def send_position1(self, x, y, z):
        self.target_position1.pose.position.x = x
        self.target_position1.pose.position.y = y
        self.target_position1.pose.position.z = z
        self.local_pos_pub1.publish(self.target_position1)

        self.target_position2.pose.position.x = self.lx
        self.target_position2.pose.position.y = self.ly
        self.target_position2.pose.position.z = self.lz
        self.local_pos_pub2.publish(self.target_position2)

        self.target_position3.pose.position.x = self.lx
        self.target_position3.pose.position.y = self.ly
        self.target_position3.pose.position.z = self.lz
        self.local_pos_pub3.publish(self.target_position3)

        self.get_logger().info(f"Leader drone moving to position: {x}, {y}, {z}")

def main(args=None):
    rclpy.init(args=args)
    leader = LeaderDrone()
    rclpy.spin(leader)
    leader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()