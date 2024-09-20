#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class LeaderDrone(Node):
    def __init__(self):
        super().__init__('leader_drone_node')
        self.num_drones = 3
        self.drones = self._initialize_drones()
        self.takeoff_altitude = 5.0
        self.takeoff_complete = False
        self.timer = self.create_timer(0.05, self.arm_and_takeoff)

    def _initialize_drones(self):
        drones = []
        for j in range(1, self.num_drones + 1):
            prefix = f'/d{j}/mavros'
            drone = {
                'state_sub': self.create_subscription(State, f'{prefix}/state', lambda msg, i=j: self._state_cb(msg, i), 10),
                'local_pos_pub': self.create_publisher(PoseStamped, f'{prefix}/setpoint_position/local', 10),
                'arming_client': self.create_client(CommandBool, f'{prefix}/cmd/arming'),
                'set_mode_client': self.create_client(SetMode, f'{prefix}/set_mode'),
                'set_takeoff_client': self.create_client(CommandTOL, f'{prefix}/cmd/takeoff'),
                'current_state': State(),
                'target_position': PoseStamped(),
                'guided_mode_set': False
            }
            if j == 1:
                drone['pose_sub'] = self.create_subscription(PoseStamped, f'{prefix}/local_position/pose', self._pose_cb, 10)
                drone['current_pose'] = PoseStamped()
            drones.append(drone)
        return drones

    def _state_cb(self, state, drone_index):
        self.drones[drone_index - 1]['current_state'] = state

    def _pose_cb(self, pose):
        self.drones[0]['current_pose'] = pose
        if self.takeoff_complete:
            self._update_follower_positions(pose)

    def _update_follower_positions(self, leader_pose):
        for i in range(1, self.num_drones):
            self.drones[i]['target_position'].pose.position = leader_pose.pose.position
            self.drones[i]['local_pos_pub'].publish(self.drones[i]['target_position'])

    async def arm_and_takeoff(self):
        if not all(drone['guided_mode_set'] for drone in self.drones):
            await self._set_guided_mode()
            return

        if not all(drone['current_state'].armed for drone in self.drones):
            await self._arm_drones()
            return

        if not self.takeoff_complete:
            await self._takeoff_drones()

    async def _set_guided_mode(self):
        for drone in self.drones:
            if not drone['guided_mode_set']:
                if drone['set_mode_client'].wait_for_service(timeout_sec=1.0):
                    req = SetMode.Request(custom_mode="GUIDED")
                    await drone['set_mode_client'].call_async(req)
                if drone['current_state'].mode == "GUIDED":
                    drone['guided_mode_set'] = True

    async def _arm_drones(self):
        for drone in self.drones:
            if not drone['current_state'].armed:
                if drone['arming_client'].wait_for_service(timeout_sec=1.0):
                    req = CommandBool.Request(value=True)
                    await drone['arming_client'].call_async(req)

    async def _takeoff_drones(self):
        for drone in self.drones:
            if drone['set_takeoff_client'].wait_for_service(timeout_sec=1.0):
                req = CommandTOL.Request(altitude=self.takeoff_altitude)
                await drone['set_takeoff_client'].call_async(req)

        self.get_logger().info(f"Drones taking off to {self.takeoff_altitude}m altitude")

        if self.drones[0]['current_pose'].pose.position.z < self.takeoff_altitude - 0.1:
            self.get_logger().info("Waiting for the drones to reach takeoff altitude...")
            return

        self.get_logger().info("Drones have reached takeoff altitude")
        self.takeoff_complete = True
        self.timer.cancel()

    def send_position(self, x, y, z):
        self.drones[0]['target_position'].pose.position.x = x
        self.drones[0]['target_position'].pose.position.y = y
        self.drones[0]['target_position'].pose.position.z = z
        self.drones[0]['local_pos_pub'].publish(self.drones[0]['target_position'])

        self._update_follower_positions(self.drones[0]['current_pose'])
        self.get_logger().info(f"Leader drone moving to position: {x}, {y}, {z}")

def main(args=None):
    rclpy.init(args=args)
    leader = LeaderDrone()
    rclpy.spin(leader)
    leader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()