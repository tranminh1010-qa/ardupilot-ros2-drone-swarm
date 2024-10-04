#!/usr/bin/env python3
import time
import pymavlink.mavutil as mavutil
import rclpy
from rclpy.node import Node

class LeaderDrone:
    def __init__(self):
        # Create a MAVLink connection for each drone
        self.master1 = mavutil.mavlink_connection('udp:localhost:14550')  # Leader
        self.master2 = mavutil.mavlink_connection('udp:localhost:14560')  # Follower 1
        self.master3 = mavutil.mavlink_connection('udp:localhost:14570')  # Follower 2

        # Define offsets for follower drones
        self.follower_offset_x2 = -2.0  # 2 meters behind the leader for drone 2
        self.follower_offset_x3 = -4.0  # 4 meters behind the leader for drone 3

        # Wait for heartbeats from all drones
        self.wait_for_heartbeat(self.master1)
        self.wait_for_heartbeat(self.master2)
        self.wait_for_heartbeat(self.master3)

        # Arm and set to GUIDED mode
        self.arm_and_set_guided(self.master1)
        self.arm_and_set_guided(self.master2)
        self.arm_and_set_guided(self.master3)

    def wait_for_heartbeat(self, master):
        print(f"Waiting for heartbeat from {master.address}")
        master.wait_heartbeat()
        print(f"Heartbeat received from {master.address}")

    def arm_and_set_guided(self, master):
        # Set mode to GUIDED and arm the drone
        while not master.wait_heartbeat().base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED:
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED,
                mavutil.mavlink.MAV_MODE_GUIDED
            )
            time.sleep(1)

        # Arm the drone
        while not master.wait_heartbeat().system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
            master.arducopter_arm()
            time.sleep(1)

        print(f"Drone {master.target_system} armed and set to GUIDED mode.")

    # ... (rest of the code remains the same)

if __name__ == '__main__':
    rclpy.init()
    leader_drone = LeaderDrone()

    # Takeoff to 5 meters
    leader_drone.takeoff(5)

    # Move the leader to different positions, followers will follow
    time.sleep(10)
    leader_drone.move_leader(47.397742, 8.545594, 5)  # Sample GPS coordinates
    time.sleep(10)
    leader_drone.move_leader(47.397642, 8.545494, 5)
    time.sleep(10)
    leader_drone.move_leader(47.397842, 8.545794, 5)

    # Start the following sequence
    leader_drone.follow_leader()
    rclpy.spin(leader_drone)
    rclpy.shutdown()