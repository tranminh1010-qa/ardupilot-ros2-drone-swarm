#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
from base_drone import BaseDrone, DroneState
from log_imu_data import IMULogger
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from datetime import datetime
from rclpy.callback_groups import ReentrantCallbackGroup

from noise_injector import NoiseInjector


def is_position_reached(current_pos, target_pos, tolerance=0.3):
    return all(abs(c - t) < tolerance for c, t in zip(current_pos, target_pos))


class LeaderDrone(BaseDrone):
    def __init__(self):
        super().__init__('leader_drone_node', 0, 'udp:localhost:14551')
        self.current_position = None
        self.target_position = None
        self.declare_parameter("leader_position_topic", "/leader_drone_node/position")
        self._leader_pos_topic = self.get_parameter("leader_position_topic").get_parameter_value().string_value
        self.position_publisher = self.create_publisher(PoseStamped, self._leader_pos_topic, 1)
        self.position_noise = NoiseInjector(mean=0.0, std_dev=0.3, time_correlation=0.8)
        self.velocity_noise = NoiseInjector(mean=0.0, std_dev=0.1, time_correlation=0.6)

        self.takeoff_complete_publisher = self.create_publisher(Bool, '/leader_takeoff_complete', 10)
        self.waypoints = [
            (10, 0, 10), (10, 10, 10), (0, 10, 10), (-10, 10, 10),
            (-10, -10, 10), (10, -10, 10), (10, 0, 20), (0, 0, 20), (0, 0, 10)
        ]
        now = datetime.now()
        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.imu_logger = IMULogger(self, f'/root/ardu_ws/src/swarm_control/imu_log/imu_data_leader_{timestamp}.csv',
                                    "LEADER")
        self.log_imu_timer = self.create_timer(1,  # Set the interval for logging IMU data (adjust as needed)
            self.log_imu_data,
            callback_group=ReentrantCallbackGroup()
        )

    def monitor_takeoff(self):
        start_time = time.time()
        while time.time() - start_time < 30:  # Wait up to 30 seconds
            msg = self.mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                relative_alt = msg.relative_alt / 1000.0  # Convert mm to m
                self.get_logger().debug(f"Current altitude: {relative_alt:.2f} m")
                if abs(relative_alt - 10) < 0.5:  # Within 0.5m of target altitude
                    self.get_logger().info("Leader: Takeoff successful")
                    self.takeoff_attempts = 0
                    self.state = DroneState.FLYING
                    takeoff_complete_msg = Bool()
                    takeoff_complete_msg.data = True
                    self.takeoff_complete_publisher.publish(takeoff_complete_msg)
                    self.start_mission()
                    return
            time.sleep(1)
        self.get_logger().info("Leader: Mission Completed")

    def start_mission(self):
        self.get_logger().info("Starting mission")
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"Moving to waypoint {i + 1}: {wp}")
            if self.goto_position(*wp):
                self.get_logger().info(f"Reached waypoint {i + 1}: {wp}")
                time.sleep(0.5)  # Reduced hover time to 2 seconds
            else:
                self.get_logger().error(f"Failed to reach waypoint {i + 1}: {wp}")
                break
        self.get_logger().info("Mission completed")

    def start_mission_timer_callback(self):
        if self.state == DroneState.CONNECTED:
            self.get_logger().info("Starting mission setup and arming process")
            self.setup_and_arm()
            if not self.ekf_check_timer:
                self.ekf_check_timer = self.create_timer(1.0, self.check_ekf_health)
            self.mission_start_timer.cancel()
        else:
            self.get_logger().warn(f"Not ready to start mission. Current state: {self.state}")

    def publish_position(self):
        self.get_logger().info("Publish position true")
        if self.mav_connection:
            self.get_logger().info("Publish position true")
            msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if msg:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = msg.x
                pose.pose.position.y = msg.y
                pose.pose.position.z = -msg.z  # NED to ENU conversion
                self.position_publisher.publish(pose)
                self.get_logger().info(f"Published leader position: x={msg.x:.2f}, y={msg.y:.2f}, z={-msg.z:.2f}")
            else:
                self.get_logger().warn("Failed to get leader position")

    def goto_position(self, x, y, z):
        self.get_logger().info(f"Original target: x={x}, y={y}, z={z}")
        noisy_pos = self.apply_noise_to_position(x, y, z)
        self.get_logger().info(f"Noisy target: x={noisy_pos[0]:.2f}, y={noisy_pos[1]:.2f}, z={noisy_pos[2]:.2f}")

        self.mav_connection.mav.set_position_target_local_ned_send(
            0, self.mav_connection.target_system, self.mav_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
            noisy_pos[0], noisy_pos[1], -noisy_pos[2], 0, 0, 0, 0, 0, 0, 0, 0)

        start = time.time()
        while time.time() - start < 60:
            msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                current_pos = (msg.x, msg.y, -msg.z)
                target = (x, y, z)
                self.publish_position()

                # Log IMU and position data
                imu_msg = self.mav_connection.recv_match(type='RAW_IMU', blocking=True, timeout=1)
                if imu_msg:
                    self.imu_logger.log_imu_data(imu_msg, current_pos, target)

                # Calculate and log deviation
                deviation = np.linalg.norm(np.array(current_pos) - np.array(target))
                self.get_logger().info(f"Position deviation: {deviation:.2f}m")

                if all(abs(c - t) < 0.3 for c, t in zip(current_pos, target)):
                    return True

            time.sleep(0.1)
        return False

    def on_armed_success(self):
        self.takeoff_with_retry()

    def log_imu_data(self):
        """Timer callback for logging IMU data"""
        if not self.mav_connection or not self.mav_connection.target_system:
            self.get_logger().error("MAVLink connection is not established.")
            return

        try:
            # Get current position
            pos_msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if pos_msg:
                self.current_position = (pos_msg.x, pos_msg.y, -pos_msg.z)

            # Get IMU data
            imu_msg = self.mav_connection.recv_match(type='RAW_IMU', blocking=True, timeout=1)

            if imu_msg and self.current_position:
                self.get_logger().info(f"IMU DATA SAVING LEADER {self.drone_id}")
                self.imu_logger.log_imu_data(
                    imu_msg,
                    self.current_position,
                    self.target_position if self.target_position else self.current_position
                )
            else:
                if not imu_msg:
                    self.get_logger().warn(f"No IMU Data for Leader {self.drone_id}")
                if not self.current_position:
                    self.get_logger().warn(f"No position data available for Leader {self.drone_id}")

        except Exception as e:
            self.get_logger().error(f"Error in IMU logging: {str(e)}")

    def __del__(self):
        if hasattr(self, 'imu_logger') and self.imu_logger is not None:
            self.imu_logger.close()

def main(args=None):
    rclpy.init(args=args)
    leader = LeaderDrone()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(leader)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        leader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()