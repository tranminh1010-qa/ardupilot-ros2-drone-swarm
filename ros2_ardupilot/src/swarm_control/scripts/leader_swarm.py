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
        radius = 30.0
        height = 8.0
        points = 12  # Number of points in the circle

        self.waypoints = []
        for i in range(points):
            angle = 2 * np.pi * i / points
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            self.waypoints.append((x, y, height))

        now = datetime.now()
        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.imu_logger = IMULogger(self, f'/root/ardu_ws/src/swarm_control/imu_log/imu_data_leader_{timestamp}.csv',
                                    "LEADER")
        self.log_imu_timer = self.create_timer(0.1,
            self.log_imu_data,
            callback_group=ReentrantCallbackGroup()
        )
        self.position_publish_timer = self.create_timer(0.1, self.publish_position)  # 10Hz publishing rate
        self.setup_data_streams_timer = self.create_timer(1.0, self.setup_data_streams)
        self.streams_configured = False

    def setup_data_streams(self):
        """Configure MAVLink data streams with appropriate rates."""
        if not self.mav_connection or not self.mav_connection.target_system:
            return

        try:
            # Configure streams with higher rates
            streams = {
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS: 50,  # IMU data
                mavutil.mavlink.MAV_DATA_STREAM_POSITION: 50,  # Position data
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: 50,  # Attitude data
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS: 50,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS: 50,
            }

            for stream_id, rate in streams.items():
                self.mav_connection.mav.request_data_stream_send(
                    self.mav_connection.target_system,
                    self.mav_connection.target_component,
                    stream_id,
                    rate,  # Hz
                    1  # Start sending
                )

            self.streams_configured = True
            self.get_logger().info("MAVLink data streams configured successfully")

            # Once configured, stop the timer
            self.setup_data_streams_timer.cancel()

        except Exception as e:
            self.get_logger().error(f"Error configuring data streams: {str(e)}")

    def monitor_takeoff(self):
        start_time = time.time()
        reached_altitude = False

        while time.time() - start_time < 30:  # 30 second timeout
            msg = self.mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                relative_alt = msg.relative_alt / 1000.0  # Convert mm to m
                self.get_logger().info(f"Current altitude: {relative_alt:.2f} m")

                if abs(relative_alt - 10) < 0.5:  # Within 0.5m of target altitude
                    reached_altitude = True
                    break

            time.sleep(0.1)

        if reached_altitude:
            self.get_logger().info("Leader: Takeoff successful")
            self.state = DroneState.FLYING
            self.publish_takeoff_complete()
            time.sleep(2)  # Allow time for stabilization
            self.start_mission()
            return True
        else:
            self.get_logger().error("Leader: Failed to reach takeoff altitude")
            return False

    def publish_takeoff_complete(self):
        msg = Bool()
        msg.data = True
        self.takeoff_complete_publisher.publish(msg)
        self.get_logger().info("Published takeoff complete message")

    def start_mission(self):
        if self.state != DroneState.FLYING:
            return

        # First stabilize at initial position
        initial_pos = (0, 0, 10)
        self.goto_position(*initial_pos)

        current_waypoint = 0
        while self.state == DroneState.FLYING:
            wp = self.waypoints[current_waypoint]
            self.target_position = wp
            self.send_position_command(*wp)

            # Check if we're close enough to current waypoint
            msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                current_pos = (msg.x, msg.y, -msg.z)
                distance = np.linalg.norm(np.array(current_pos) - np.array(wp))

                # If we're within 1 meter of the waypoint, move to next one
                if distance < 1.0:
                    current_waypoint = (current_waypoint + 1) % len(self.waypoints)
                    time.sleep(0.5)  # Brief pause at waypoint

            time.sleep(0.1)  # Control rate

    def send_position_command(self, x, y, z):
        try:
            noisy_pos = self.apply_noise_to_position(x, y, z)
            self.mav_connection.mav.set_position_target_local_ned_send(
                0,
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                noisy_pos[0], noisy_pos[1], -noisy_pos[2],
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0  # yaw
            )
            self.publish_position()

        except Exception as e:
            self.get_logger().error(f"MAVLink command failed: {str(e)}")

    def publish_position(self):
        if not self.mav_connection or self.state != DroneState.FLYING:
            return

        try:
            # Request position data explicitly
            self.mav_connection.mav.request_data_stream_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                50,  # 50 Hz
                1
            )

            msg = self.mav_connection.recv_match(
                type='LOCAL_POSITION_NED',
                blocking=True,
                timeout=0.1
            )

            if msg:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = msg.x
                pose.pose.position.y = msg.y
                pose.pose.position.z = -msg.z
                self.position_publisher.publish(pose)

        except Exception as e:
            self.get_logger().error(f"Error publishing position: {str(e)}")

    def goto_position(self, x, y, z):
        if self.state != DroneState.FLYING:
            self.get_logger().error("Cannot goto position - drone not in FLYING state")
            return False

        self.get_logger().info(f"Original target: x={x}, y={y}, z={z}")
        noisy_pos = self.apply_noise_to_position(x, y, z)

        try:
            self.mav_connection.mav.set_position_target_local_ned_send(
                0, self.mav_connection.target_system, self.mav_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
                noisy_pos[0], noisy_pos[1], -noisy_pos[2], 0, 0, 0, 0, 0, 0, 0, 0)

            # Wait for position to be reached
            return self.wait_for_position(x, y, z)
        except Exception as e:
            self.get_logger().error(f"Error sending position command: {str(e)}")
            return False

    def wait_for_position(self, x, y, z, timeout=60, tolerance=0.3):
        start = time.time()
        while time.time() - start < timeout:
            msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                current_pos = (msg.x, msg.y, -msg.z)
                self.publish_position()

                deviation = np.linalg.norm(np.array(current_pos) - np.array((x, y, z)))
                self.get_logger().info(f"Position deviation: {deviation:.2f}m")

                if deviation < tolerance:
                    return True
            time.sleep(0.1)
        return False
    def on_armed_success(self):
        self.takeoff_with_retry()


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