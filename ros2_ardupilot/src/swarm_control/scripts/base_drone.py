#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import csv
import enum
from noise_injector import NoiseInjector
import numpy as np

class DroneState(enum.Enum):
    INITIALIZING = 0
    CONNECTING = 1
    CONNECTED = 2
    ARMING = 3
    ARMED = 4
    TAKING_OFF = 5
    FLYING = 6
    LANDING = 7
    ERROR = 8


class BaseDrone(Node):
    def __init__(self, node_name, drone_id, mavlink_connection):
        super().__init__(node_name)
        self.waypoints = None
        self.takeoff_complete_publisher = None
        self.imu_subscription = None
        self.imu_logger = None
        self.position_publisher = None
        self.mission_start_timer = None
        self.position_timer = None
        self.ekf_check_timer = None
        self.drone_id = drone_id
        self.mavlink_connection = mavlink_connection
        self.mav_connection = None
        self.armed = False
        self.disarm_requested = False
        self.state = DroneState.INITIALIZING
        self.arming_attempts = 0
        self.takeoff_attempts = 0
        self.guided_attempts = 0
        self.max_attempts = 3

        self.connection_timer = self.create_timer(1.0, self.connection_check)
        self.position_noise = NoiseInjector(mean=0.0, std_dev=0.3, time_correlation=0.8)
        self.velocity_noise = NoiseInjector(mean=0.0, std_dev=0.1, time_correlation=0.6)

    def connection_check(self):
        if self.state != DroneState.CONNECTED:
            self.attempt_connect()
        else:
            self.connection_timer.cancel()
            self.post_connection_setup()

    def apply_noise_to_position(self, x, y, z):
        noise = self.position_noise.generate_noise()
        noisy_pos = np.array([x, y, z]) + noise
        deviation = np.linalg.norm(noise)
        self.get_logger().info(f'Position deviation: {deviation:.2f}m')
        return tuple(noisy_pos)

    def attempt_connect(self):
        try:
            if not self.mav_connection:
                self.mav_connection = mavutil.mavlink_connection(self.mavlink_connection,
                                                                 source_system=self.drone_id + 1)

            self.mav_connection.wait_heartbeat(timeout=5)
            self.get_logger().info(f"Connected to FCU on {self.mavlink_connection}")
            self.state = DroneState.CONNECTED
        except Exception as e:
            self.get_logger().warn(f"Connection attempt failed: {str(e)}")

    def post_connection_setup(self):
        self.create_timer(5.0, self.start_mission_setup)

    def start_mission_setup(self):
        if self.state == DroneState.CONNECTED:
            self.get_logger().info("Starting mission setup and arming process")
            self.state = DroneState.ARMING
            self.setup_and_arm()
        else:
            self.get_logger().warn(f"Not ready to start mission. Current state: {self.state}")

    def setup_and_arm(self):
        if self.arm_drone_with_retry():
            #    self.set_ekf_parameters()
            self.state = DroneState.ARMED
        else:
            self.get_logger().error("Failed to Arm drone")
            self.state = DroneState.ERROR
            return

        # current_mode = self.get_current_mode()
        # if current_mode != 'GUIDED':
        #     if not self.set_guided_mode_with_retry():
        #         self.state = DroneState.ERROR
                # return

        # self.arm_drone_with_retry()

    def check_ekf_health(self):
        msg = self.mav_connection.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
        if msg:
            flags = msg.flags
            required_flags = [
                (mavutil.mavlink.EKF_ATTITUDE, "Attitude"),
                (mavutil.mavlink.EKF_VELOCITY_HORIZ, "Horizontal Velocity"),
                (mavutil.mavlink.EKF_VELOCITY_VERT, "Vertical Velocity"),
                (mavutil.mavlink.EKF_POS_HORIZ_REL, "Relative Horizontal Position"),
                (mavutil.mavlink.EKF_POS_HORIZ_ABS, "Absolute Horizontal Position"),
                (mavutil.mavlink.EKF_POS_VERT_ABS, "Absolute Vertical Position"),
                (mavutil.mavlink.EKF_POS_VERT_AGL, "Vertical Position AGL"),
                (mavutil.mavlink.EKF_CONST_POS_MODE, "Constant Position Mode"),
                (mavutil.mavlink.EKF_PRED_POS_HORIZ_REL, "Predicted Horizontal Position")
            ]

            all_flags_set = True
            for flag, name in required_flags:
                if not flags & flag:
                    self.get_logger().warn(f"EKF flag not set: {name}")
                    all_flags_set = False

            if all_flags_set:
                self.get_logger().info("EKF is healthy")
            else:
                self.get_logger().warn("EKF is not healthy")
            return all_flags_set
        else:
            self.get_logger().warn("No EKF_STATUS_REPORT received")
            return False

    def wait_for_gps(self):
        self.get_logger().info("Waiting for GPS lock...")
        start_time = time.time()
        while time.time() - start_time < 30:  # Wait up to 30 seconds
            msg = self.mav_connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            if msg:
                self.get_logger().info(
                    f"GPS status: fix_type={msg.fix_type}, satellites_visible={msg.satellites_visible}")
                if msg.fix_type >= 3:
                    self.get_logger().info("GPS lock acquired")
                    return True
            time.sleep(1)
        self.get_logger().error("Failed to acquire GPS lock")
        return False

    def arm_drone(self):
        self.get_logger().info("Attempting to arm drone")
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)

        start = time.time()
        while time.time() - start < 10:
            msg = self.mav_connection.recv_match(type=['COMMAND_ACK', 'HEARTBEAT'], blocking=False)
            if msg is not None:
                if msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.armed = True
                        self.disarm_requested = False
                        self.get_logger().info("Drone armed successfully.")
                        return True
                elif msg.get_type() == 'HEARTBEAT':
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        self.armed = True
                        self.disarm_requested = False
                        self.get_logger().info("Drone armed successfully (confirmed by HEARTBEAT).")
                        return True
        return False

    def set_guided_mode(self):
        self.get_logger().info("Attempting to set GUIDED mode")
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,  # 4 is GUIDED mode for ArduCopter
            0, 0, 0, 0, 0)
        self.get_logger().info("Set GUIDED mode command sent")
        if self.check_mode():
            return True
        else:
            self.get_logger().error(f"Failed to set GUIDED mode")
            return False

    def get_current_mode(self):
        msg = self.mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            self.get_logger().info(f"Current flight mode: {mode}")
            return mode
        self.get_logger().warn("Failed to get current flight mode")
        return None

    def takeoff(self, altitude):
        self.get_logger().info(f"Attempting to takeoff to {altitude} meters")
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)

        start = time.time()
        while time.time() - start < 20:  # Increased timeout to 20 seconds
            msg = self.mav_connection.recv_match(type='COMMAND_ACK', blocking=False)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("Takeoff command accepted")
                    return True
        self.get_logger().error("Takeoff command failed")
        return False

    def check_armed_status(self):
        if self.mav_connection and not self.mav_connection.motors_armed() and self.armed and not self.disarm_requested:
            self.get_logger().warn("Drone disarmed unexpectedly. Attempting to re-arm...")
            self.state = DroneState.ARMING
            self.set_guided_mode()
            time.sleep(1)
            self.arm_drone()

    def arm_drone_with_retry(self):
        if self.arming_attempts < self.max_attempts:
            self.arming_attempts += 1
            self.get_logger().info(f"Drone {self.drone_id}: Arming attempt {self.arming_attempts}")

            if not self.check_pre_arm_status():
                self.get_logger().warn("Pre-arm checks failed, attempting to arm anyway")

            if self.arm_drone():
                self.get_logger().info(f"Drone {self.drone_id}: Armed successfully")
                self.arming_attempts = 0
                self.state = DroneState.ARMED
                self.on_armed_success()
                return True
            else:
                self.get_logger().warn(f"Drone {self.drone_id}: Arming failed, attempt {self.arming_attempts}")
                self.create_timer(5.0, self.arm_drone_with_retry)
        else:
            self.get_logger().error(f"Drone {self.drone_id}: Failed to arm after {self.max_attempts} attempts")
            self.state = DroneState.ERROR

    def check_pre_arm_status(self):
        start_time = time.time()
        while time.time() - start_time < 10:  # Check for 10 seconds
            msg = self.mav_connection.recv_match(type='STATUSTEXT', blocking=False)
            if msg and "PreArm" in msg.text:
                self.get_logger().warn(f"Pre-arm check: {msg.text}")
                return False
            time.sleep(0.1)
        return True

    def takeoff_with_retry(self):

        if self.wait_for_gps():
            guided_attempt = 0
            while not self.state == DroneState.FLYING:
                self.get_logger().info(f"Drone: {self.drone_id} guided set attempt {guided_attempt}")
                if self.set_guided_mode():
                    self.state = DroneState.TAKING_OFF
                    if self.takeoff(10):
                        self.get_logger().info(f"Drone: {self.drone_id} Takeoff command accepted")
                        self.monitor_takeoff()
                        return
                else:
                    guided_attempt += 1
        else:
            self.get_logger().error(f"Drone: {self.drone_id} Failed to takeoff after {self.max_attempts} attempts")
            self.state = DroneState.ERROR

    def monitor_takeoff(self):
        start_time = time.time()
        while time.time() - start_time < 30:  # Monitor for 30 seconds
            msg = self.mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                relative_alt = msg.relative_alt / 1000.0  # Convert mm to m
                self.get_logger().info(f"Current altitude: {relative_alt:.2f} m")
                if abs(relative_alt - 10) < 0.5:  # Within 0.5m of target altitude
                    self.get_logger().info(f"Drone {self.drone_id}: Takeoff successful")
                    self.takeoff_attempts = 0
                    self.state = DroneState.FLYING
                    return
            time.sleep(1)
        self.get_logger().warn(f"Drone {self.drone_id}: Takeoff timeout, retrying")
        self.takeoff_with_retry()

    def check_mode(self):
        msg = self.mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            custom_mode = msg.custom_mode
            if custom_mode == 4:
                return True
            else:
                return False

    def heartbeat_check(self):
        if not self.mav_connection or not self.mav_connection.target_system:
            self.get_logger().warn(f"Drone {self.drone_id}: No heartbeat received from FCU")
        else:
            self.get_logger().debug(f"Drone {self.drone_id}: Heartbeat received from FCU")

    def shutdown(self):
        self.disarm_requested = True
        if self.mav_connection and self.mav_connection.target_system:
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
            self.get_logger().info("Landing command sent to drone")
            self.state = DroneState.LANDING

    def recover_from_error(self):
        self.get_logger().info(f"Drone {self.drone_id}: Attempting error recovery")
        self.arming_attempts = 0
        self.takeoff_attempts = 0
        self.state = DroneState.INITIALIZING
        self.attempt_connect()
