#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import csv
import enum

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
        self.drone_id = drone_id
        self.mavlink_connection = mavlink_connection
        self.mav_connection = None
        self.armed = False
        self.disarm_requested = False
        self.state = DroneState.INITIALIZING
        self.arming_attempts = 0
        self.takeoff_attempts = 0
        self.max_attempts = 3

        self.connect_timer = self.create_timer(5.0, self.attempt_connect)
        self.create_timer(5.0, self.check_armed_status)
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat_check)

    def attempt_connect(self):
        if self.state == DroneState.INITIALIZING or self.state == DroneState.CONNECTING:
            try:
                self.get_logger().info(f"Attempting to connect to {self.mavlink_connection}")
                self.mav_connection = mavutil.mavlink_connection(self.mavlink_connection, source_system=self.drone_id + 1, timeout=60)
                self.get_logger().info(f"Connection established: {self.mavlink_connection}")
                self.get_logger().info("Waiting for heartbeat...")
                self.mav_connection.wait_heartbeat(timeout=30)
                self.get_logger().info("Heartbeat received!")
                self.connect_timer.cancel()
                self.state = DroneState.CONNECTED

                self.set_ekf_parameters()
            except Exception as e:
                self.get_logger().error(f"Error in connection process: {str(e)}")
                self.state = DroneState.ERROR

    def check_ekf_health(self):
        msg = self.mav_connection.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
        if msg:
            flags = msg.flags
            return (flags & mavutil.mavlink.EKF_ATTITUDE and
                    flags & mavutil.mavlink.EKF_VELOCITY_HORIZ and
                    flags & mavutil.mavlink.EKF_VELOCITY_VERT and
                    flags & mavutil.mavlink.EKF_POS_HORIZ_REL and
                    flags & mavutil.mavlink.EKF_POS_HORIZ_ABS and
                    flags & mavutil.mavlink.EKF_POS_VERT_ABS and
                    flags & mavutil.mavlink.EKF_POS_VERT_AGL and
                    flags & mavutil.mavlink.EKF_CONST_POS_MODE and
                    flags & mavutil.mavlink.EKF_PRED_POS_HORIZ_REL)
        return False

    def set_ekf_parameters(self):
        # Adjust these parameters as needed
        self.mav_connection.param_set_send('EK2_CHECK_SCALE', 200)
        self.mav_connection.param_set_send('EK3_CHECK_SCALE', 200)
        self.mav_connection.param_set_send('EK2_ALT_SOURCE', 0)  # Use barometer for altitude
        self.mav_connection.param_set_send('EK2_GPS_TYPE', 3)  # GPS with SBAS and Baro
        self.mav_connection.param_set_send('EK2_IMU_MASK', 3)  # Use first two IMUs
        self.mav_connection.param_set_send('EK2_MAG_MASK', 1)  # Use first magnetometer
        self.mav_connection.param_set_send('EK2_ALT_SOURCE', 0)  # Use barometer for altitude
        self.mav_connection.param_set_send('AHRS_EKF_TYPE', 2)  # Use EKF2
        self.mav_connection.param_set_send('EK2_ENABLE', 1)
        self.mav_connection.param_set_send('EK3_ENABLE', 0)
        self.mav_connection.param_set_send('GPS_AUTO_CONFIG', 1)
        self.mav_connection.param_set_send('GPS_AUTO_SWITCH', 1)

        self.get_logger().info("EKF parameters set for more permissive arming")

    def setup_and_arm(self):
        if self.set_guided_mode():
            self.set_ekf_parameters()
            self.wait_for_gps()
            self.arm_drone_with_retry()
        else:
            self.get_logger().error("Failed to set GUIDED mode")
            self.state = DroneState.ERROR

    def wait_for_gps(self):
        self.get_logger().info("Waiting for GPS lock...")
        while True:
            msg = self.mav_connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            if msg and msg.fix_type >= 3:
                self.get_logger().info("GPS lock acquired")
                break
            time.sleep(1)

    def set_guided_mode(self):
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,  # 4 is GUIDED mode for ArduCopter
            0, 0, 0, 0, 0)
        self.get_logger().info("Set GUIDED mode command sent")

        start = time.time()
        while time.time() - start < 10:
            msg = self.mav_connection.recv_match(type='COMMAND_ACK', blocking=False)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("GUIDED mode set successfully")
                    return True
        self.get_logger().error("Failed to set GUIDED mode")
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
        self.get_logger().error("Arming failed")
        return False

    def takeoff(self, altitude):
        self.get_logger().info(f"Attempting to takeoff to {altitude} meters")
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)

        start = time.time()
        while time.time() - start < 10:
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

            # Wait for EKF to be healthy
            start_time = time.time()
            while time.time() - start_time < 30:  # Wait up to 30 seconds
                if self.check_ekf_health():
                    break
                self.get_logger().info("Waiting for EKF to be healthy...")
                time.sleep(1)
            else:
                self.get_logger().warn("EKF not healthy after 30 seconds, attempting to arm anyway")

            # Wait for GPS fix
            while True:
                msg = self.mav_connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
                if msg and msg.fix_type >= 3:
                    break
                self.get_logger().info("Waiting for GPS fix...")
                time.sleep(1)

            if self.arm_drone():
                self.get_logger().info(f"Drone {self.drone_id}: Armed successfully")
                self.arming_attempts = 0
                self.state = DroneState.ARMED
                if self.drone_id == 0:  # If this is the leader drone
                    self.takeoff_with_retry()
            else:
                self.get_logger().warn(f"Drone {self.drone_id}: Arming failed, attempt {self.arming_attempts}")
                self.create_timer(5.0, self.arm_drone_with_retry)
        else:
            self.get_logger().error(f"Drone {self.drone_id}: Failed to arm after {self.max_attempts} attempts")
            self.state = DroneState.ERROR

    def takeoff_with_retry(self):
        if self.takeoff_attempts < self.max_attempts:
            self.takeoff_attempts += 1
            self.state = DroneState.TAKING_OFF
            if self.takeoff(10):
                self.get_logger().info(f"Drone {self.drone_id}: Takeoff successful")
                self.takeoff_attempts = 0
                self.state = DroneState.FLYING
            else:
                self.get_logger().warn(f"Drone {self.drone_id}: Takeoff failed, attempt {self.takeoff_attempts}")
                self.create_timer(5.0, self.takeoff_with_retry)
        else:
            self.get_logger().error(f"Drone {self.drone_id}: Failed to takeoff after {self.max_attempts} attempts")
            self.state = DroneState.ERROR


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