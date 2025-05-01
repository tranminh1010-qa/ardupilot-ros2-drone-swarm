#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import csv
import enum
from noise_injector import NoiseInjector
from points_distributor import generate_circular_waypoints

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
    def __init__(self, node_name, drone_id, mavlink_connection, assigned_waypoints=None):
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

        # If no waypoints are assigned, create default circular waypoints
        if assigned_waypoints is None:
            self.waypoints = generate_circular_waypoints()
        else:
            self.waypoints = eval(assigned_waypoints)
        self.get_logger().info("waypoints assigned to drone {}: {}".format(drone_id, self.waypoints))


    def connection_check(self):
        if self.state != DroneState.CONNECTED:
            self.attempt_connect()
        else:
            self.connection_timer.cancel()
            self.post_connection_setup()

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
            self.state = DroneState.ARMED
        else:
            self.get_logger().error("Failed to Arm drone")
            self.state = DroneState.ERROR
            return

    def arm_drone_with_retry(self):
        if self.arming_attempts < self.max_attempts:
            self.arming_attempts += 1
            self.get_logger().info(f"Drone {self.drone_id}: Arming attempt {self.arming_attempts}")

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

    def check_mode(self):
        msg = self.mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            custom_mode = msg.custom_mode
            if custom_mode == 4:
                return True
            else:
                return False
        return None

    def log_imu_data(self):
        """Timer callback for logging IMU data"""
        if not self.mav_connection or not self.mav_connection.target_system:
            self.get_logger().error("MAVLink connection is not established.")
            return

        try:
            # Request fresh IMU data
            self.mav_connection.mav.request_data_stream_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                50,  # 50 Hz
                1
            )

            # Get current position with shorter timeout
            pos_msg = self.mav_connection.recv_match(
                type='LOCAL_POSITION_NED',
                blocking=True,
                timeout=0.1
            )

            if pos_msg:
                self.current_position = (pos_msg.x, pos_msg.y, -pos_msg.z)
            else:
                self.get_logger().warn(f"No position data available for Drone {self.drone_id}")
                return

            # Get IMU data with shorter timeout
            imu_msg = self.mav_connection.recv_match(
                type='RAW_IMU',
                blocking=True,
                timeout=0.1
            )

            if imu_msg:
                self.get_logger().info(f"IMU DATA SAVING DRONE {self.drone_id}")
                self.imu_logger.log_imu_data(
                    imu_msg,
                    self.current_position,
                    self.target_position if self.target_position else self.current_position
                )
            else:
                self.get_logger().warn(f"No IMU Data for Drone {self.drone_id}")

        except Exception as e:
            self.get_logger().error(f"Error in IMU logging: {str(e)}")

    def apply_noise_to_position(self, x, y, z):
        noise = self.position_noise.generate_noise()
        noisy_pos = np.array([x, y, z]) + noise
        deviation = np.linalg.norm(noise)
        self.get_logger().info(f'Position deviation: {deviation:.2f}m')
        return tuple(noisy_pos)

    def shutdown(self):
        self.disarm_requested = True
        if self.mav_connection and self.mav_connection.target_system:
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
            self.get_logger().info("Landing command sent to drone")
            self.state = DroneState.LANDING

    def on_armed_success(self):
        node_name = self.get_name()
        self.get_logger().info(f"This is Drone Node {node_name}")
        self.takeoff_with_retry()

    def perform_action_if_base_drone(self):
        # Check if the node name is 'base_drone'
        if self.get_name() == 'base_drone':
            self.get_logger().info("This is the base_drone node. Executing action.")
            # Place your action here.
            return True
        else:
            self.get_logger().info("Not the base_drone node. Action skipped.")
            return False

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
            self.get_logger().info(" Takeoff successful")
            self.state = DroneState.FLYING
            if self.get_name() == 'leader_drone_node':
                self.publish_takeoff_complete()
            time.sleep(2)  # Allow time for stabilization
            self.start_mission()
            return True
        else:
            self.get_logger().error("Failed to reach takeoff altitude")
            return False

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
            if self.get_name() == 'leader_drone_node':
                noisy_pos = self.apply_noise_to_position(x, y, z)
            else:
                noisy_pos = (x,y,z)
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
            if self.get_name() == 'leader_drone_node':
                self.publish_position()
        except Exception as e:
            self.get_logger().error(f"MAVLink command failed: {str(e)}")

    def goto_position(self, x, y, z):
        if self.state != DroneState.FLYING:
            self.get_logger().error("Cannot goto position - drone not in FLYING state")
            return False

        self.get_logger().info(f"Original target: x={x}, y={y}, z={z}")

        if self.get_name() == 'leader_drone_node':
            noisy_pos = self.apply_noise_to_position(x, y, z)
        else:
            noisy_pos = [x,y,z]

        try:
            self.mav_connection.mav.set_position_target_local_ned_send(
                0, self.mav_connection.target_system, self.mav_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
                noisy_pos[0], noisy_pos[1], -noisy_pos[2], 0, 0, 0, 0, 0, 0, 0, 0)

            # Wait for position to be reached
            if self.get_name() == 'leader_drone_node':
                return self.wait_for_position(x, y, z)
            else:
                return (x, y, z)
        except Exception as e:
            self.get_logger().error(f"Error sending position command: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)

    node = BaseDrone('base_drone', 1, 'udp:localhost:14551')  # default values

    # Retrieve parameters from the ROS parameter server (if set)
    drone_id_param = node.declare_parameter('drone_id', 1).value
    mavlink_connection_param = node.declare_parameter('mavlink_connection', 'udp:localhost:14551').value

    # Update the node's attributes if parameters are provided
    node.drone_id = drone_id_param
    node.mavlink_connection = mavlink_connection_param

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()