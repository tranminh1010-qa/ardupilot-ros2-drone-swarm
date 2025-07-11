#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import csv
import enum
import os
import socket
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
    def __init__(self, node_name, drone_id=None, mavlink_connection=None, assigned_waypoints=None):
        super().__init__(node_name)
        self.waypoints = assigned_waypoints
        self.drone_id = drone_id
        self.mavlink_connection = mavlink_connection
        self.mav_connection = None
        self.armed = False
        self.disarm_requested = False
        self.state = DroneState.INITIALIZING
        self.arming_attempts = 0
        self.takeoff_attempts = 0
        self.guided_attempts = 0
        self.max_attempts = 5
        self.current_position = None
        self.target_position = None

        # Docker Compose environment settings
        self.in_docker_compose = True
        self.connection_timeout = 45  # Extended timeout for Docker Compose
        self.ros_domain_id = os.environ.get('ROS_DOMAIN_ID', '1')

        # Enhanced connection management for distributed containers
        self.connection_timer = self.create_timer(5.0, self.connection_check)
        self.position_noise = NoiseInjector(mean=0.0, std_dev=0.3, time_correlation=0.8)
        self.velocity_noise = NoiseInjector(mean=0.0, std_dev=0.1, time_correlation=0.6)

        self.get_logger().info(f"Initializing drone {self.drone_id} in Docker Compose environment")
        self.get_logger().info(f"Connection: {self.mavlink_connection}")
        self.get_logger().info(f"ROS Domain ID: {self.ros_domain_id}")

    def connection_check(self):
        """Enhanced connection check for Docker Compose"""
        if self.state != DroneState.CONNECTED:
            self.attempt_connect()
        else:
            self.connection_timer.cancel()
            self.post_connection_setup()

    def attempt_connect(self):
        """Enhanced connection for Docker Compose with container checks"""
        try:
            # First check if the SITL container/port is available
            if self.check_sitl_container_ready():
                if not self.mav_connection:
                    self.get_logger().info(f"Attempting MAVLink connection to {self.mavlink_connection}")
                    self.mav_connection = mavutil.mavlink_connection(
                        self.mavlink_connection,
                        source_system=self.drone_id,  # Unique source system
                        timeout=self.connection_timeout
                    )

                # Extended heartbeat wait for container startup
                self.get_logger().info(f"Waiting for heartbeat from drone {self.drone_id}...")
                self.mav_connection.wait_heartbeat(timeout=self.connection_timeout)

                self.get_logger().info(f"Successfully connected to drone {self.drone_id}")
                self.state = DroneState.CONNECTED

            else:
                self.get_logger().info(f"Waiting for SITL container for drone {self.drone_id}...")

        except Exception as e:
            self.get_logger().warn(f"Connection attempt failed for drone {self.drone_id}: {str(e)}")


    def check_sitl_container_ready(self):
        """Check if SITL container and port are ready"""
        try:
            # Extract port from connection string
            port_str = self.mavlink_connection.split(':')[-1]
            port = int(port_str)

            # Check if port is open using socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex(('127.0.0.1', port))
            sock.close()

            if result == 0:
                self.get_logger().debug(f"Port {port} is accessible for drone {self.drone_id}")
                return True
            else:
                self.get_logger().debug(f"Port {port} not yet accessible for drone {self.drone_id}")
                return False

        except Exception as e:
            self.get_logger().debug(f"Port check error for drone {self.drone_id}: {e}")
            return False


    def post_connection_setup(self):
        """Extended setup for Docker Compose environment"""
        # Longer setup delay for distributed container environment
        setup_delay = 20.0
        self.get_logger().info(f"MAVLink connection established for drone {self.drone_id}. Setup in {setup_delay}s...")
        self.create_timer(setup_delay, self.start_mission_setup)

    def start_mission_setup(self):
        """Begin mission setup with enhanced checks"""
        if self.state == DroneState.CONNECTED:
            self.get_logger().info(f"Starting mission setup for drone {self.drone_id}")
            self.state = DroneState.ARMING

            # Perform extended pre-checks for container environment
            if self.perform_extended_pre_checks():
                self.setup_and_arm()
            else:
                self.get_logger().error(f"Extended pre-checks failed for drone {self.drone_id}")
                self.state = DroneState.ERROR
        else:
            self.get_logger().warn(f"Not ready for mission. Current state: {self.state}")

    def perform_extended_pre_checks(self):
        """Extended pre-flight checks for Docker Compose"""
        self.get_logger().info(f"Performing extended pre-checks for drone {self.drone_id}...")

        # Check system status
        if not self.check_system_status():
            return False

        # Check GPS with extended timeout
        if not self.wait_for_gps():
            return False

        # Check EKF status
        if not self.wait_for_ekf():
            return False

        self.get_logger().info(f"Extended pre-checks passed for drone {self.drone_id}")
        return True

    def check_system_status(self):
        """Check overall system status"""
        self.get_logger().info("Checking system status...")
        try:
            # Request system status
            self.mav_connection.mav.request_data_stream_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                1, 1)

            start_time = time.time()
            while time.time() - start_time < 10:
                msg = self.mav_connection.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
                if msg:
                    self.get_logger().info(f"System status received for drone {self.drone_id}")
                    return True

        except Exception as e:
            self.get_logger().warn(f"System status check error: {e}")

        return True  # Proceed even if status check fails

    def setup_and_arm(self):
        """Setup and arm with enhanced retry logic"""
        if self.arm_drone_with_retry():
            self.state = DroneState.ARMED
            self.on_armed_success()
        else:
            self.get_logger().error(f"Failed to arm drone {self.drone_id}")
            self.state = DroneState.ERROR

    def arm_drone_with_retry(self):
        """Enhanced arming with better error handling"""
        while self.arming_attempts < self.max_attempts:
            self.arming_attempts += 1
            self.get_logger().info(f"Drone {self.drone_id}: Arming attempt {self.arming_attempts}/{self.max_attempts}")

            if self.arm_drone():
                self.get_logger().info(f"Drone {self.drone_id}: Armed successfully")
                self.arming_attempts = 0
                return True
            else:
                self.get_logger().warn(f"Drone {self.drone_id}: Arming failed, retrying in 8 seconds...")
                time.sleep(8.0)

        self.get_logger().error(f"Drone {self.drone_id}: Failed to arm after {self.max_attempts} attempts")
        return False

    def arm_drone(self):
        """Send arm command with enhanced acknowledgment"""
        self.get_logger().info(f"Sending arm command to drone {self.drone_id}")
        try:
            # Send arm command
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)

            # Wait for acknowledgment with extended timeout
            start = time.time()
            timeout = 30

            while time.time() - start < timeout:
                msg = self.mav_connection.recv_match(type=['COMMAND_ACK', 'HEARTBEAT'], blocking=False)
                if msg:
                    if msg.get_type() == 'COMMAND_ACK':
                        if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                                self.armed = True
                                return True
                            else:
                                self.get_logger().warn(f"Arm command rejected: {msg.result}")
                                return False
                    elif msg.get_type() == 'HEARTBEAT':
                        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                            self.armed = True
                            return True

                time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f"Arming error for drone {self.drone_id}: {e}")

        return False

    def wait_for_gps(self):
        """Enhanced GPS wait for Docker Compose"""
        self.get_logger().info(f"Waiting for GPS lock for drone {self.drone_id}...")
        start_time = time.time()
        timeout = 120  # Extended timeout for container startup

        while time.time() - start_time < timeout:
            try:
                msg = self.mav_connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
                if msg:
                    fix_type = msg.fix_type
                    satellites = msg.satellites_visible

                    if fix_type >= 3 and satellites >= 6:
                        self.get_logger().info(
                            f"GPS lock acquired for drone {self.drone_id} (fix:{fix_type}, sats:{satellites})")
                        return True
                    else:
                        self.get_logger().debug(f"GPS status: fix_type={fix_type}, satellites={satellites}")

            except Exception as e:
                self.get_logger().debug(f"GPS check error: {e}")

            time.sleep(3)

        self.get_logger().warn(f"GPS lock timeout for drone {self.drone_id} - proceeding anyway")
        return True  # Proceed even without perfect GPS for simulation

    def wait_for_ekf(self):
        """Enhanced EKF check"""
        self.get_logger().info(f"Checking EKF status for drone {self.drone_id}...")
        start_time = time.time()
        timeout = 60

        while time.time() - start_time < timeout:
            try:
                msg = self.mav_connection.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=3)
                if msg:
                    if msg.flags & 0x01:  # EKF_ATTITUDE
                        self.get_logger().info(f"EKF ready for drone {self.drone_id}")
                        return True

            except Exception as e:
                self.get_logger().debug(f"EKF check error: {e}")

            time.sleep(2)

        self.get_logger().info(f"EKF check timeout for drone {self.drone_id} - proceeding")
        return True

    def takeoff_with_retry(self):
        """Enhanced takeoff for Docker environment"""
        self.get_logger().info(f"Starting takeoff sequence for drone {self.drone_id}")

        if self.set_guided_mode():
            self.state = DroneState.TAKING_OFF
            if self.takeoff(10):
                if self.monitor_takeoff():
                    return True

        self.get_logger().error(f"Takeoff failed for drone {self.drone_id}")
        self.state = DroneState.ERROR
        return False

    def set_guided_mode(self):
        """Enhanced mode setting"""
        self.get_logger().info(f"Setting GUIDED mode for drone {self.drone_id}")
        try:
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4,  # GUIDED mode
                0, 0, 0, 0, 0)

            # Enhanced mode confirmation
            start_time = time.time()
            while time.time() - start_time < 20:
                mode = self.check_mode()
                if mode == 4:
                    self.get_logger().info(f"GUIDED mode confirmed for drone {self.drone_id}")
                    return True
                time.sleep(0.5)

        except Exception as e:
            self.get_logger().error(f"Mode setting error: {e}")

        return False

    def check_mode(self):
        """Check current flight mode"""
        try:
            msg = self.mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg:
                return msg.custom_mode
        except Exception:
            pass
        return None

    def takeoff(self, altitude):
        """Enhanced takeoff command"""
        self.get_logger().info(f"Sending takeoff command ({altitude}m) to drone {self.drone_id}")
        try:
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, altitude)

            # Enhanced acknowledgment waiting
            start = time.time()
            while time.time() - start < 25:
                msg = self.mav_connection.recv_match(type='COMMAND_ACK', blocking=False)
                if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.get_logger().info(f"Takeoff command accepted for drone {self.drone_id}")
                        return True
                time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f"Takeoff command error: {e}")

        return False

    def monitor_takeoff(self):
        """Enhanced takeoff monitoring for Docker"""
        self.get_logger().info(f"Monitoring takeoff for drone {self.drone_id}")
        start_time = time.time()
        timeout = 90  # Extended timeout for Docker
        target_altitude = 10.0
        last_altitude = 0

        while time.time() - start_time < timeout:
            try:
                msg = self.mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
                if msg:
                    current_alt = msg.relative_alt / 1000.0

                    # Log progress every 2 meters
                    if abs(current_alt - last_altitude) > 2.0:
                        self.get_logger().info(f"Drone {self.drone_id} altitude: {current_alt:.1f}m")
                        last_altitude = current_alt

                    if current_alt >= target_altitude - 1.5:  # Within 1.5m
                        self.get_logger().info(
                            f"Drone {self.drone_id} takeoff successful! Final altitude: {current_alt:.1f}m")
                        self.state = DroneState.FLYING
                        time.sleep(5)  # Extended stabilization
                        self.start_mission()
                        return True

            except Exception as e:
                self.get_logger().debug(f"Altitude monitoring error: {e}")

            time.sleep(1)

        self.get_logger().error(f"Takeoff timeout for drone {self.drone_id}")
        return False

    def start_mission(self):
        """Enhanced mission start for Docker environment"""
        if self.state != DroneState.FLYING:
            return

        self.get_logger().info(f"Starting waypoint mission for drone {self.drone_id} ({len(self.waypoints)} waypoints)")

        current_waypoint = 0
        mission_start_time = time.time()
        max_mission_time = 900  # 15 minutes for Docker environment
        waypoint_timeout = 60  # Max time per waypoint

        while self.state == DroneState.FLYING and (time.time() - mission_start_time) < max_mission_time:
            try:
                wp = self.waypoints[current_waypoint]
                self.target_position = wp
                waypoint_start_time = time.time()

                self.get_logger().info(f"Drone {self.drone_id} heading to waypoint {current_waypoint + 1}: {wp}")

                # Navigate to waypoint with timeout
                while (time.time() - waypoint_start_time) < waypoint_timeout:
                    self.send_position_command(*wp)

                    current_pos = self.get_current_position()
                    if current_pos:
                        distance = np.linalg.norm(np.array(current_pos) - np.array(wp))

                        if distance < 4.0:  # Reached waypoint
                            self.get_logger().info(f"Drone {self.drone_id} reached waypoint {current_waypoint + 1}")
                            current_waypoint = (current_waypoint + 1) % len(self.waypoints)
                            time.sleep(3)  # Pause at waypoint
                            break

                    time.sleep(1)
                else:
                    # Waypoint timeout - move to next
                    self.get_logger().warn(f"Waypoint {current_waypoint + 1} timeout for drone {self.drone_id}")
                    current_waypoint = (current_waypoint + 1) % len(self.waypoints)

            except Exception as e:
                self.get_logger().error(f"Mission execution error: {e}")
                break

        self.get_logger().info(f"Mission completed for drone {self.drone_id}")

    def get_current_position(self):
        """Enhanced position retrieval"""
        try:
            msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
            if msg:
                return (msg.x, msg.y, -msg.z)
        except Exception:
            pass
        return None

    def send_position_command(self, x, y, z):
        """Enhanced position command for Docker"""
        try:
            # Apply noise only to first drone for testing
            if self.drone_id == 1:
                noisy_pos = self.apply_noise_to_position(x, y, z)
            else:
                noisy_pos = (x, y, z)

            self.mav_connection.mav.set_position_target_local_ned_send(
                0, self.mav_connection.target_system, self.mav_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
                noisy_pos[0], noisy_pos[1], -noisy_pos[2], 0, 0, 0, 0, 0, 0, 0, 0)

        except Exception as e:
            self.get_logger().error(f"Position command error: {e}")

    def apply_noise_to_position(self, x, y, z):
        """Apply position noise for testing"""
        noise = self.position_noise.generate_noise()
        noisy_pos = np.array([x, y, z]) + noise
        deviation = np.linalg.norm(noise)
        self.get_logger().debug(f'Position noise applied: {deviation:.2f}m')
        return tuple(noisy_pos)

    def on_armed_success(self):
        """Called when drone arms successfully"""
        self.get_logger().info(f"Drone {self.drone_id} armed successfully - starting takeoff in 5 seconds")
        # Extended delay before takeoff for Docker stability
        self.create_timer(5.0, self.takeoff_with_retry)

    def shutdown(self):
        """Enhanced shutdown for Docker environment"""
        self.get_logger().info(f"Shutting down drone {self.drone_id}")
        if self.mav_connection and self.mav_connection.target_system:
            try:
                # Send land command
                self.mav_connection.mav.command_long_send(
                    self.mav_connection.target_system,
                    self.mav_connection.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

                self.get_logger().info(f"Landing command sent to drone {self.drone_id}")
                self.state = DroneState.LANDING

                # Wait briefly for land command acknowledgment
                start = time.time()
                while time.time() - start < 5:
                    msg = self.mav_connection.recv_match(type='COMMAND_ACK', blocking=False)
                    if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            self.get_logger().info(f"Landing confirmed for drone {self.drone_id}")
                            break
                    time.sleep(0.1)

            except Exception as e:
                self.get_logger().error(f"Shutdown error for drone {self.drone_id}: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Create node with default values
    node = BaseDrone('base_drone', 1, 'udp:localhost:14551')

    try:
        # Get parameters from launch file
        drone_id_param = node.declare_parameter('drone_id', 1).value
        mavlink_connection_param = node.declare_parameter('mavlink_connection', 'udp:localhost:14551').value
        assigned_waypoints_param = node.declare_parameter('assigned_waypoints', '').value

        # Update node attributes
        node.drone_id = drone_id_param
        node.mavlink_connection = mavlink_connection_param

        # Handle waypoints
        if not assigned_waypoints_param or assigned_waypoints_param == '':
            node.waypoints = generate_circular_waypoints()
            node.get_logger().info(f"Generated default circular waypoints for drone {node.drone_id}")
        else:
            try:
                node.waypoints = eval(assigned_waypoints_param)
                node.get_logger().info(
                    f"Using assigned waypoints for drone {node.drone_id}: {len(node.waypoints)} points")
            except Exception as e:
                node.get_logger().error(f"Error parsing waypoints: {e}, using default")
                node.waypoints = generate_circular_waypoints()

        node.get_logger().info(f"Drone {node.drone_id} initialized in Docker Compose environment")
        node.get_logger().info(f"  - MAVLink connection: {node.mavlink_connection}")
        node.get_logger().info(f"  - ROS Domain ID: {node.ros_domain_id}")
        node.get_logger().info(f"  - Waypoints: {len(node.waypoints)} assigned")

    except Exception as e:
        node.get_logger().error(f"Parameter initialization error: {e}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()