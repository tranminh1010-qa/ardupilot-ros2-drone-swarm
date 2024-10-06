#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import sys
import time


class FollowerDrone(Node):
    def __init__(self):
        super().__init__('follower_drone_node')
        self.get_logger().info("FollowerDrone __init__ started")

        # Declare parameters with default values
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('mavlink_connection', 'udp:localhost:14551')
        self.declare_parameter('leader_pos_topic', '/leader_drone_node/position')
        self.declare_parameter('offset', [1.0, 0.0, 0.0])

        # Get parameter values
        self.drone_id = self.get_parameter('drone_id').value
        self.mavlink_connection = self.get_parameter('mavlink_connection').value
        self.leader_pos_topic = self.get_parameter('leader_pos_topic').value
        self.offset = self.get_parameter('offset').value

        # Use drone_id to create unique node name and adjust offset
        self.get_logger().info(f"Initializing Follower Drone {self.drone_id}")
        self.offset = [x * self.drone_id for x in self.offset]

        self.get_logger().info(f"Attempting to establish mavlink connection: {self.mavlink_connection}")

        self.mav_connection = None
        self.connect_timer = self.create_timer(5.0, self.attempt_connect)

        self.create_subscription(PoseStamped, self.leader_pos_topic, self.leader_position_callback, 10)
        self.create_timer(5.0, self.check_armed_status)

        self.get_logger().info(f"Follower drone {self.drone_id} initialized")

    def attempt_connect(self):
        if self.mav_connection is None or not self.mav_connection.target_system:
            try:
                self.get_logger().info(f"Drone {self.drone_id}: Attempting to connect to {self.mavlink_connection}")
                self.mav_connection = mavutil.mavlink_connection(self.mavlink_connection, source_system=self.drone_id+1, timeout=60)
                self.get_logger().info(f"Drone {self.drone_id}: Connection established: {self.mavlink_connection}")
                self.get_logger().info(f"Drone {self.drone_id}: Waiting for heartbeat...")
                self.mav_connection.wait_heartbeat(timeout=30)
                self.get_logger().info(f"Drone {self.drone_id}: Heartbeat received!")
                self.connect_timer.cancel()
                self.pre_arm_routine()
                if self.set_guided_mode():
                    if self.arm_drone():
                        self.get_logger().info(f"Drone {self.drone_id}: Armed successfully")
                        if self.takeoff(10):
                            self.get_logger().info(f"Drone {self.drone_id}: Takeoff successful")
                        else:
                            self.get_logger().error(f"Drone {self.drone_id}: Takeoff failed")
                    else:
                        self.get_logger().error(f"Drone {self.drone_id}: Arming failed")
                else:
                    self.get_logger().error(f"Drone {self.drone_id}: Failed to set GUIDED mode, skipping arming")
            except Exception as e:
                self.get_logger().error(f"Drone {self.drone_id}: Error in connection process: {str(e)}")


    def pre_arm_routine(self):
        self.get_logger().info("Starting pre-arm routine")

        self.mav_connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)
        time.sleep(1)

        self.mav_connection.mav.request_data_stream_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            1,  # 1 Hz
            1  # start
        )
        time.sleep(2)

        # Add gyro calibration
        self.get_logger().info("Starting gyro calibration")
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 1, 0, 0, 0, 0, 0, 0)

        # Wait for gyro calibration to complete
        start_time = time.time()
        while time.time() - start_time < 30:  # Wait up to 30 seconds
            msg = self.mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("Gyro calibration completed successfully")
                    break
        else:
            self.get_logger().error("Gyro calibration timed out")

        # Set EKF home
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0, 0, 0, 0, 0, 0, 0, 0)

        # Set EK2_GPS_TYPE to 0 (None)
        self.mav_connection.mav.param_set_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            b'EK2_GPS_TYPE',
            0,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        # Set EK3_GPS_TYPE to 0 (None)
        self.mav_connection.mav.param_set_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            b'EK3_GPS_TYPE',
            0,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        # Wait for parameter changes to take effect
        time.sleep(2)

        self.get_logger().info("Pre-arm routine completed")

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

        # Wait for mode change acknowledgement
        start = time.time()
        while time.time() - start < 10:
            msg = self.mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("GUIDED mode set successfully")
                    return True
        self.get_logger().error("Failed to set GUIDED mode")
        return False

    def arm_drone(self):
        self.get_logger().info("Attempting to arm drone")
        for attempt in range(3):  # Try 3 times
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)

            start = time.time()
            while time.time() - start < 10:
                msg = self.mav_connection.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=1)
                if msg is not None:
                    if msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            self.get_logger().info("Drone armed successfully.")
                            return True
                        else:
                            self.get_logger().error(f"Arming failed with result: {msg.result}")
                            break
                    elif msg.get_type() == 'STATUSTEXT':
                        self.get_logger().info(f"Status: {msg.text}")

            self.get_logger().warn(f"Arming attempt {attempt + 1} failed. Waiting before retry...")
            time.sleep(5)  # Wait 5 seconds before next attempt

        self.get_logger().error("Arming failed after multiple attempts")
        return False

    def takeoff(self, altitude):
        self.get_logger().info(f"Attempting to takeoff to {altitude} meters")
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)

        # Wait for acknowledgment
        for _ in range(10):  # Try for 10 seconds
            msg = self.mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("Takeoff command accepted")
                    return True
        self.get_logger().error("Takeoff command failed")
        return False

    def leader_position_callback(self, msg):
        if not self.mav_connection or not self.mav_connection.motors_armed():
            self.get_logger().warn(
                f"Drone {self.drone_id}: Not armed or connection not established. Cannot follow leader.")
            return

        self.get_logger().debug(f"Drone {self.drone_id}: leader_position_callback called")

        target_x = msg.pose.position.x + self.offset[0]
        target_y = msg.pose.position.y + self.offset[1]
        target_z = msg.pose.position.z + self.offset[2]

        self.get_logger().info(f"Drone {self.drone_id}: Following leader at offset: {target_x}, {target_y}, {target_z}")

        try:
            self.mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, self.mav_connection.target_system, self.mav_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
                target_x, target_y, -target_z, 0, 0, 0, 0, 0, 0, 0, 0))
        except Exception as e:
            self.get_logger().error(f"Drone {self.drone_id}: Error sending MAVLink message: {str(e)}")

    def check_armed_status(self):
        if self.mav_connection and not self.mav_connection.motors_armed():
            self.get_logger().warn("Drone disarmed unexpectedly. Attempting to re-arm...")
            self.set_guided_mode()
            time.sleep(1)
            self.arm_drone()

    def shutdown(self):
        if self.mav_connection and self.mav_connection.target_system:
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
            self.get_logger().info("Landing command sent to follower drone")


def main(args=None):
    rclpy.init(args=args)
    follower = FollowerDrone()
    try:
        rclpy.spin(follower)
    finally:
        follower.shutdown()
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()