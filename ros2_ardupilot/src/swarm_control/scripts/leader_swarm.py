#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
import csv
from std_msgs.msg import Bool


class LeaderDrone(Node):
    def __init__(self):
        super().__init__('leader_drone_node')
        self.get_logger().info("LeaderDrone __init__ started")

        self.declare_parameter('mavlink_connection', 'udp:localhost:14550')
        self.mavlink_connection = self.get_parameter('mavlink_connection').value

        self.mav_connection = None
        self.connect_timer = self.create_timer(5.0, self.attempt_connect)

        self.position_publisher = self.create_publisher(PoseStamped, '/leader_drone_node/position', 10)
        self.position_timer = self.create_timer(1.0, self.publish_position)
        self.takeoff_complete_publisher = self.create_publisher(Bool, '/leader_takeoff_complete', 10)
        self.waypoints = [
            (10, 0, 10),
            (10, 10, 10),
            (0, 10, 10),
            (-10, 10, 10),
            (-10, -10, 10),
            (10, -10, 10),
            (10, 0, 20),
            (0, 0, 20),
            (0, 0, 10)
        ]
        # Open a CSV file to write the IMU data
        self.csv_file = open('/root/ardu_ws/src/swarm_control/imu_log/imu_data_leader.csv', mode='a') 
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["LEADER"])
        self.csv_writer.writerow(["Time", "Orientation X", "Orientation Y", "Orientation Z",
                                "Angular Velocity X", "Angular Velocity Y", "Angular Velocity Z",
                                "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"])

    def attempt_connect(self):
        if self.mav_connection is None or not self.mav_connection.target_system:
            try:
                self.get_logger().info(f"Attempting to connect to {self.mavlink_connection}")
                self.mav_connection = mavutil.mavlink_connection(self.mavlink_connection, source_system=1, timeout=60)
                self.get_logger().info(f"Connection established: {self.mavlink_connection}")
                self.get_logger().info("Waiting for heartbeat...")
                self.mav_connection.wait_heartbeat(timeout=30)
                self.get_logger().info("Heartbeat received!")
                self.connect_timer.cancel()
                self.setup_and_arm()
                self.imu_timer = self.create_timer(1.0, self.imu_values)
            except Exception as e:
                self.get_logger().error(f"Error in connection process: {str(e)}")

    def setup_and_arm(self):
        self.set_guided_mode()
        time.sleep(2)
        self.wait_for_position_estimate()
        if self.arm_drone():
            self.get_logger().info("Drone armed successfully")
            if self.takeoff(10):
                self.get_logger().info("Takeoff successful")
                self.start_mission()
            else:
                self.get_logger().error("Takeoff failed")
        else:
            self.get_logger().error("Arming failed")

    def wait_for_position_estimate(self):
        self.get_logger().info("Waiting for position estimate...")
        while True:
            msg = self.mav_connection.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=10)
            if msg and (msg.flags & mavutil.mavlink.EKF_PRED_POS_HORIZ_ABS):
                self.get_logger().info("Got position estimate")
                return True
            self.get_logger().info("Still waiting for position estimate...")
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
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)

        # Wait for armed state
        start = time.time()
        while time.time() - start < 30:
            msg = self.mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                self.get_logger().info("Drone armed successfully.")
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

        # Wait for reached target altitude
        start = time.time()
        while time.time() - start < 60:
            msg = self.mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if msg and (msg.relative_alt / 1000.0) >= altitude * 0.95:
                self.get_logger().info("Takeoff successful")
                takeoff_msg = Bool()
                takeoff_msg.data = True
                self.takeoff_complete_publisher.publish(takeoff_msg)
                return True
        self.get_logger().error("Takeoff failed")
        return False

    def start_mission(self):
        self.get_logger().info("Starting mission")
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"Moving to waypoint {i + 1}: {wp}")
            if self.goto_position(*wp):
                self.get_logger().info(f"Reached waypoint {i + 1}: {wp}")
                time.sleep(2)  # Reduced hover time to 2 seconds
            else:
                self.get_logger().error(f"Failed to reach waypoint {i + 1}: {wp}")
                break
        self.get_logger().info("Mission completed")

    def goto_position(self, x, y, z):
        self.get_logger().info(f"Sending goto command: x={x}, y={y}, z={z}")
        self.mav_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (only positions enabled)
            x, y, -z,  # x, y, z positions (z is negative in NED frame)
            0, 0, 0,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)  # yaw, yaw_rate

        # Wait for reaching the position
        start = time.time()
        while time.time() - start < 60:
            msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                current_pos = (msg.x, msg.y, -msg.z)
                self.get_logger().info(
                    f"Current position: x={current_pos[0]:.2f}, y={current_pos[1]:.2f}, z={current_pos[2]:.2f}")
                if self.is_position_reached(current_pos, (x, y, z)):
                    self.get_logger().info(f"Reached position: x={x}, y={y}, z={z}")
                    return True
            time.sleep(3)  # Check every second
        self.get_logger().error(f"Failed to reach position: x={x}, y={y}, z={z}")
        return False

    def is_position_reached(self, current_pos, target_pos, tolerance=0.3):
        return all(abs(c - t) < tolerance for c, t in zip(current_pos, target_pos))

    def publish_position(self):
        if self.mav_connection:
            msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if msg:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = msg.x
                pose.pose.position.y = msg.y
                pose.pose.position.z = -msg.z  # NED to ENU conversion
                self.position_publisher.publish(pose)
                self.get_logger().debug(f"Published position: x={msg.x:.2f}, y={msg.y:.2f}, z={-msg.z:.2f}")

    def test_movement(self):
        self.get_logger().info("Testing basic movement...")
        self.mav_connection.mav.rc_channels_override_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            1500, 1600, 1500, 1500, 1500, 1500, 1500, 1500)  # Pitch forward
        time.sleep(5)
        self.mav_connection.mav.rc_channels_override_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500)  # Center sticks
        self.get_logger().info("Basic movement test completed")
    
    def imu_values(self):
        while self.arm_drone():
            msg = self.mav_connection.recv_match(type='RAW_IMU', blocking=True, timeout=1)
            if msg:
                self.get_logger().info("IMU DATA SAVING LEADER")
                self.csv_writer.writerow([msg.time_usec, msg.xacc, msg.yacc, msg.zacc,
                                    msg.xgyro, msg.ygyro, msg.zgyro,
                                    msg.xmag, msg.ymag, msg.zmag])
            else:
                self.get_logger().error(f"No IMU Data")

    def __del__(self):
        # Close the CSV file when done
        self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    leader = LeaderDrone()
    try:
        rclpy.spin(leader)
    except KeyboardInterrupt:
        pass
    finally:
        leader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()