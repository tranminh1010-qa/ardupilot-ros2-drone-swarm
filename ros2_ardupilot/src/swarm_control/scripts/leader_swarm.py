#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import time
from base_drone import BaseDrone, DroneState
from log_imu_data import IMULogger
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu


def is_position_reached(current_pos, target_pos, tolerance=0.3):
    return all(abs(c - t) < tolerance for c, t in zip(current_pos, target_pos))


class LeaderDrone(BaseDrone):
    def __init__(self):
        super().__init__('leader_drone_node', 0, 'udp:localhost:14550')
        self.position_publisher = self.create_publisher(PoseStamped, '/leader_drone_node/position', 10)
        self.position_timer = self.create_timer(30, self.publish_position)
        self.takeoff_complete_publisher = self.create_publisher(Bool, '/leader_takeoff_complete', 10)
        self.waypoints = [
            (10, 0, 10), (10, 10, 10), (0, 10, 10), (-10, 10, 10),
            (-10, -10, 10), (10, -10, 10), (10, 0, 20), (0, 0, 20), (0, 0, 10)
        ]
        self.imu_logger = IMULogger(self, '/root/ardu_ws/src/swarm_control/imu_log/imu_data_leader.csv', "LEADER")
        self.imu_subscription = self.create_subscription(
            Imu,
            '/leader_drone_node/imu',
            self.imu_callback,
            10)
        self.ekf_check_timer = None
        self.mission_start_timer = self.create_timer(15.0, self.start_mission_timer_callback)


    def imu_callback(self, msg):
        self.imu_logger.log_imu_data(msg)

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
                self.get_logger().info(f"Published leader position: x={msg.x:.2f}, y={msg.y:.2f}, z={-msg.z:.2f}")
            else:
                self.get_logger().warn("Failed to get leader position")

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
            0, self.mav_connection.target_system, self.mav_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
            x, y, -z, 0, 0, 0, 0, 0, 0, 0, 0)

        start = time.time()
        while time.time() - start < 60:
            msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                current_pos = (msg.x, msg.y, -msg.z)
                self.get_logger().info(f"Current position: x={current_pos[0]:.2f}, y={current_pos[1]:.2f}, z={current_pos[2]:.2f}")
                if is_position_reached(current_pos, (x, y, z)):
                    self.get_logger().info(f"Reached position: x={x}, y={y}, z={z}")
                    return True
            time.sleep(1)
        self.get_logger().error(f"Failed to reach position: x={x}, y={y}, z={z}")
        return False

    def takeoff_with_retry(self):
        if self.takeoff_attempts < self.max_attempts:
            self.takeoff_attempts += 1
            self.state = DroneState.TAKING_OFF
            self.get_logger().info(f"Leader: Takeoff attempt {self.takeoff_attempts}")

            if not self.set_guided_mode():
                self.get_logger().error("Failed to set GUIDED mode")
                self.create_timer(5.0, self.takeoff_with_retry)
                return

            if not self.arm_drone():
                self.get_logger().error("Failed to arm the drone")
                self.create_timer(5.0, self.takeoff_with_retry)
                return

            if self.takeoff(10):
                self.get_logger().info("Leader: Takeoff command accepted")

                # Wait for the drone to reach the target altitude
                start_time = time.time()
                while time.time() - start_time < 30:  # Wait up to 30 seconds
                    msg = self.mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                    if msg:
                        relative_alt = msg.relative_alt / 1000.0  # Convert mm to m
                        self.get_logger().info(f"Current altitude: {relative_alt:.2f} m")
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

                self.get_logger().warn("Leader: Takeoff timeout, retrying")
                self.create_timer(5.0, self.takeoff_with_retry)
            else:
                self.get_logger().warn(f"Leader: Takeoff command failed, attempt {self.takeoff_attempts}")
                self.create_timer(5.0, self.takeoff_with_retry)
        else:
            self.get_logger().error(f"Leader: Failed to takeoff after {self.max_attempts} attempts")
            self.state = DroneState.ERROR

    def log_imu_data(self):
        msg = self.mav_connection.recv_match(type='RAW_IMU', blocking=True, timeout=1)
        if msg:
            self.get_logger().info("IMU DATA SAVING LEADER")
            self.imu_logger.log_imu_data(msg)
        else:
            self.get_logger().error("No IMU Data")

    def __del__(self):
        self.imu_logger.close()

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