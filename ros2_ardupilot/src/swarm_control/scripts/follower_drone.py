#!/usr/bin/env python3
from base_drone import BaseDrone, DroneState
from log_imu_data import IMULogger
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
from sensor_msgs.msg import Imu
import rclpy
from datetime import datetime
import time

class FollowerDrone(BaseDrone):
    def __init__(self, node_name, drone_id, mavlink_connection, leader_pos_topic, offset, follow_distance):
        super().__init__(node_name, drone_id, mavlink_connection)

        # Declare parame ters
        self.declare_parameter('drone_id', drone_id)
        self.declare_parameter('mavlink_connection', mavlink_connection)
        self.declare_parameter('leader_pos_topic', leader_pos_topic)
        self.declare_parameter('offset', offset)
        self.declare_parameter('follow_distance', follow_distance)

        # Get parameter values
        self.drone_id = self.get_parameter('drone_id').value
        self.mavlink_connection = self.get_parameter('mavlink_connection').value
        self.leader_pos_topic = self.get_parameter('leader_pos_topic').get_parameter_value().string_value
        self.offset = self.get_parameter('offset').value
        self.follow_distance = self.get_parameter('follow_distance').value

        self.leader_pos_topic = leader_pos_topic
        self.offset = offset
        self.follow_distance = follow_distance
        self.leader_position = None
        self.last_command_time = self.get_clock().now()
        self.takeoff_subscription = None
        self.leader_takeoff_complete = False
        self.command_frequency = 5.0  # Hz

        # Attempt to subscribe
        self.create_takeoff_subscription()
        
        # Check for subscription status
        if not self.takeoff_subscription:
            self.get_logger().error('Unable to create takeoff subscription. Node will not start.')
            return

        # Start timer to check if takeoff message is received
        self.timer = self.create_timer(5.0, self.check_takeoff)

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
        )
        self.create_subscription(PoseStamped, self.leader_pos_topic, self.leader_position_callback, qos)

        now = datetime.now()
        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.imu_logger = IMULogger(self,
                                    f'/root/ardu_ws/src/swarm_control/imu_log/imu_data_follower{self.drone_id}_{timestamp}.csv',
                                    f"Follower {self.drone_id}")
        self.imu_subscription = self.create_subscription(
            Imu,
            f'/follower_drone_node_{drone_id}/imu',
            self.imu_callback,
            10)
                
    def create_takeoff_subscription(self):
        try:
            self.takeoff_subscription = self.create_subscription(Bool, '/leader_takeoff_complete', self.leader_takeoff_callback, 10)
        except Exception as e:
            self.get_logger().error(f'Failed to create takeoff subscription: {e}')
            self.takeoff_subscription = None

    def imu_callback(self, msg):
        self.imu_logger.log_imu_data(msg)

    def leader_takeoff_callback(self, msg):
        if msg.data:
            self.leader_takeoff_complete = True
            self.initiate_takeoff_sequence()

    def check_takeoff(self):
        if self.leader_takeoff_complete:
            if self.leader_pos_topic:
                self.get_logger().info('Intiating Follower mission')

    def leader_position_callback(self, msg: PoseStamped):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.get_clock().now().from_msg(msg.header.stamp)).nanoseconds / 1e9
        self.get_logger().info(f"Position update delay: {time_diff:.2f} seconds")
        self.leader_position = msg
        self.get_logger().info(
            f"Drone {self.drone_id}: Received leader position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}")

        if self.state != DroneState.FLYING:
            self.get_logger().warn(f"Drone {self.drone_id}: Not in FLYING state. Current state: {self.state}")
            return

        current_time = self.get_clock().now()
        if (current_time - self.last_command_time).nanoseconds / 1e9 < 1.0 / self.command_frequency:
            return

        self.last_command_time = current_time

        target_x = msg.pose.position.x + self.offset[0]
        target_y = msg.pose.position.y + self.offset[1]
        target_z = msg.pose.position.z + self.offset[2]

        self.get_logger().info(
            f"Drone {self.drone_id}: Calculated target position: {target_x:.2f}, {target_y:.2f}, {target_z:.2f}")

        try:
            self.mav_connection.mav.set_position_target_local_ned_send(
                0, self.mav_connection.target_system, self.mav_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
                target_x, target_y, -target_z, 0, 0, 0, 0, 0, 0, 0, 0)
            self.get_logger().info(f"Drone {self.drone_id}: Sent position target command")
        except Exception as e:
            self.get_logger().error(f"Drone {self.drone_id}: Error sending MAVLink message: {str(e)}")

        self.check_current_position()

    def check_current_position(self):
        msg = self.mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            self.get_logger().info(
                f"Drone {self.drone_id}: Current position: x={msg.x:.2f}, y={msg.y:.2f}, z={-msg.z:.2f}")
        else:
            self.get_logger().warn(f"Drone {self.drone_id}: Failed to get current position")

    def initiate_takeoff_sequence(self):
        if self.arm_drone_with_retry():
            self.get_logger().info(f'Takeoff successful follower: {self.drone_id} is following leader ...')
            self.on_armed_success()

    def on_armed_success(self):
        if self.leader_takeoff_complete:
            self.get_logger().info(f"Leader Takeoff complete initiating FOLLOWER {self.drone_id} Takeoff")
            self.takeoff_with_retry()
        else:
            self.get_logger().info('Waiting for leader to complete takeoff ...')


    def monitor_takeoff(self):
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
                    return
            time.sleep(1)
        self.get_logger().info(f"Drone{self.drone_id}: Mission Completed")

    def log_imu_data(self):
        msg = self.mav_connection.recv_match(type='RAW_IMU', blocking=False)
        if msg:
            self.get_logger().info(f"IMU DATA SAVING FOLLOWER {self.drone_id}")
            self.imu_logger.log_imu_data(msg)
        else:
            self.get_logger().warn(f"No IMU Data for Follower {self.drone_id}")

    def __del__(self):
        self.imu_logger.close()

def main(args=None):
    rclpy.init(args=args)

    follower = FollowerDrone(
        node_name='follower_drone_node',
        drone_id=1,
        mavlink_connection='udp:localhost:14561',
        leader_pos_topic='/leader_position',
        offset=[2.0, 2.0, 0.0],
        follow_distance=5.0
    )

    try:
        if follower.takeoff_subscription:
            follower.get_logger().info('Waiting for takeoff command...')
            while not follower.leader_takeoff_complete:
                rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()