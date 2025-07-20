#!/usr/bin/env python3

import enum
import json

import rclpy
import time
from ardupilot_msgs.msg import Status, GlobalPosition  # Use DDS messages
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from points_distributor import generate_circular_waypoints


class DroneState(enum.Enum):
    INITIALIZING = 0
    CONNECTING = 1
    CONNECTED = 2
    ARMING = 3
    GUIDED = 4
    TAKING_OFF = 5
    FLYING = 6
    LANDING = 7
    ERROR = 8


class BaseDrone(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Declare all parameters first
        self.waypoints = []
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('assigned_waypoints', '')
        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self.declare_parameter("pose_topic", "/ap/pose/filtered")
        self.declare_parameter("global_position_topic", "/ap/cmd_gps_pose")

        # Then get parameter values
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self._global_pos_topic = self.get_parameter("global_position_topic").get_parameter_value().string_value
        self._pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value

        # Handle Initial Values
        self.handle_waypoints()

        self.state = DroneState.INITIALIZING
        self._cur_geopose = GeoPoseStamped()
        self.current_position = None
        self.target_position = None
        self.armed = False

        self.get_logger().info(f"Initializing drone {self.drone_id} with DDS interface")

        # Service clients for DDS
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self.arm_client = self.create_client(ArmMotors, self._arm_topic)
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self.mode_client = self.create_client(ModeSwitch, self._mode_topic)
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')
        self.takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Takeoff service not available, waiting...')

        # Create ArduPilot-compatible QoS profile
        ardupilot_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # DDS subscribers and service clients
        self._global_pos_pub = self.create_publisher(GlobalPosition, self._global_pos_topic, 1)
        self.status_subscription = self.create_subscription(Status, '/ap/status', self.status_callback, ardupilot_qos)
        self.subscription_pose = self.create_subscription(PoseStamped, self._pose_topic, self.pose_callback, ardupilot_qos)

        self.get_logger().info(f"QoS: {self.status_subscription.qos_profile}")
        self.get_logger().info(f"Subscribed to: {self.status_subscription.topic_name}")
        self.get_logger().info(f"Subscribed to: {self.subscription_pose.topic_name}")

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        """Try to arm. Returns true on success, or false if arming fails or times out."""
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm().result
            time.sleep(1)
        return armed

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: DroneState, timeout: rclpy.duration.Duration):
        """Try to switch mode. Returns true on success or false if mode switch fails or times out."""
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode and self.get_clock().now() - start < timeout:
            result = self.switch_mode(desired_mode)
            # Handle a successful switch or the case that the vehicle is already in expected mode
            is_in_desired_mode = result.status or result.curr_mode == desired_mode
            time.sleep(1)

        return is_in_desired_mode

    def wait_for_position_estimate(self, timeout: rclpy.duration.Duration):
        """Wait for EKF to have a good position estimate before switching to GUIDED mode."""
        start = self.get_clock().now()

        while self.get_clock().now() - start < timeout:
            # Process callbacks to allow pose_callback to execute
            rclpy.spin_once(self)
            # Check if we have a valid position from the pose callback
            if self.current_position is not None:
                self.get_logger().info("Position estimate available, ready for GUIDED mode")
                return True

            self.get_logger().info("Waiting for position estimate...")
            time.sleep(2)  # Check every 2 seconds

        self.get_logger().error("Timeout waiting for position estimate")
        return False

    def status_callback(self, msg):
        """Handle ArduPilot status messages"""
        self.armed = msg.armed
        self.get_logger().info(f"Status callback: {msg}")
        if self.state == DroneState.CONNECTING:
            self.state = DroneState.CONNECTED
            self.get_logger().info(f"DDS connection established for drone {self.drone_id}")

    def pose_callback(self, msg):
        """Process a GeoPose message."""
        self.current_position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

    def arm_response_callback(self, future):
        """Handle arm service response"""
        try:
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response.result:
                self.get_logger().info(f"Drone {self.drone_id} armed successfully via DDS")
                self.set_guided_mode_dds()
            else:
                self.get_logger().error(f"Failed to arm drone {self.drone_id}")
        except Exception as e:
            self.get_logger().error(f'Arm service call failed: {e}')

    def switch_mode(self, mode: DroneState):
        req = ModeSwitch.Request()
        mode_map = {
            DroneState.GUIDED: 4,  # GUIDED mode for copter
            DroneState.TAKING_OFF: 4,  # Use GUIDED for takeoff
        }
        req.mode = mode_map.get(mode, 4)  # Default to GUIDED
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff(self, altitude=10.0):
        req = Takeoff.Request()
        req.alt = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def start_mission(self):
        """Start waypoint mission using DDS"""
        self.get_logger().info(f"Starting mission for drone {self.drone_id}")
        self.state = DroneState.FLYING

        # Simple waypoint following
        for i, wp in enumerate(self.waypoints):
            self.send_position_command_dds(wp[0], wp[1], wp[2])
            self.get_logger().info(f"Drone {self.drone_id} heading to waypoint {i + 1}: {wp}")
            time.sleep(10)  # Wait between waypoints

    def send_position_command_dds(self, lat, lon, alt):
        msg = GlobalPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type_mask = 0
        msg.header.frame_id = "map"
        msg.coordinate_frame = 5  # FRAME_GLOBAL_INT
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)

        self._global_pos_pub.publish(msg)

    def handle_waypoints(self):
        assigned_waypoints_param = self.get_parameter('assigned_waypoints').get_parameter_value().string_value
        if not assigned_waypoints_param or assigned_waypoints_param == '':
            self.waypoints = generate_circular_waypoints()
            self.get_logger().info(f"Generated default circular waypoints for drone {self.drone_id}")
        else:
            try:
                self.waypoints = json.loads(assigned_waypoints_param)
                self.get_logger().info(
                    f"Using assigned waypoints for drone {self.drone_id}: {len(self.waypoints)} points")
            except Exception as e:
                self.get_logger().error(f"Error parsing waypoints: {e}, using default")
                self.waypoints = generate_circular_waypoints()


def main(args=None):
    rclpy.init(args=args)
    # Start the node with default parameter
    node = BaseDrone('base_drone_dds')

    # Block till armed, which will wait for EKF3 to initialize
    if not node.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
        raise RuntimeError("Unable to arm")

    # Wait for a position estimate before switching to GUIDED
    if not node.wait_for_position_estimate(rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("No position estimate available")

    if not node.switch_mode_with_timeout(DroneState.GUIDED, rclpy.duration.Duration(seconds=10)):
        raise RuntimeError("Unable to switch to GUIDED mode")

        # Takeoff
    takeoff_result = node.takeoff(10.0)
    if not takeoff_result.status:
        raise RuntimeError("Takeoff failed")
    # Wait for takeoff completion
    time.sleep(15)

    node.get_logger().info(f"Drone {node.drone_id} initialized with DDS")
    node.start_mission()
    node.get_logger().info(f"Drone {node.drone_id} started mission")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()