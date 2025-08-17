#!/usr/bin/env python3

import json

import rclpy
import time
from ardupilot_msgs.msg import Status, GlobalPosition  # Use DDS messages
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from pyproj import Transformer

from points_distributor import generate_circular_waypoints
from scripts.drone_state import DroneState, FlightMode

from scripts.time_out_retry import TimeoutRetry


def mode_success_check_factory(mode: FlightMode):
    """Factory that creates success check for mode switching"""
    def check(result):
        return result and (result.status or result.curr_mode == mode.value)
    return check

class BaseDrone(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Initialize home position variables
        self.initialize_vars()
        self.get_parameter_values()
        self.sysid = self.drone_id
        self.handle_waypoints()
        self.state = DroneState.INITIALIZING

        self.get_logger().info(f"Initializing drone {self.drone_id} with DDS interface")
        self.get_logger().info(f"Using system ID: {self.sysid}")

        # Service clients for DDS - using drone-specific topics
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self.get_logger().info(f"Arm service topic: {self._arm_topic}")
        self.arm_client = self.create_client(ArmMotors, self._arm_topic)
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self.get_logger().info(f"Mode service topic: {self._mode_topic}")
        self.mode_client = self.create_client(ModeSwitch, self._mode_topic)
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')
        self.takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Takeoff service not available, waiting...')

        self._setup_subscriptions()

        # DDS subscribers and service clients
        self._global_pos_pub = self.create_publisher(GlobalPosition, self._global_pos_topic, 1)
        self.get_logger().info(f"QoS: {self.status_subscription.qos_profile}")
        self.get_logger().info(f"Subscribed to: {self.status_subscription.topic_name}")
        self.get_logger().info(f"Subscribed to: {self.subscription_pose.topic_name}")

    def initialize_vars(self):
        self.waypoints = []
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.transformer = None
        self.current_position = None
        self.target_position = None
        self.armed = False

    def get_parameter_values(self):
        # Declare all parameters first
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('assigned_waypoints', '')
        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self.declare_parameter("pose_topic", "/ap/pose/filtered")
        self.declare_parameter("global_position_topic", "/ap/cmd_gps_pose")
        self.declare_parameter("takeoff_topic", "/ap/experimental/takeoff")
        
        # Then get parameter values
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self._global_pos_topic = self.get_parameter("global_position_topic").get_parameter_value().string_value
        self._pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value

    def _setup_subscriptions(self):
        """Set up subscriptions """
        ardupilot_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.status_callback = self._status_callback_impl
        self.pose_callback = self._pose_callback_impl

        # ADD: GPS origin callback with SYSID filter
        self.gps_origin_callback = self._gps_origin_callback_impl

        # Create subscriptions
        self.status_subscription = self.create_subscription(
            Status, '/ap/status', self.status_callback, ardupilot_qos)
        self.subscription_pose = self.create_subscription(
            PoseStamped, '/ap/pose/filtered', self.pose_callback, ardupilot_qos)
        self.gps_origin_subscription = self.create_subscription(
            GeoPointStamped,
            '/ap/gps_global_origin/filtered',
            self.gps_origin_callback,
            ardupilot_qos
        )

    @TimeoutRetry(timeout_sec=30.0, retry_interval=1.0,
                  success_check_factory=lambda: lambda result: result and result.result)
    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    @TimeoutRetry(timeout_sec=20.0, retry_interval=2.0,
                  success_check_factory=lambda: lambda pos: pos is not None)
    def wait_for_position_estimate(self):
        """Wait for position with automatic retry"""
        rclpy.spin_once(self)
        return self.current_position

    def _status_callback_impl(self, msg):
        """Handle ArduPilot status messages - implementation"""
        self.armed = msg.armed
        self.current_mode = msg.mode
        self.get_logger().info(f"Status callback: {msg}")
        if self.state == DroneState.CONNECTING:
            self.state = DroneState.CONNECTED
            self.get_logger().info(f"DDS connection established for drone {self.drone_id}")

    def _pose_callback_impl(self, msg):
        """Process a GeoPose message - implementation"""
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

    @TimeoutRetry(timeout_sec=10.0, retry_interval=1.0,
                  success_check_factory=mode_success_check_factory)
    def switch_mode(self, mode: FlightMode):
        req = ModeSwitch.Request()
        req.mode = mode.value
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff(self, altitude=10.0):
        req = Takeoff.Request()
        req.alt = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _gps_origin_callback_impl(self, msg):
        """Capture the GPS origin (home position) when it's set - implementation"""
        lat = msg.position.latitude
        lon = msg.position.longitude
        if self.home_lat is None or self.home_lon is None or self.home_lat != lat or self.home_lon != lon:
            self.home_lon = lon
            self.home_lat = lat
            self.home_alt = msg.position.altitude

            # Initialize the coordinate transformer
            self.setup_coordinate_transformer()

            self.get_logger().info(
                f"Home position set: lat={self.home_lat:.8f}, "
                f"lon={self.home_lon:.8f}, alt={self.home_alt:.2f}"
            )

    def setup_coordinate_transformer(self):
        """Set up pyproj transformer for local to global conversions"""
        # Create a local tangent plane projection centered at home
        # Using Transverse Mercator projection
        proj_string = (
            f"+proj=tmerc +lat_0={self.home_lat} +lon_0={self.home_lon} "
            f"+k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
        )
        self.transformer = Transformer.from_crs(
            proj_string,  # Local coordinate system
            "EPSG:4326",  # WGS84
            always_xy=True
        )

        self.get_logger().info("Coordinate transformer initialized")

    def xy_to_latlon(self, x, y):
        """Convert local NED coordinates to lat/lon using pyproj
        Args:
            x: North position in meters (positive north)
            y: East position in meters (positive east)
        """
        # pyproj expects x=east, y=north, so swap the inputs
        lon, lat = self.transformer.transform(y, x)
        return lat, lon

    def start_mission(self):
        """Start waypoint mission using DDS"""
        self.get_logger().info(f"Starting mission for drone {self.drone_id}")
        self.state = DroneState.FLYING

        # Simple waypoint following
        for i, wp in enumerate(self.waypoints):
            lat, lon = self.xy_to_latlon(wp[0], wp[1])
            self.send_position_command_dds(lat, lon, wp[2])
            rclpy.spin_once(self)
            self.get_logger().info(f"Drone {self.drone_id} heading to waypoint {i + 1}: {lat, lon, wp[2]}")
            time.sleep(10)  # Wait between waypoints

        self.get_logger().info("Mission complete, returning to launch")
        self.state = DroneState.RETURNING
        if not self.switch_mode(FlightMode.RTL):
            self.get_logger().error("Unable to switch to RTL mode, sending home position")
            self.send_position_command_dds(self.home_lat, self.home_lon, self.home_alt)


    def send_position_command_dds(self, lat, lon, alt):
        msg = GlobalPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type_mask = 0b0000111111111000  # Use position only, ignore velocity/acceleration
        msg.header.frame_id = "map"
        msg.coordinate_frame = 6  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

        # Convert to integers as expected by INT frames
        msg.latitude = float(lat * 1e7)  # Convert to float32 (degrees * 1e7)
        msg.longitude = float(lon * 1e7)  # Convert to float32 (degrees * 1e7)
        msg.altitude = float(alt * 1000)  # Convert to millimeters (relative to home)

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
    if not node.arm():
        raise RuntimeError("Unable to arm")

    # Wait for a position estimate before switching to GUIDED
    if not node.wait_for_position_estimate():
        raise RuntimeError("No position estimate available")

    if not node.switch_mode(FlightMode.GUIDED):
        raise RuntimeError("Unable to switch to GUIDED mode")

    # Takeoff
    takeoff_result = node.takeoff(50.0)
    if not takeoff_result.status:
        raise RuntimeError("Takeoff failed")
    node.get_logger().info(f"Drone take off status: {takeoff_result.status}")
    # Wait for takeoff completion
    time.sleep(15)

    node.get_logger().info(f"Drone {node.drone_id} initialized with DDS")
    node.get_logger().info(f"Drone {node.drone_id} started mission")
    node.start_mission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()