#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, Mavlink


class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        self.declare_parameter('publish_rate', 1.0)  # Default to 1 Hz

        self.publisher = self.create_publisher(PoseStamped, '/drone/setpoint_position', 10)
        self.state_sub = self.create_subscription(
            State,
            '/mavlink/state',
            self.state_callback,
            10)
        self.mavlink_from_sub = self.create_subscription(
            Mavlink,
            '/mavlink/from',
            self.mavlink_from_callback,
            10)

        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_setpoint)

        self.drone_state = State()

    def state_callback(self, msg):
        self.drone_state = msg
        self.get_logger().info(f'Drone state: connected={msg.connected}, armed={msg.armed}, mode={msg.mode}')

    def mavlink_from_callback(self, msg):
        self.get_logger().info(f'Received MAVLink message, length: {len(msg.payload)}')

    def publish_setpoint(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 1.0  # Example setpoint
        msg.pose.position.y = 2.0
        msg.pose.position.z = 3.0
        self.publisher.publish(msg)
        self.get_logger().info('Published setpoint')


def main(args=None):
    rclpy.init(args=args)
    node = DroneControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()