#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, Mavlink
from std_msgs.msg import Header


class MavlinkNode(Node):
    def __init__(self):
        super().__init__('mavlink_node')
        self.mavlink_connection = mavutil.mavlink_connection('udpout:localhost:14551', source_system=1)
        self.mavlink_listener = mavutil.mavlink_connection('udpin:localhost:14550', source_system=1)
        self.get_logger().info(f"Initialized MAVLink connection to {self.mavlink_connection.address}")
        self.get_logger().info(f"Initialized MAVLink listener on {self.mavlink_listener.address}")

        # Publishers
        self.state_pub = self.create_publisher(State, '/mavlink/state', 10)
        self.mavlink_from_pub = self.create_publisher(Mavlink, '/mavlink/from', 10)
        self.mavlink_to_pub = self.create_publisher(Mavlink, '/mavlink/to', 10)

        # Subscribers
        self.setpoint_sub = self.create_subscription(
            PoseStamped,
            '/drone/setpoint_position',
            self.setpoint_callback,
            10)

        # Timers
        self.create_timer(0.1, self.check_mavlink_messages)
        self.create_timer(1.0, self.publish_heartbeat)

    def setpoint_callback(self, msg):
        try:
            # Convert ROS PoseStamped to MAVLink SET_POSITION_TARGET_LOCAL_NED
            mavlink_msg = self.mavlink_connection.mav.set_position_target_local_ned_encode(
                0,  # time_boot_ms
                1,  # target system
                0,  # target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,  # type_mask (only positions enabled)
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0)

            # Create and publish the ROS Mavlink message
            ros_mavlink_msg = Mavlink()
            ros_mavlink_msg.header = Header()
            ros_mavlink_msg.header.stamp = self.get_clock().now().to_msg()
            ros_mavlink_msg.msgid = mavlink_msg.get_msgId()
            ros_mavlink_msg.payload64 = [int.from_bytes(mavlink_msg.get_msgbuf(), byteorder='little')]
            self.mavlink_to_pub.publish(ros_mavlink_msg)

            # Send the MAVLink message
            self.mavlink_connection.mav.send(mavlink_msg)
            self.get_logger().info(
                f'Sent setpoint to MAVLink: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')
        except Exception as e:
            self.get_logger().error(f'Error in setpoint_callback: {str(e)}')

    def check_mavlink_messages(self):
        try:
            msg = self.mavlink_listener.recv_match(blocking=False)
            if msg:
                if msg.get_type() == 'HEARTBEAT':
                    state_msg = State()
                    state_msg.connected = True
                    state_msg.armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    state_msg.mode = mavutil.mode_string_v10(msg)
                    self.state_pub.publish(state_msg)
                    self.get_logger().info(
                        f'Published state: connected={state_msg.connected}, armed={state_msg.armed}, mode={state_msg.mode}')

                # Publish all received MAVLink messages to ROS
                ros_mavlink_msg = Mavlink()
                ros_mavlink_msg.header = Header()
                ros_mavlink_msg.header.stamp = self.get_clock().now().to_msg()
                ros_mavlink_msg.msgid = msg.get_msgId()
                ros_mavlink_msg.payload64 = [int.from_bytes(msg.get_msgbuf(), byteorder='little')]
                self.mavlink_from_pub.publish(ros_mavlink_msg)

                self.get_logger().info(f'Received and published MAVLink message: {msg.get_type()}')
            else:
                self.get_logger().debug('No MAVLink message received')
        except Exception as e:
            self.get_logger().error(f'Error in check_mavlink_messages: {str(e)}')

    def publish_heartbeat(self):
        try:
            self.mavlink_connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0)
            self.get_logger().info('Sent heartbeat to MAVLink')
        except Exception as e:
            self.get_logger().error(f'Error in publish_heartbeat: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()