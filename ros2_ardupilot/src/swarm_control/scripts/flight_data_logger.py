#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
import csv
import os

class FlightDataLogger(Node):
    def __init__(self):
        super().__init__('flight_data_logger')
        log_dir = get_logger().get_log_directory()
        self.get_logger().info(f"Log directory: {log_dir}")
        self.log_file = os.path.join(log_dir, 'drone_flight_data.csv')
        self.get_logger().info(f"Log file path: {self.log_file}")
        self.create_subscription(PoseStamped, '/d1/mavros/local_position/pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/d2/mavros/local_position/pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/d3/mavros/local_position/pose', self.pose_callback, 10)
        self.fieldnames = ['drone_id', 'timestamp', 'x', 'y', 'z']
        with open(self.log_file, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
            writer.writeheader()

    def pose_callback(self, msg, drone_id):
        data = {
            'drone_id': drone_id,
            'timestamp': self.get_clock().now().to_msg().sec,
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        with open(self.log_file, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
            writer.writerow(data)

def main(args=None):
    rclpy.init(args=args)
    flight_data_logger = FlightDataLogger()
    rclpy.spin(flight_data_logger)
    flight_data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()