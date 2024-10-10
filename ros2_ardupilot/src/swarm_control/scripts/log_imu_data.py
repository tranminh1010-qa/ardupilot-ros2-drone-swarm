#!/usr/bin/env python

import rclpy
import csv
from sensor_msgs.msg import Imu

class IMULogger:
    def __init__(self, node, file_path, drone_label):
        self.node = node
        self.file_path = file_path
        self.csv_file = open(file_path, mode='a')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([drone_label])
        self.csv_writer.writerow([
            "Time", "Orientation X", "Orientation Y", "Orientation Z",
            "Angular Velocity X", "Angular Velocity Y", "Angular Velocity Z",
            "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"
        ])

    def log_imu_data(self, msg):
        self.csv_writer.writerow([
            self.node.get_clock().now().to_msg().sec,
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ])

    def close(self):
        self.csv_file.close()