#!/usr/bin/env python

import rclpy
import csv
from sensor_msgs.msg import Imu
import numpy as np

class IMULogger:
    def __init__(self, node, file_path, drone_label):
        self.node = node
        self.file_path = file_path
        self.csv_file = open(file_path, mode='a')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([drone_label])
        self.csv_writer.writerow([
            "Time", "Magnetic Field X", "Magnetic Field Y", "Magnetic Field Z",
            "Angular Velocity X", "Angular Velocity Y", "Angular Velocity Z",
            "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z",
            "Target Position X", "Target Position Y", "Target Position Z",
            "Actual Position X", "Actual Position Y", "Actual Position Z",
            "Position Deviation (m)"
        ])

    def log_imu_data(self, msg, current_pos, target_pos=None):
        if target_pos is None:
            target_pos = current_pos

        deviation = np.linalg.norm(np.array(current_pos) - np.array(target_pos))

        self.csv_writer.writerow([
            self.node.get_clock().now().to_msg().sec,
            msg.xmag, msg.ymag, msg.zmag,
            msg.xgyro, msg.ygyro, msg.zgyro,
            msg.xacc, msg.yacc, msg.zacc,
            current_pos[0], current_pos[1], current_pos[2],
            target_pos[0], target_pos[1], target_pos[2],
            deviation
        ])

    def close(self):
        self.csv_file.close()