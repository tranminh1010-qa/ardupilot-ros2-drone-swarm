#!/usr/bin/env python

import rclpy
import csv
from sensor_msgs.msg import Imu
import numpy as np
import os
import atexit
import time


class IMULogger:
    def __init__(self, node, file_path, drone_label, flush_interval=1.0):
        """
        Initialize the IMU logger with proper file handling and buffering.

        Args:
            node: The ROS node instance
            file_path: Path to the CSV file
            drone_label: Label to identify the drone in the logs
            flush_interval: How often to flush to disk (in seconds)
        """
        self.node = node
        self.file_path = file_path
        self.last_flush_time = time.time()
        self.flush_interval = flush_interval
        self.buffer_count = 0
        self.MAX_BUFFER_SIZE = 50  # Maximum number of entries before forced flush

        # Open file with a larger buffer size for better performance
        self.csv_file = open(file_path, mode='a', buffering=8192)  # 8KB buffer
        self.csv_writer = csv.writer(self.csv_file)

        # Write headers
        self.csv_writer.writerow([drone_label])
        self.csv_writer.writerow([
            "Time", "Magnetic Field X", "Magnetic Field Y", "Magnetic Field Z",
            "Angular Velocity X", "Angular Velocity Y", "Angular Velocity Z",
            "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z",
            "Target Position X", "Target Position Y", "Target Position Z",
            "Actual Position X", "Actual Position Y", "Actual Position Z",
            "Position Deviation (m)"
        ])

        # Initial flush of headers
        self._flush_to_disk()

        # Register cleanup on exit
        atexit.register(self.close)

    def _should_flush(self):
        """Determine if we should flush based on time or buffer size."""
        current_time = time.time()
        time_to_flush = (current_time - self.last_flush_time) >= self.flush_interval
        buffer_full = self.buffer_count >= self.MAX_BUFFER_SIZE
        return time_to_flush or buffer_full

    def _flush_to_disk(self):
        """Perform the actual flush operation."""
        try:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.last_flush_time = time.time()
            self.buffer_count = 0
        except Exception as e:
            if hasattr(self.node, 'get_logger'):
                self.node.get_logger().error(f"Error flushing to disk: {str(e)}")
            else:
                print(f"Error flushing to disk: {str(e)}")

    def log_imu_data(self, msg, current_pos, target_pos=None):
        """
        Log IMU data with buffered disk writing.

        Args:
            msg: IMU message containing sensor data
            current_pos: Current position tuple (x, y, z)
            target_pos: Target position tuple (x, y, z), defaults to current_pos if None
        """
        try:
            if target_pos is None:
                target_pos = current_pos

            deviation = np.linalg.norm(np.array(current_pos) - np.array(target_pos))

            # Get microsecond precision timestamp
            timestamp = self.node.get_clock().now().to_msg()
            time_sec = float(timestamp.sec) + float(timestamp.nanosec) / 1e9

            self.csv_writer.writerow([
                f"{time_sec:.6f}",  # Include microseconds in timestamp
                msg.xmag, msg.ymag, msg.zmag,
                msg.xgyro, msg.ygyro, msg.zgyro,
                msg.xacc, msg.yacc, msg.zacc,
                target_pos[0], target_pos[1], target_pos[2],
                current_pos[0], current_pos[1], current_pos[2],
                deviation
            ])

            self.buffer_count += 1

            # Check if we should flush based on time or buffer size
            if self._should_flush():
                self._flush_to_disk()

        except Exception as e:
            if hasattr(self.node, 'get_logger'):
                self.node.get_logger().error(f"Error logging IMU data: {str(e)}")
            else:
                print(f"Error logging IMU data: {str(e)}")

    def close(self):
        """
        Properly close the file handle.
        """
        try:
            if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
                self._flush_to_disk()
                self.csv_file.close()
        except Exception as e:
            if hasattr(self.node, 'get_logger'):
                self.node.get_logger().error(f"Error closing IMU logger: {str(e)}")
            else:
                print(f"Error closing IMU logger: {str(e)}")

    def __del__(self):
        """
        Ensure file is closed when object is deleted.
        """
        self.close()