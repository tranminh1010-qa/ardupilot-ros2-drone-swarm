#!/usr/bin/env python

import rospy
import csv
from sensor_msgs.msg import Imu

class IMULogger:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('imu_logger', anonymous=True)

        # Subscribe to the IMU topic
        rospy.Subscriber("/drone/mavros/imu/data", Imu, self.imu_callback)

        # Open a CSV file to write the IMU data
        self.csv_file = open('/path/to/save/imu_data.csv', mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write the header row to the CSV file
        self.csv_writer.writerow(["Time", "Orientation X", "Orientation Y", "Orientation Z", "Orientation W",
                                  "Angular Velocity X", "Angular Velocity Y", "Angular Velocity Z",
                                  "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"])

    def imu_callback(self, data):
        # Extract the IMU data
        orientation = data.orientation
        angular_velocity = data.angular_velocity
        linear_acceleration = data.linear_acceleration

        # Log the data to the CSV file
        self.csv_writer.writerow([rospy.get_time(), orientation.x, orientation.y, orientation.z, orientation.w,
                                  angular_velocity.x, angular_velocity.y, angular_velocity.z,
                                  linear_acceleration.x, linear_acceleration.y, linear_acceleration.z])

    def run(self):
        rospy.spin()

    def __del__(self):
        # Close the CSV file when done
        self.csv_file.close()

if __name__ == '__main__':
    try:
        imu_logger = IMULogger()
        imu_logger.run()
    except rospy.ROSInterruptException:
        pass
