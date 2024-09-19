#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, Imu, FluidPressure
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf
import numpy as np

class CustomStateEstimator:
    def __init__(self):
        rospy.init_node('custom_state_estimator', anonymous=True)
        
        self.gps_data = None
        self.imu_data = None
        self.baro_data = None

        rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/mavros/imu/static_pressure', FluidPressure, self.baro_callback)
        
        self.pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.twist_pub = rospy.Publisher('/mavros/vision_speed/speed_twist', TwistStamped, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_estimated_state)
        
    def gps_callback(self, data):
        self.gps_data = data

    def imu_callback(self, data):
        self.imu_data = data

    def baro_callback(self, data):
        self.baro_data = data

    def custom_state_estimator(self):
        """
        Custom state estimator logic goes here.
        Use self.gps_data, self.imu_data, and self.baro_data to compute position and velocity.
        """
        position = np.array([0.0, 0.0, 0.0])
        velocity = np.array([0.0, 0.0, 0.0])
        
        
        if self.gps_data and self.imu_data and self.baro_data:

            latitude = self.gps_data.latitude
            longitude = self.gps_data.longitude
            altitude = self.baro_data.fluid_pressure  # Assuming barometric pressure is converted to altitude
            position = np.array([latitude, longitude, altitude])
            velocity = np.array([0.0, 0.0, 0.0]) 
            
        return position, velocity

    def publish_estimated_state(self, event):
        position, velocity = self.custom_state_estimator()
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        # Create TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "map"
        twist_msg.twist.linear.x = velocity[0]
        twist_msg.twist.linear.y = velocity[1]
        twist_msg.twist.linear.z = velocity[2]
        
        self.pose_pub.publish(pose_msg)
        self.twist_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        estimator = CustomStateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

