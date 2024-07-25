#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class FollowerDrone1:
    def __init__(self):
        rospy.init_node('follower_drone1_node', anonymous=True)
        self.state_sub = rospy.Subscriber('/mavros2/state', State, self.state_cb)
        self.leader_pos_sub = rospy.Subscriber('/leader/position', PoseStamped, self.leader_pos_cb)
        self.local_pos_pub = rospy.Publisher('/mavros2/setpoint_position/local', PoseStamped, queue_size=10)
        
        self.arming_client = rospy.ServiceProxy('/mavros2/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros2/set_mode', SetMode)
        
        self.current_state = State()
        self.target_position = PoseStamped()
        self.offset_x = 1.0
        self.offset_y = 0.0
        self.offset_z = 0.0

        self.rate = rospy.Rate(20)
        self.offboard_mode_set = False

    def state_cb(self, state):
        self.current_state = state

    def leader_pos_cb(self, leader_pos):
        self.target_position.pose.position.x = leader_pos.pose.position.x + self.offset_x
        self.target_position.pose.position.y = leader_pos.pose.position.y + self.offset_y
        self.target_position.pose.position.z = leader_pos.pose.position.z + self.offset_z
        self.local_pos_pub.publish(self.target_position)

    def arm_and_takeoff(self):
        while not self.current_state.armed:
            self.arming_client(True)
            self.rate.sleep()
        
        while not self.offboard_mode_set:
            self.set_mode_client(custom_mode="OFFBOARD")
            if self.current_state.mode == "OFFBOARD":
                self.offboard_mode_set = True
            self.rate.sleep()
        
        rospy.loginfo("Follower drone 1 taking off with offset")

if __name__ == '__main__':
    follower1 = FollowerDrone1()
    follower1.arm_and_takeoff()
    rospy.spin()
