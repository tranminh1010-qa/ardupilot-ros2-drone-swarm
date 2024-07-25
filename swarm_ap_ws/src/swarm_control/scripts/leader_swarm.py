#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class LeaderDrone:
    def __init__(self):
        rospy.init_node('leader_drone_node', anonymous=True)
        self.state_sub = rospy.Subscriber('/mavros1/state', State, self.state_cb)
        self.local_pos_pub = rospy.Publisher('/mavros1/setpoint_position/local', PoseStamped, queue_size=10)
        
        self.arming_client = rospy.ServiceProxy('/mavros1/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros1/set_mode', SetMode)
        
        self.current_state = State()
        self.target_position = PoseStamped()
        self.target_position.pose.position.x = 0
        self.target_position.pose.position.y = 0
        self.target_position.pose.position.z = 5  # Set initial altitude to 5m
        
        self.rate = rospy.Rate(20)
        self.offboard_mode_set = False

    def state_cb(self, state):
        self.current_state = state

    def arm_and_takeoff(self):
        while not self.current_state.armed:
            self.arming_client(True)
            self.rate.sleep()
        
        while not self.offboard_mode_set:
            self.set_mode_client(custom_mode="OFFBOARD")
            if self.current_state.mode == "OFFBOARD":
                self.offboard_mode_set = True
            self.rate.sleep()
        
        self.local_pos_pub.publish(self.target_position)
        rospy.loginfo("Leader drone taking off to 5m altitude")

    def send_position(self, x, y, z):
        self.target_position.pose.position.x = x
        self.target_position.pose.position.y = y
        self.target_position.pose.position.z = z
        self.local_pos_pub.publish(self.target_position)
        rospy.loginfo("Leader drone moving to position: {}, {}, {}".format(x, y, z))

if __name__ == '__main__':
    leader = LeaderDrone()
    leader.arm_and_takeoff()
    leader.send_position(5, 5, 5)
    rospy.spin()
