#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time

class LeaderDrone:
    def __init__(self):
        rospy.init_node('leader_drone_node', anonymous=True)
        self.state_sub1 = rospy.Subscriber('/d1/mavros/state', State, self.state_cb1)
        self.local_pos_pub1 = rospy.Publisher('/d1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        self.arming_client1 = rospy.ServiceProxy('/d1/mavros/cmd/arming', CommandBool)
        self.set_mode_client1 = rospy.ServiceProxy('/d1/mavros/set_mode', SetMode)
        self.set_takeoff_client1 = rospy.ServiceProxy('/d1/mavros/cmd/takeoff', CommandTOL)
        
        self.state_sub2 = rospy.Subscriber('/d2/mavros/state', State, self.state_cb2)
        self.local_pos_pub2 = rospy.Publisher('/d2/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        self.arming_client2 = rospy.ServiceProxy('/d2/mavros/cmd/arming', CommandBool)
        self.set_mode_client2 = rospy.ServiceProxy('/d2/mavros/set_mode', SetMode)
        self.set_takeoff_client2 = rospy.ServiceProxy('/d2/mavros/cmd/takeoff', CommandTOL)
        
        self.state_sub3 = rospy.Subscriber('/d3/mavros/state', State, self.state_cb3)
        self.local_pos_pub3 = rospy.Publisher('/d3/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        self.arming_client3 = rospy.ServiceProxy('/d3/mavros/cmd/arming', CommandBool)
        self.set_mode_client3 = rospy.ServiceProxy('/d3/mavros/set_mode', SetMode)
        self.set_takeoff_client3 = rospy.ServiceProxy('/d3/mavros/cmd/takeoff', CommandTOL)
        
        self.current_state1 = State()
        self.current_state2 = State()
        self.current_state3 = State()

        self.current_pose1 = PoseStamped()

        self.target_position1 = PoseStamped()
        self.target_position1.pose.position.x = 0
        self.target_position1.pose.position.y = 0
        self.target_position1.pose.position.z = 5  # Set initial altitude to 5m

        self.target_position2 = PoseStamped()
        self.target_position2.pose.position.x = self.target_position1.pose.position.x
        self.target_position2.pose.position.y = self.target_position1.pose.position.y
        self.target_position2.pose.position.z = self.target_position1.pose.position.z  # Set initial altitude to 5m

        self.target_position3 = PoseStamped()
        self.target_position3.pose.position.x = self.target_position1.pose.position.x
        self.target_position3.pose.position.y = self.target_position1.pose.position.y
        self.target_position3.pose.position.z = self.target_position1.pose.position.z # Set initial altitude to 5m

        self.takeoff_complete = False
        self.rate = rospy.Rate(20)
        self.guided_mode_set1 = False
        self.guided_mode_set2 = False
        self.guided_mode_set3 = False

        self.pose_sub1 = rospy.Subscriber('/d1/mavros/local_position/pose', PoseStamped, self.pose_cb1)
        self.lx = 0.0
        self.ly = 0.0
        self.lz = 0.0

    def state_cb1(self, state):
        self.current_state1 = state

    def state_cb2(self, state):
        self.current_state2 = state

    def state_cb3(self, state):
        self.current_state3 = state

    def arm_and_takeoff(self):
        while not (self.guided_mode_set1 and self.guided_mode_set2 and self.guided_mode_set3):
            self.set_mode_client1(custom_mode="GUIDED")
            self.set_mode_client2(custom_mode="GUIDED")
            self.set_mode_client3(custom_mode="GUIDED")
            if self.current_state1.mode == "GUIDED":
                self.guided_mode_set1 = True
            if self.current_state2.mode == "GUIDED":
                self.guided_mode_set2 = True
            if self.current_state3.mode == "GUIDED":
                self.guided_mode_set3 = True
            self.rate.sleep()

        while not (self.current_state1.armed and self.current_state2.armed and self.current_state3.armed):
            self.arming_client1(True)
            self.arming_client2(True)
            self.arming_client3(True)
            self.rate.sleep()

        self.set_takeoff_client1(altitude=5.0)
        self.set_takeoff_client2(altitude=5.0)
        self.set_takeoff_client3(altitude=5.0)
        
        rospy.loginfo("Leader drone taking off to 5m altitude")

        # Wait until the drones reach the desired altitude
        while self.current_pose1.pose.position.z < 4.9:  # Adding a small margin to ensure stability at the altitude
            rospy.loginfo("Waiting for the drone to reach takeoff altitude...")
            self.rate.sleep()

        rospy.loginfo("Drones have reached takeoff altitude")
        self.takeoff_complete = True

    def pose_cb1(self, pose):
        # Only update positions after takeoff is complete
        self.current_pose1 = pose
        self.lx = pose.pose.position.x
        self.ly = pose.pose.position.y
        self.lz = pose.pose.position.z

        if self.takeoff_complete:
            self.target_position2.pose.position.x = self.lx
            self.target_position2.pose.position.y = self.ly
            self.target_position2.pose.position.z = self.lz
            self.local_pos_pub2.publish(self.target_position2)
            
            self.target_position3.pose.position.x = self.lx
            self.target_position3.pose.position.y = self.ly
            self.target_position3.pose.position.z = self.lz
            self.local_pos_pub3.publish(self.target_position3)

    def send_position1(self, x, y, z):
        self.target_position1.pose.position.x = x
        self.target_position1.pose.position.y = y
        self.target_position1.pose.position.z = z
        self.local_pos_pub1.publish(self.target_position1)

        self.target_position2.pose.position.x = self.lx
        self.target_position2.pose.position.y = self.ly
        self.target_position2.pose.position.z = self.lz
        self.local_pos_pub2.publish(self.target_position2)
            
        self.target_position3.pose.position.x = self.lx
        self.target_position3.pose.position.y = self.ly
        self.target_position3.pose.position.z = self.lz
        self.local_pos_pub3.publish(self.target_position3)

        rospy.loginfo("Leader drone moving to position: {}, {}, {}".format(x, y, z))

if __name__ == '__main__':
    leader = LeaderDrone()
    leader.arm_and_takeoff()
    time.sleep(10)
    leader.send_position1(4, 1, 5)
    time.sleep(10)
    leader.send_position1(1, 8, 5)
    time.sleep(10)
    leader.send_position1(8, 6, 7)
    time.sleep(10)
    leader.send_position1(0, 0, 5)

    rospy.spin()
