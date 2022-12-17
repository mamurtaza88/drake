#! /usr/bin/env python3

import rospy
from rospy import Time 
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from wrist_tracking.msg import lcmt_iiwa_ee_pose

import cv2
import sys
import numpy as np
#LCM
import lcm
from exlcm import example_t

wrist_num = 7
bool_barrier = 0.0

# Create LCM object
Main_channel = str("Hand_Position")
lc = lcm.LCM() # Creare a lcm object
msg = example_t()  # communication format


def receiver(data):

    # Update = true means that a new twist has been calculated and is ready to be published
    msg.pnt = (data.ee_pose[0], data.ee_pose[1], data.ee_pose[2])  # position values

    # cnt_cos = cnt_cos + 0.1
    # msg.pnt = (0.75, 0.0, 0.45 + 0.15*np.cos(cnt_cos))  # position values
    msg.rpy = (data.ee_rpy[0], data.ee_rpy[1], data.ee_rpy[2])
    msg.motion_time = 1.0 # double variable (unit: second), can be changed
    lc.publish(Main_channel, msg.encode()) # LCM publisher

    # rospy.loginfo(pnt_found)
    # pub_found.publish(pnt_found)
    


    
def listener():
    # Listener is the initialization function and subscribes to the point value of the "find_ball" node and creates the publisher of twists

    rospy.init_node('consensus2', anonymous=True)
    rospy.Subscriber("/lcm_to_ros/lcmt_iiwa_ee_pose", lcmt_iiwa_ee_pose, receiver)
    # global pub
    # pub = rospy.Publisher('/ridgeback2/cmd_vel', Twist, queue_size=2)


    

if __name__ == '__main__':
    listener()

rate = rospy.Rate(10)


while not rospy.is_shutdown():
    # This is the infinite loop that keep the program running

    rate.sleep()