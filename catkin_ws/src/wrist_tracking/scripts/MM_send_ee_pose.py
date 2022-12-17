#! /usr/bin/env python3

import rospy
from rospy import Time 
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R


import cv2
import sys
# import pyzed.sl as sl
# import ogl_viewer.viewer as gl
# import cv_viewer.tracking_viewer as cv_viewer
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


    r = R.from_quat([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    xyz = r.as_euler('xyz', degrees=False)
    r1 = R.from_euler('xyz',[0,1.57, -1.57 + xyz[2]])
    # print(r1.as_euler('zyx', degrees=True))
    # Update = true means that a new twist has been calculated and is ready to be published
    msg.pnt = (data.position.x, data.position.y, data.position.z)  # position values
    # msg.pnt = (0.65, 0.0, 0.44)  # position values
    msg.rpy = r1.as_euler('zyx', degrees=False)
    print(msg.pnt)
    

    msg.motion_time = 1.0 # double variable (unit: second), can be changed
    lc.publish(Main_channel, msg.encode()) # LCM publisher

    # rospy.loginfo(pnt_found)
    # pub_found.publish(pnt_found)
    


    
def listener():
    # Listener is the initialization function and subscribes to the point value of the "find_ball" node and creates the publisher of twists

    rospy.init_node('consensus1', anonymous=True)
    rospy.Subscriber("/desire_ee_pose", Pose, receiver)
    # global pub
    # pub = rospy.Publisher('/ridgeback2/cmd_vel', Twist, queue_size=2)


    

if __name__ == '__main__':
    listener()

rate = rospy.Rate(10)


while not rospy.is_shutdown():
    # This is the infinite loop that keep the program running

    rate.sleep()
    
    # msg.pnt = (0.65, 0.0, 0.44)  # position values
    # msg.pnt = (0.726, 0.0, 0.367)  # position values
    # # r1 = R.from_euler('xyz',[0,1.57,-1.57])
    # r1 = R.from_euler('xyz',[0,0,-1.57])
    # print(r1.as_euler('zyx', degrees=False))
    # msg.rpy = r1.as_euler('zyx', degrees=False)
    # msg.motion_time = 1.0 # double variable (unit: second), can be changed
    # lc.publish(Main_channel, msg.encode()) # LCM publisher


    # rospy.loginfo(pn