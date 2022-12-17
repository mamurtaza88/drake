#! /usr/bin/env python3
import rospy
from rospy import Time 
from geometry_msgs.msg import Point
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity
from std_msgs.msg import Header

import cv2
import sys

#LCM
import lcm
from exlcm import example_t


import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm
np.set_printoptions(linewidth=100, formatter={'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

import time

wrist_num = 7

# Create LCM object
Main_channel = str("Hand_Position")
lc = lcm.LCM() # Creare a lcm object
msg = example_t()  # communication format

IIWA14 = rtb.models.DH.IIWA14()
# print(IIWA14)

# IIWA14.plot(IIWA14.qk4)
# print(IIWA14.fkine(IIWA14.qk4))

def callback(data):
    # rospy.loginfo("I heard %s", data.position)

    q = [data.position.a1, data.position.a2, data.position.a3, data.position.a4, data.position.a5, data.position.a6, data.position.a7]
    Taux = IIWA14.fkine(q)

    print(Taux)
    # rospy.loginfo("Goal: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",Taux.t[0], Taux.t[1],Taux.t[2],Taux.eul()[0],Taux.eul()[1]-1.5708,Taux.eul()[2])
    # print(Taux.eul())
    msg.pnt = (Taux.t[0], Taux.t[1],Taux.t[2],Taux.rpy(order='xyz')[0],Taux.eul()[0],Taux.eul()[1]-1.5708,Taux.eul()[2])  # position values
    # cnt_cos = cnt_cos + 0.1
    # msg.pnt = (0.75, 0.0, 0.45 + 0.15*np.cos(cnt_cos))  # position values

    msg.motion_time = 1.0 # double variable (unit: second), can be changed
    lc.publish(Main_channel, msg.encode())

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("myJointPosition", JointPosition, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()