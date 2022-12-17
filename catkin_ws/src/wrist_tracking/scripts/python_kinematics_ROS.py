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

T1 = SE3(0.6, 0.0, 0.35) * SE3.Ry(180, unit='deg')
T2 = SE3(0.6, 0.2, 0.35) * SE3.Ry(180, unit='deg')
T3 = SE3(0.6, 0.2, 0.55) * SE3.Ry(180, unit='deg')
T4 = SE3(0.6, -0.2, 0.55) * SE3.Ry(180, unit='deg')
T5 = SE3(0.6, -0.2, 0.35) * SE3.Ry(180, unit='deg')
T6 = SE3(0.6, 0.0, 0.35) * SE3.Ry(180, unit='deg')
# print(T)

sol1 = IIWA14.ikine_LM(T1, q0=IIWA14.qk4)
# print(IIWA14.qk4)
# print(sol)
qt1 = rtb.tools.trajectory.jtraj(IIWA14.qk4, sol1.q, 10)
sol2 = IIWA14.ikine_LM(T2, q0=sol1.q)
qt2 = rtb.tools.trajectory.jtraj(sol1.q, sol2.q, 20)

sol3 = IIWA14.ikine_LM(T3, q0=sol2.q)
qt3 = rtb.tools.trajectory.jtraj(sol2.q, sol3.q, 20)

sol4 = IIWA14.ikine_LM(T4, q0=sol3.q)
qt4 = rtb.tools.trajectory.jtraj(sol3.q, sol4.q, 20)

sol5 = IIWA14.ikine_LM(T5, q0=sol4.q)
qt5 = rtb.tools.trajectory.jtraj(sol4.q, sol5.q, 20)

sol6 = IIWA14.ikine_LM(T6, q0=sol5.q)
qt6 = rtb.tools.trajectory.jtraj(sol5.q, sol6.q, 20)


# # print(qt.q)
# rtb.tools.trajectory.qplot(qt.q, block=False)
joint_pos = np.concatenate((qt1.q, qt2.q,qt3.q, qt4.q, qt5.q, qt6.q), axis=0)
len_points = len(joint_pos)
print(len_points)
# IIWA14.plot(np.concatenate((qt1.q, qt2.q,qt3.q, qt4.q, qt5.q, qt6.q), axis=0),dt=0.05)
def talker():

    global wrist_num
    global cnt_cos
    global counter

    pub = rospy.Publisher('/myJointPosition', JointPosition, queue_size=10)
    pub_found = rospy.Publisher('wrist_found', Point, queue_size=10)
    rospy.init_node('Second_Arm_pos', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    counter = 1
    angle = 0.1

    pnt_command = Point(0.8,0.0,0.4)
    first = True
    goal_joints = JointPosition()
    goal_joints.header = Header()
    goal_joints.position = JointQuantity(0,0,0,0,0,0,0)
    pub.publish(goal_joints)
    while not rospy.is_shutdown():

        # rospy.loginfo("Goal: %.2f, %.2f, %.2f",pnt.x, pnt.y, pnt.z)
    
        # rospy.loginfo("Goal: %f",counter)
    
        # rospy.loginfo("Goal: %.2f, %.2f, %.2f",pnt_command.x, pnt_command.y, pnt_command.z)
        Taux = IIWA14.fkine(joint_pos[counter])
        # rospy.loginfo("Goal: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",Taux.t[0], Taux.t[1],Taux.t[2],Taux.eul()[0],Taux.eul()[1]-1.5708,Taux.eul()[2])
        print(Taux.eul())
        msg.pnt = (Taux.t[0], Taux.t[1],Taux.t[2],Taux.rpy(order='xyz')[0],Taux.eul()[0],Taux.eul()[1]-1.5708,Taux.eul()[2])  # position values
        # cnt_cos = cnt_cos + 0.1
        # msg.pnt = (0.75, 0.0, 0.45 + 0.15*np.cos(cnt_cos))  # position values

        msg.motion_time = 1.0 # double variable (unit: second), can be changed
        lc.publish(Main_channel, msg.encode()) # LCM publisher
        counter = counter + 1
        if counter == len_points:
            counter = 11


        # IIWA14.plot(joint_pos[counter])
        print(counter)
        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass