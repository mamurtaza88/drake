#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench
from wrist_tracking.msg import lcmt_iiwa_status
import copy
import sys, select, termios, tty
import numpy as np


init = True
init_ft = WrenchStamped()
joint_wrench_unbias = Wrench()
joint_wrench_init = Wrench()
joint_wrench_prev = Wrench()

def getKey():
    # Get key function is used to receive user interaction with the keyboard through the terminal in which "drive_wheel" is running
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def ft_republisher(data):
    
    global init
    global joint_wrench_init
    global joint_wrench_prev
    
    if init:
        joint_wrench_init.force.x = data.joint_torque_external[5]/0.33
        joint_wrench_init.force.y = data.joint_torque_external[5]*0
        joint_wrench_init.force.y = data.joint_torque_external[5]*0
        joint_wrench_init.torque.x = data.joint_torque_external[5]*0
        joint_wrench_init.torque.y = data.joint_torque_external[5]*0
        joint_wrench_init.torque.z = data.joint_torque_external[6]
        joint_wrench_prev.force.x = 0
        joint_wrench_prev.force.y = 0
        joint_wrench_prev.force.z = 0
        joint_wrench_prev.torque.x = 0
        joint_wrench_prev.torque.y = 0
        joint_wrench_prev.torque.z = 0
        init = False

    alpha = 0.99
    
    joint_wrench_prev.force.x  = joint_wrench_prev.force.x *(1-alpha) + (data.joint_torque_external[5]/0.33 - joint_wrench_init.force.x)*alpha
    joint_wrench_prev.force.y  = joint_wrench_prev.force.y *(1-alpha)
    joint_wrench_prev.force.z  = joint_wrench_prev.force.z *(1-alpha)
    joint_wrench_prev.torque.x = joint_wrench_prev.torque.x*(1-alpha)
    joint_wrench_prev.torque.y = joint_wrench_prev.torque.y*(1-alpha)
    joint_wrench_prev.torque.z = joint_wrench_prev.torque.z*(1-alpha) + (data.joint_torque_external[6] - joint_wrench_init.torque.z)*alpha
    
    joint_wrench_unbias.force.x = joint_wrench_prev.force.x
    joint_wrench_unbias.force.y = 0
    joint_wrench_unbias.force.z = 0
    joint_wrench_unbias.torque.x = 0
    joint_wrench_unbias.torque.y = 0
    joint_wrench_unbias.torque.z = joint_wrench_prev.torque.z

    ft_pub.publish(joint_wrench_unbias)
    # print(data.joint_torque_external[5])
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", init_ft.wrench.force)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joint_wrench_sensor_processing', anonymous=True)
    rospy.Subscriber("lcm_to_ros/lcmt_iiwa_status_2", lcmt_iiwa_status, ft_republisher)

    
    global ft_pub
    ft_pub = rospy.Publisher('/joint_wrench_unbias', Wrench, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    listener()



rate = rospy.Rate(10)

while not rospy.is_shutdown():

    rate.sleep()

    # getKey is the fuction that waits for a user input on the keyboard.
    key = getKey()
    if key == '1' :
        init = True
        rospy.loginfo("Restart Unbias")
    else:   
        if (key == '\x03'):
            break