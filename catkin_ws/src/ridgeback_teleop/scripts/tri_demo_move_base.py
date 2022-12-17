#!/usr/bin/env python
import rospy
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R
import sys, select, termios, tty
import numpy as np
from wrist_tracking.msg import lcmt_iiwa_ee_pose


# Global variables are created
target_angular_vel  = 0
update = False
mode = 2
error = 0.05
center = 0.1
bang_step = 0.13
twist = Twist()
Kp = 0.4
Ki = 0.002
acumulated_error = 0
Control_State = False
state = 1
offset = [-1,-0.03,0]


# Linear PID
acumulated_error_lin = 0
error_lin = 0.05
ref_lin = -0.70
Kp_lin = 0.25
Ki_lin = 0.005
last_lin_vel = 0
max_lin_vel_diff = 0.1


#goal variables
cart_Pose = Pose()
goal_Pose = Pose()

fixed_orientation = False

ee_goal = np.array([0.726, 0, 0.36])
ee_pos = np.array([0.726, 0, 0.36])
First_point = True
rotation = 0

class PID:
    def __init__(self,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_e = 0 # Integrated error
        self.p_e = 0 # previous error
        self.dt = 0.005
        self.start = True
        self.max_u = 1
        self.u_ant = 0
        self.max_u_dif = 0.05

    def control(self, error):

        self.i_e = self.i_e + error*self.dt
        if self.start:
            self.start = False
            d_e = 0
        else:
            d_e = (error-self.p_e)/self.dt
        
        u = self.kp*error + self.kd*d_e + self.ki*self.i_e
        self.p_e = error
        if np.abs(u-self.u_ant)>self.max_u_dif:
            u = self.u_ant + self.max_u_dif*np.sign(u-self.u_ant)
        self.u_ant = u
        return u


pid_x = PID(3,0.000004,0.000001)
pid_y = PID(3,0.000004,0.000001)
pid_theta = PID(1.5,0.000003,0.000008)

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

def get_ee_position(data):

    global ee_pos
    global ee_goal
    global twist
    global update
    global pid_x, pid_y, pid_theta
    global First_point

    ee_pos = np.array([data.ee_pose[0], data.ee_pose[1], data.ee_pose[2]])
    if First_point:
        ee_goal = ee_pos
        First_point = False
    error = ee_goal-ee_pos
    print("pos: ")
    print(ee_pos)
    print("error: ")

    print(error)
    twist.linear.x = pid_x.control(error[0]); twist.linear.y = pid_y.control(error[1]); twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0#-pid_theta.control(error[1])

    # Update = true means that a new twist has been calculated and is ready to be published
    update = True
    

def listener():
    # Listener is the initialization function and subscribes to the point value of the "find_ball" node and creates the publisher of twists

    rospy.init_node('Control_ridgeback_base', anonymous=True)

    rospy.Subscriber("/lcm_to_ros/lcmt_iiwa_ee_pose_2", lcmt_iiwa_ee_pose, get_ee_position)

    
    global pub
    global pub_ee_pose
    global pub_cp_cart
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    pub_ee_pose = rospy.Publisher('/desire_ee_pose', Pose, queue_size=2)


    

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    listener()

rate = rospy.Rate(40)

robotiq_client = actionlib.SimpleActionClient('command_robotiq_action', CommandRobotiqGripperAction)
goal = CommandRobotiqGripperGoal()
goal.emergency_release = False
goal.stop = False
goal.position = 0.00 #between 0.00 (close) to 0.085 (open)
goal.speed = 0.1
goal.force = 5.0
height = 0.4

# Sends the goal to the gripper.
robotiq_client.send_goal(goal)

speed_offset = 0
grasp_offset = 0


while not rospy.is_shutdown():
    # This is the infinite loop that keep the program running

    # If a new twist has been calculated, the update is true and the twist is publish
    if update:
        pub.publish(twist)
        update = False
    rate.sleep()

    # getKey is the fuction that waits for a user input on the keyboard.
    straight_line = True
    key = getKey()
    if key == '1':
        state = 1
        rospy.loginfo("State = 1")
        rospy.loginfo("position MM 1 meter behind the cart")
        offset = [-1,-0.03,0]
        # Sends the goal to the gripper.
        goal.position = 0.08
        robotiq_client.send_goal(goal)
    elif key == '2' :
        state = 2
        rospy.loginfo("State = 2")
        rospy.loginfo("position MM 0.63 meter behind the cart")
        offset = [-0.64,-0.03,0]
        # Sends the goal to the gripper.
        goal.position = 0.00
        First_point = True
        robotiq_client.send_goal(goal)

    else:
        if (key == '\x03'):
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
            # rospy.loginfo("Target Angular Velocity: %.2f",twist.angular.z)
            break