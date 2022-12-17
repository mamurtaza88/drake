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
import math

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
offset = [-1,0.0,0]
counter = 0
start_counter = False
gripper_offset = 0.4


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
MM_Pose = Pose()
cart_orientation = 0.0
follow_trajectory = False

goal_Pose = Pose()
goal_Pose_ee = Pose()
goal_Pose_ee.orientation.x = 0.0
goal_Pose_ee.orientation.y = 0.7068252
goal_Pose_ee.orientation.z = 0.0
goal_Pose_ee.orientation.w = 0.7073883
goal_Pose_ee.position.x = 0.650
goal_Pose_ee.position.y = 0.0
goal_Pose_ee.position.z = 0.44
fixed_orientation = False
rotation = 0

cart_CP = np.array([0,0,0])
CP_pos = np.array([0,0,0])



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


pid_x = PID(0.5,0.04,0.001)
pid_y = PID(0.5,0.04,0.001)
pid_theta = PID(0.8,0.03,0.008)
pid_hand_orientation = PID(1,0.000,0.00)
pid_hand_orientation.max_u_dif = 0.0001


def trajectory(x,y):
    #Define the desired trajectory that we want the cart to follow
    desired_position = np.array([x + 0.2, 0, 0]) #defined in SE(2) ---> (x, y, theta)
    return desired_position
    
    

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

def Cart_position(data):

    # Gets the position of the cart (center of the handle), we need to compute the updated position of the contact point
    # and set the new position for the end-effector.
    global cart_Pose
    global cart_CP
    global CP_pos
    global cart_orientation
    
    cart_Pose = data.pose
    pos_cart = np.array([cart_Pose.position.x, cart_Pose.position.y, cart_Pose.position.z])
    r = R.from_quat([cart_Pose.orientation.x, cart_Pose.orientation.y, cart_Pose.orientation.z, cart_Pose.orientation.w])
    cart_orientation = r.as_rotvec()[2]
    
    CP_pos = pos_cart + r.apply(cart_CP)

def orientation_error(q, qd):
    w = q.w
    w_d = qd.w
    xyz = np.array([q.x, q.y, q.z])
    xyz_d = np.array([qd.x, qd.y, qd.z])
    skew = skew_symmetric_matrix(xyz)

    return xyz*w_d - xyz_d*w+ np.dot(skew,xyz)

def skew_symmetric_matrix(p):
    return np.array([[0, -p[2], p[1]],[p[2],0,-p[0]],[-p[1],p[0],0]])


def ridgeback_control(data):
    # The receiver function is called everytime a new point is publish.

    global cart_Pose
    global MM_Pose
    global CP_pos
    global EE_quat_error
    global twist
    global update
    global goal_Pose
    global pid_x, pid_y, pid_theta
    global offset
    global counter
    global goal_pos
    global arm_error_b
    global cart_pos
    
    MM_Pose = data.pose
    MM_pos = np.array([MM_Pose.position.x, MM_Pose.position.y, MM_Pose.position.z])
    # Desired position of the base. 
    cart_pos = np.array([cart_Pose.position.x, cart_Pose.position.y, cart_Pose.position.z])
    cart_r = R.from_quat([cart_Pose.orientation.x, cart_Pose.orientation.y, cart_Pose.orientation.z, cart_Pose.orientation.w])
    test_offset = offset + np.array([np.abs(0.2*math.sin(counter)),0.4*math.sin(counter),0])
    if(start_counter):
        counter = counter + 0.002
    offset_rot = cart_r.apply(test_offset)
    goal_pos = cart_pos + offset_rot
    

    cart_error_w = goal_pos-MM_pos
    MM_r = R.from_quat([MM_Pose.orientation.x, MM_Pose.orientation.y, MM_Pose.orientation.z, MM_Pose.orientation.w])
    r_inv = MM_r.inv()
    error_b = r_inv.apply(cart_error_w)


    # Error between the orientation of the cart (or CP) and the base of the MM. Based on this error
    # we update the angle of the end-effector. 
    MM_quat_error = orientation_error(MM_Pose.orientation, cart_Pose.orientation)
    MM_quat_error[0] = 0
    MM_quat_error[1] = 0
    EE_quat_error = np.array([0,0,MM_quat_error[2]])
    cart_desired_pose = trajectory(cart_pos[0], cart_pos[1])
    if follow_trajectory:
        cart_orientation_error = (cart_desired_pose[2] - cart_r.as_rotvec()[2]) + (cart_desired_pose[1]-cart_pos[1])*0.2
        EE_quat_error[2] = MM_quat_error[2] - pid_hand_orientation.control(cart_orientation_error)
    r_aux = R.from_rotvec(EE_quat_error)
    q_aux = r_aux.as_quat()
    goal_Pose_ee.orientation.x = q_aux[0]
    goal_Pose_ee.orientation.y = q_aux[1]
    goal_Pose_ee.orientation.z = q_aux[2]
    goal_Pose_ee.orientation.w = q_aux[3]

    # Error between the base of the MM and the CP position
    arm_error_w = CP_pos - MM_pos
    arm_error_w[2] = arm_error_w[2] + gripper_offset
    arm_error_b = r_inv.apply(arm_error_w)


    if arm_error_b[0]<0.1:
        arm_error_b[0] = 0.1
    elif arm_error_b[0]>0.7:
        arm_error_b[0] = 0.7
        
    #Contraint in the y dimension
    if arm_error_b[1]<-0.7:
        arm_error_b[1] = -0.7
    elif arm_error_b[1]>0.7:
        arm_error_b[1] = 0.7

    #Contraint in the z dimension
    if arm_error_b[2]<0.25:
        arm_error_b[2] = 0.25
    elif arm_error_b[2]>0.75:
        arm_error_b[2] = 0.75


    goal_Pose_ee.position.x =  arm_error_b[0]
    goal_Pose_ee.position.y =  arm_error_b[1]
    goal_Pose_ee.position.z =  arm_error_b[2]

    if follow_trajectory:
        goal_Pose_ee.position.x = goal_Pose_ee.position.x + 0.06


    
    twist.linear.x = -pid_x.control(error_b[0]); twist.linear.y = -pid_y.control(error_b[1]); twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -pid_theta.control(MM_quat_error[2])
    
    # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

    # Update = true means that a new twist has been calculated and is ready to be published
    update = True

def listener():
    # Listener is the initialization function and subscribes to the point value of the "find_ball" node and creates the publisher of twists

    rospy.init_node('Control_ridgeback_base', anonymous=True)

    rospy.Subscriber("/vrpn_client_node/Ridgeback2/pose", PoseStamped, ridgeback_control)
    rospy.Subscriber("/vrpn_client_node/Shopping_cart/pose", PoseStamped, Cart_position)
    
    global pub
    global pub_ee_pose
    global pub_cp_cart
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    pub_ee_pose = rospy.Publisher('/desire_ee_pose', Pose, queue_size=2)
    pub_cp_cart = rospy.Publisher('/cp_cart_point', Point, queue_size=2)


    

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    listener()

rate = rospy.Rate(40)

robotiq_client = actionlib.SimpleActionClient('command_robotiq_action', CommandRobotiqGripperAction)
goal = CommandRobotiqGripperGoal()
goal.emergency_release = False
goal.stop = False
goal.position = 0.08 #between 0.00 (close) to 0.085 (open)
goal.speed = 0.1
goal.force = 5.0
height = 0.4

# Sends the goal to the gripper.
robotiq_client.send_goal(goal)

speed_offset = 0
grasp_offset = 0
pose_ee = Point()
pose_ee.x = 0
pose_ee.y = 0
pose_ee.z = 0

while not rospy.is_shutdown():
    # This is the infinite loop that keep the program running

    # If a new twist has been calculated, the update is true and the twist is publish
    if update:
        pub.publish(twist)
        pub_ee_pose.publish(goal_Pose_ee)
        update = False
    rate.sleep()

    print("EE_quat_error: ")
    print(EE_quat_error[2]*180/3.14156)
    print("Cart_pos: ")
    print(cart_pos[0:2])

    # getKey is the fuction that waits for a user input on the keyboard.
    straight_line = True
    key = getKey()
    if key == '1':
        state = 1
        rospy.loginfo("State = 1")
        rospy.loginfo("position MM 1 meter behind the cart")
        # offset = [-1,-0.03,0]
        offset = [-1,0,0]
        # Sends the goal to the gripper.
        goal.position = 0.08
        robotiq_client.send_goal(goal)
        pid_x.i_e = 0
        gripper_offset = 0.4
        follow_trajectory = False
    elif key == '2' :
        state = 2
        rospy.loginfo("State = 2")
        rospy.loginfo("position MM 0.63 meter behind the cart")
        # offset = [-0.64,-0.03,0]
        offset = [-0.54,0,0]
        # Sends the goal to the gripper.
        goal.position = 0.08
        robotiq_client.send_goal(goal)
        pid_x.i_e = 0
        start_counter = False
        counter = 0
        gripper_offset = 0.4
        follow_trajectory = False
    elif key == '3' :
        state = 3
        rospy.loginfo("State = 3")
        rospy.loginfo("position MM 0.63 meter behind the cart and lower end-effector")
        offset = [-0.54,0,0]
        fixed_orientation = straight_line
        pid_x.i_e = 0
        gripper_offset = 0.33
        follow_trajectory = False
    elif key == '4' :
        state = 4
        rospy.loginfo("State = 4")
        rospy.loginfo("Close Gripper")
        goal.position = 0.025
        robotiq_client.send_goal(goal)
        pid_x.i_e = 0
        follow_trajectory = False
    elif key == '5' :
        state = 5
        rospy.loginfo("State = 5")
        rospy.loginfo("Open Gripper")
        goal.position = 0.08
        robotiq_client.send_goal(goal)
        pid_x.i_e = 0
        follow_trajectory = False
    elif key == '6' :
        state = "6"
        rospy.loginfo("State = 6")
        rospy.loginfo("move base sideways sinusoid")
        start_counter = True
        follow_trajectory = False
    elif key == '7':
        state = "7"
        rospy.loginfo("State = 7")
        rospy.loginfo("Follow linear trajectory")
        follow_trajectory = True
    else:
        if (key == '\x03'):
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
            goal_Pose_ee.position.x = 0.650
            goal_Pose_ee.position.y = grasp_offset
            goal_Pose_ee.position.z = 0.48
            pub_ee_pose.publish(goal_Pose_ee)
            # rospy.loginfo("Target Angular Velocity: %.2f",twist.angular.z)
            break