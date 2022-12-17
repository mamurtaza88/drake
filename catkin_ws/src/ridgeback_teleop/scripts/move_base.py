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

def set_goal_cart(data):

    global cart_Pose
    global goal_Pose
    global goal_Pose_ee
    global offset
    global speed_offset
    global rotation
    
    if fixed_orientation:
        
        goal_Pose.orientation = cart_Pose.orientation
        r = R.from_quat([cart_Pose.orientation.x, cart_Pose.orientation.y, cart_Pose.orientation.z, cart_Pose.orientation.w])
        # ang = r.as_euler('xyz', degrees=False)
        # ang[2] = ang[2]- rotation
        # r2 = R.from_euler('xyz', ang)
        # quat_aux = r2.as_quat()
        # goal_Pose.orientation.x = quat_aux[0]
        # goal_Pose.orientation.y = quat_aux[1]
        # goal_Pose.orientation.z = quat_aux[2]
        # goal_Pose.orientation.w = quat_aux[3]
        
        pos = np.array([cart_Pose.position.x, cart_Pose.position.y, cart_Pose.position.z])
        offset_rot = r.apply([speed_offset,0,0])
        goal_pos = pos + offset_rot
        goal_Pose.position.x =  goal_pos[0]
        goal_Pose.position.y =  goal_pos[1]
        goal_Pose.position.z =  goal_pos[2]
    else:
        goal_Pose.orientation = data.pose.orientation
        r = R.from_quat([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        offset_rot = r.apply(offset)
        goal_pos = pos + offset_rot
        goal_Pose.position.x =  goal_pos[0]
        goal_Pose.position.y =  goal_pos[1]
        goal_Pose.position.z =  goal_pos[2]
    

def orientation_error(q, qd):
    w = q.w
    w_d = q.w
    xyz = np.array([q.x, q.y, q.z])
    xyz_d = np.array([qd.x, qd.y, qd.z])
    skew = np.array([[0, -xyz[2], xyz[1]],[xyz[2],0,-xyz[0]],[-xyz[1], xyz[0], 0]])

    return xyz*w_d - xyz_d*w+ np.dot(skew,xyz)



def ridgeback_control(data):
    # The receiver function is called everytime a new point is publish.

    global twist
    global update
    global goal_Pose
    global pid_x, pid_y, pid_theta
    global cart_Pose
    
    quat_error = orientation_error(data.pose.orientation, goal_Pose.orientation)

    r_aux = R.from_rotvec(quat_error)
    q_aux = r_aux.as_quat()
    goal_Pose_ee.orientation.x = q_aux[0]
    goal_Pose_ee.orientation.y = q_aux[1]
    goal_Pose_ee.orientation.z = q_aux[2]
    goal_Pose_ee.orientation.w = q_aux[3]

    cart_Pose = data.pose

    pos_g = np.array([goal_Pose.position.x, goal_Pose.position.y, goal_Pose.position.z])
    pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

    error_w = pos_g-pos
    r = R.from_quat([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    r_inv = r.inv()
    error_b = r_inv.apply(error_w)
    
    twist.linear.x = -pid_x.control(error_b[0]); twist.linear.y = -pid_y.control(error_b[1]); twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -pid_theta.control(quat_error[2])

    # Update = true means that a new twist has been calculated and is ready to be published
    update = True

def listener():
    # Listener is the initialization function and subscribes to the point value of the "find_ball" node and creates the publisher of twists

    rospy.init_node('Control_ridgeback_base', anonymous=True)

    rospy.Subscriber("/vrpn_client_node/Ridgeback2/pose", PoseStamped, ridgeback_control)
    rospy.Subscriber("/vrpn_client_node/Shopping_cart/pose", PoseStamped, set_goal_cart)
    
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
        goal_Pose_ee.position.x = 0.650
        goal_Pose_ee.position.y = grasp_offset
        goal_Pose_ee.position.z = 0.44
        pid_x.i_e = 0
    elif key == '2' :
        state = 2
        rospy.loginfo("State = 2")
        rospy.loginfo("position MM 0.63 meter behind the cart")
        offset = [-0.64,-0.03,0]
        # Sends the goal to the gripper.
        goal.position = 0.08
        robotiq_client.send_goal(goal)
        goal_Pose_ee.position.x = 0.650
        goal_Pose_ee.position.y = grasp_offset
        goal_Pose_ee.position.z = 0.44
        pid_x.i_e = 0
    elif key == '3' :
        state = 3
        rospy.loginfo("State = 3")
        rospy.loginfo("position MM 0.63 meter behind the cart and lower end-effector")
        offset = [-0.64,-0.03,0]
        goal_Pose_ee.position.x = 0.650
        goal_Pose_ee.position.y = grasp_offset
        goal_Pose_ee.position.z = height
        fixed_orientation = straight_line
        pid_x.i_e = 0
    elif key == '4' :
        state = 4
        rospy.loginfo("State = 4")
        rospy.loginfo("Close Gripper")
        goal.position = 0.025
        robotiq_client.send_goal(goal)
        pid_x.i_e = 0
    elif key == '5' :
        state = 5
        rospy.loginfo("State = 5")
        rospy.loginfo("Open Gripper")
        goal.position = 0.08
        robotiq_client.send_goal(goal)
        pid_x.i_e = 0
    elif key == 'w' :
        state = "w"
        rospy.loginfo("State = w")
        rospy.loginfo("Speed up the mobile base")
        speed_offset = speed_offset + 0.02
        offset = [-0.64+speed_offset,-0.03,0]
        goal_Pose_ee.position.x = 0.650
        goal_Pose_ee.position.y = grasp_offset
        goal_Pose_ee.position.z = height
        pid_x.i_e = 0
        fixed_orientation = straight_line
    elif key == 'x' :
        state = 'x'
        rospy.loginfo("State = x")
        rospy.loginfo("slow down the mobile base")
        speed_offset = speed_offset - 0.02
        offset = [-0.64+speed_offset,-0.03,0]
        goal_Pose_ee.position.x = 0.650
        goal_Pose_ee.position.y = grasp_offset
        goal_Pose_ee.position.z = height 
        pid_x.i_e = 0
        fixed_orientation = straight_line
    elif key == 's' :
        state = 's'
        rospy.loginfo("State = s")
        rospy.loginfo("stop the mobile base")
        speed_offset = 0
        offset = [-0.64+speed_offset,-0.03,0]
        goal_Pose_ee.position.x = 0.650
        goal_Pose_ee.position.y = grasp_offset
        goal_Pose_ee.position.z = height 
        pid_x.i_e = 0
        fixed_orientation = straight_line
    elif key == 'd' :
        state = 'd'
        rospy.loginfo("State = d")
        rospy.loginfo("Move end-effector to the right")
        grasp_offset = grasp_offset - 0.005
        goal.position = 0.033
        robotiq_client.send_goal(goal)
        if grasp_offset < -0.18:
            grasp_offset = -0.18
        goal_Pose_ee.position.y = grasp_offset
        pose_ee.y = grasp_offset
        pub_cp_cart.publish(pose_ee)
        pid_x.i_e = 0
    elif key == 'a' :
        state = 'a'
        rospy.loginfo("State = a")
        rospy.loginfo("Move end-effector to the left")
        goal.position = 0.033
        robotiq_client.send_goal(goal)
        grasp_offset = grasp_offset + 0.005
        if grasp_offset > 0.18:
            grasp_offset = 0.18
        goal_Pose_ee.position.y = grasp_offset
        pid_x.i_e = 0
        pose_ee.y = grasp_offset
        pub_cp_cart.publish(pose_ee)

    elif key == 't' :
        state = 't'
        rospy.loginfo("State = t")
        rospy.loginfo("Circle")
        speed_offset = 0
        offset = [-0.64+speed_offset,-0.03,0]
        goal_Pose_ee.position.x = 0.650
        goal_Pose_ee.position.y = grasp_offset
        goal_Pose_ee.position.z = height
        pid_x.i_e = 0
        fixed_orientation = straight_line
        rotation = rotation + 0.01
    elif key == 'g' :
        state = 'g'
        rospy.loginfo("State = g")
        rospy.loginfo("Circle")
        speed_offset = 0
        offset = [-0.64+speed_offset,-0.03,0]
        goal_Pose_ee.position.x = 0.650
        goal_Pose_ee.position.y = grasp_offset
        goal_Pose_ee.position.z = height
        pid_x.i_e = 0
        fixed_orientation = straight_line
        rotation = 0
    elif key == 'b' :
        state = 'b'
        rospy.loginfo("State = b")
        rospy.loginfo("Circle")
        speed_offset = 0
        offset = [-0.64+speed_offset,-0.03,0]
        goal_Pose_ee.position.x = 0.650
        goal_Pose_ee.position.y = grasp_offset
        goal_Pose_ee.position.z = height
        pid_x.i_e = 0
        fixed_orientation = straight_line
        rotation = rotation - 0.01
    else:
        if (key == '\x03'):
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
            goal_Pose_ee.position.x = 0.650
            goal_Pose_ee.position.y = grasp_offset
            goal_Pose_ee.position.z = 0.44
            pub_ee_pose.publish(goal_Pose_ee)
            # rospy.loginfo("Target Angular Velocity: %.2f",twist.angular.z)
            break