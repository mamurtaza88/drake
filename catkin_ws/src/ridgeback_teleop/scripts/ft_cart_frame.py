#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
import copy
import sys, select, termios, tty
from wrist_tracking.msg import lcmt_iiwa_ee_pose
from scipy.spatial.transform import Rotation as R
import numpy as np
from numpy.linalg import inv
from geometry_msgs.msg import Wrench





init = True
init_ft = WrenchStamped()
ft_unbias = WrenchStamped()
ft_EE = np.array([0,0,0,0,0,0])
ft_CP = np.array([0,0,0,0,0,0])
ft_cp_frame = Wrench()
r_aux = R.from_euler('xyz',[0,1.57, -1.57])
position = np.array([0.65,0,0.4]) 

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

def Transform_calculation(data):

    global ft_EE
    global ft_CP
    global r_aux

    r_aux = R.from_euler('zyx',data.ee_rpy)
    position = np.array(data.ee_pose) 
    # r_align = R.from_euler('xyz',[-np.pi/2, 0, -np.pi/2])
    # ang = r_aux.as_euler('xyz', degrees=False)
    # r_ee = R.from_euler('xyz',[ang[2],ang[1],ang[0]])

    # r_cp = R.from_euler('xyz',[0,0,0])

    # #The origin of the ft sensor and the contact point from the end-effector frame
    # ft_ee = np.array([0,0,0.02])
    # cp_ee = np.array([0.0,0.0,0.2])
    
    # # Position of the ft sensor and contact point in the world frame
    # ft_align = r_align.apply(ft_ee)
    # cp_align = r_align.apply(cp_ee)
    # ft_s = r_ee.apply(ft_align) + np.array(data.ee_pose) 
    # cp_s = r_ee.apply(cp_align) + np.array(data.ee_pose) 

    # #Transformation from the ft frame to the world frame (s)
    # rot_ft_s = np.matmul(r_ee.as_dcm(),r_align.as_dcm())
    # skew_ft_s = ssm(ft_s)

    # adjoint_S = np.block([[rot_ft_s, np.matmul(skew_ft_s,rot_ft_s)],[np.zeros((3,3)), rot_ft_s]])
    # adjoint_S_inv = inv(adjoint_S.transpose())
    # ft_S = np.matmul(adjoint_S_inv,ft_EE)
    # skew_cp_s = ssm(cp_s)
    # adjoint_CP = np.block([[np.eye(3), np.matmul(skew_cp_s,np.eye(3))],[np.zeros((3,3)), np.eye(3)]])
    # ft_CP = np.matmul(adjoint_CP.transpose(),ft_S)
    
    # global ft_pub
    # ft_cp_frame.force.x = -ft_CP[0]
    # ft_cp_frame.force.y = -ft_CP[1]
    # ft_cp_frame.force.z = -ft_CP[2]
    # ft_cp_frame.torque.x = -ft_CP[3]
    # ft_cp_frame.torque.y = -ft_CP[4]
    # ft_cp_frame.torque.z = -ft_CP[5]
    # ft_pub.publish(ft_cp_frame)


def ssm(p): #Skew Symmetric Matrix of 3d vector
    return([[0,-p[2], p[1]],[p[2],0,-p[0]],[-p[1],p[0],0]])


def ft_republisher(data):
    
    global ft_EE
    global r_aux
    global position
    ft_EE = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

    r_align = R.from_euler('xyz',[-np.pi/2, 0, -np.pi/2])
    ang = r_aux.as_euler('xyz', degrees=False)
    r_ee = R.from_euler('xyz',[ang[2],ang[1],ang[0]])

    r_cp = R.from_euler('xyz',[0,0,0])

    #The origin of the ft sensor and the contact point from the end-effector frame
    ft_ee = np.array([0,0,0.02])
    cp_ee = np.array([0.0,0.0,0.2])
    
    # Position of the ft sensor and contact point in the world frame
    ft_align = r_align.apply(ft_ee)
    cp_align = r_align.apply(cp_ee)
    ft_s = r_ee.apply(ft_align) + position 
    cp_s = r_ee.apply(cp_align) + position 

    #Transformation from the ft frame to the world frame (s)
    rot_ft_s = np.matmul(r_ee.as_dcm(),r_align.as_dcm())
    skew_ft_s = ssm(ft_s)

    adjoint_S = np.block([[rot_ft_s, np.matmul(skew_ft_s,rot_ft_s)],[np.zeros((3,3)), rot_ft_s]])
    adjoint_S_inv = inv(adjoint_S.transpose())
    ft_S = np.matmul(adjoint_S_inv,ft_EE)
    skew_cp_s = ssm(cp_s)
    adjoint_CP = np.block([[np.eye(3), np.matmul(skew_cp_s,np.eye(3))],[np.zeros((3,3)), np.eye(3)]])
    ft_CP = np.matmul(adjoint_CP.transpose(),ft_S)
    
    global ft_pub
    ft_cp_frame.force.x = -ft_CP[0]
    ft_cp_frame.force.y = -ft_CP[1]
    ft_cp_frame.force.z = -ft_CP[2]
    ft_cp_frame.torque.x = -ft_CP[3]
    ft_cp_frame.torque.y = -ft_CP[4]
    ft_cp_frame.torque.z = -ft_CP[5]
    ft_pub.publish(ft_cp_frame)
    # ft_pub.publish(ft_unbias)

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", init_ft.wrench.force)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ft_sensor_processing', anonymous=True)


    rospy.Subscriber("lcm_to_ros/lcmt_iiwa_ee_pose_2", lcmt_iiwa_ee_pose, Transform_calculation)
    rospy.Subscriber("/ft_unbias", WrenchStamped, ft_republisher)
    
    global ft_pub
    ft_pub = rospy.Publisher('/ft_cp_frame', Wrench, queue_size=10)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    listener()



rate = rospy.Rate(40)

while not rospy.is_shutdown():

    rate.sleep()
    # print(ft_CP)
    # getKey is the fuction that waits for a user input on the keyboard.
    key = getKey()
    if key == '1' :
        init = True
        rospy.loginfo("Restart Unbias")
    else:   
        if (key == '\x03'):
            break