#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
import copy
import sys, select, termios, tty



init = True
init_ft = WrenchStamped()
ft_unbias = WrenchStamped()

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
    global init_ft
    
    if init:
        init_ft = data
        init = False

    ft_unbias.header = data.header
    alpha = 0.5

    ft_unbias.wrench.force.x = (ft_unbias.wrench.force.x)*(1-alpha) + (data.wrench.force.x - init_ft.wrench.force.x)*alpha
    ft_unbias.wrench.force.y = (ft_unbias.wrench.force.y)*(1-alpha) + (data.wrench.force.y - init_ft.wrench.force.y)*alpha
    ft_unbias.wrench.force.z = (ft_unbias.wrench.force.z)*(1-alpha) + (data.wrench.force.z - init_ft.wrench.force.z)*alpha
    ft_unbias.wrench.torque.x = (ft_unbias.wrench.torque.x)*(1-alpha) + (data.wrench.torque.x - init_ft.wrench.torque.x)*alpha
    ft_unbias.wrench.torque.y = (ft_unbias.wrench.torque.y)*(1-alpha) + (data.wrench.torque.y - init_ft.wrench.torque.y)*alpha
    ft_unbias.wrench.torque.z = (ft_unbias.wrench.torque.z)*(1-alpha) + (data.wrench.torque.z - init_ft.wrench.torque.z)*alpha


    # ft_unbias.wrench.force.x =  data.wrench.force.x - init_ft.wrench.force.x
    # ft_unbias.wrench.force.y = data.wrench.force.y - init_ft.wrench.force.y
    # ft_unbias.wrench.force.z = data.wrench.force.z - init_ft.wrench.force.z
    # ft_unbias.wrench.torque.x = data.wrench.torque.x - init_ft.wrench.torque.x
    # ft_unbias.wrench.torque.y = data.wrench.torque.y - init_ft.wrench.torque.y
    # ft_unbias.wrench.torque.z = data.wrench.torque.z - init_ft.wrench.torque.z 

    ft_pub.publish(ft_unbias)

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", init_ft.wrench.force)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ft_sensor_processing', anonymous=True)

    rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, ft_republisher)
    
    global ft_pub
    ft_pub = rospy.Publisher('/ft_unbias', WrenchStamped, queue_size=10)
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