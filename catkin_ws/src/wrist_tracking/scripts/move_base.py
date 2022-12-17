#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import sys, select, termios, tty

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


# Linear PID
acumulated_error_lin = 0
error_lin = 0.05
ref_lin = -0.70
Kp_lin = 0.25
Ki_lin = 0.005
last_lin_vel = 0
max_lin_vel_diff = 0.1




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

def activate_controller(data):
    global Control_State
    if data.x == 1:
        Control_State = True
    else:
        Control_State = False

    # rospy.loginfo("Control: %s",Control_State)


def receiver(data):
    # The receiver function is called everytime a new point is publish.

    global twist
    global target_angular_vel
    global update
    global center
    global Control_State
    global error

    target_angular_vel = 0
    target_lin_vel = 0

    if Control_State:
        if mode == 1:
            target_angular_vel = BangBang(data)

        if mode == 2:
            target_angular_vel = PID(data)
        if abs(center-data.x) < error:
            target_lin_vel = PID_lin(data)   
            
    else:
        target_angular_vel = 0
        target_lin_vel = 0

    # rospy.loginfo("Target Linear Velocity: %.2f",target_lin_vel)

    # After the controller has calculated the control's value, the twisth is updated with the new values
    twist.linear.x = target_lin_vel; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_angular_vel

    # Update = true means that a new twist has been calculated and is ready to be published
    update = True
    
def BangBang(data):

    global bang_step
    global center
    global error

    # For the Bang Bang three possible states are calculated

    if data.x < center -error:
        # If the center of the ball is at the left side of the image, the the robot needs to receive a positive angular velocity
        target_angular_vel  = bang_step
        # rospy.loginfo("Rotating left: %.1f rad/s",target_angular_vel)


    elif data.x > center +error:
        # If the center of the ball is at the right side of the image, the the robot needs to receive a negative angular velocity
        target_angular_vel  = -bang_step
        # rospy.loginfo("Rotating right: %.1f rad/s",target_angular_vel)
    
    else:
        # If the robot is in the center, with some error selected by the user, then the robot stops.
        target_angular_vel  = 0
        # rospy.loginfo("TARGET COMPLETE")

    return target_angular_vel

def PID(data):

    global acumulated_error
    global center
    global error
    global Kp
    global Ki

    # For the PID, the error with the center of the image is calculated
    pid_error = center - data.x

    # The error is acumulated for the integral part of the controler
    acumulated_error = acumulated_error + pid_error 


    if data.x < center - error or data.x > center + error:
        
        # If the center of the ball is not centered in the image, then, the PID is calculated
        target_angular_vel  = Kp*pid_error + Ki*acumulated_error

    
    else:
        # If the center of the ball is in the center of the image, the acumulated error diminish faster to not over shoot.
        acumulated_error = acumulated_error/2
        target_angular_vel  = Kp*pid_error + Ki*acumulated_error

        #rospy.loginfo("TARGET COMPLETE")

    return target_angular_vel

def PID_lin(data):

    global acumulated_error_lin
    global error_lin
    global ref_lin
    global Kp_lin
    global Ki_lin
    global last_lin_vel
    global max_lin_vel_diff

    
    
    pid_error_dist = ref_lin - data.z 
    if abs(pid_error_dist) > error_lin:
        acumulated_error_dist = acumulated_error_lin + pid_error_dist

        target_linear_vel  = -(Kp_lin*pid_error_dist + Ki_lin*acumulated_error_dist)
    else:
        target_linear_vel = 0

    if target_linear_vel - last_lin_vel > max_lin_vel_diff:
        rospy.loginfo("1")
        target_linear_vel = last_lin_vel + max_lin_vel_diff
    elif  target_linear_vel - last_lin_vel < -max_lin_vel_diff:
        rospy.loginfo("2")
        target_linear_vel = last_lin_vel - max_lin_vel_diff

    
    
    last_lin_vel = target_linear_vel
    return target_linear_vel


def listener():
    # Listener is the initialization function and subscribes to the point value of the "find_ball" node and creates the publisher of twists

    rospy.init_node('move_base', anonymous=True)

    rospy.Subscriber("wrist_pos", Point, receiver)
    rospy.Subscriber("wrist_found", Point, activate_controller)
    global pub
    pub = rospy.Publisher('/ridgeback2/cmd_vel', Twist, queue_size=2)


    

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    listener()

rate = rospy.Rate(10)


while not rospy.is_shutdown():
    # This is the infinite loop that keep the program running

    # If a new twist has been calculated, the update is true and the twist is publish
    if update:
        pub.publish(twist)
        rospy.loginfo("Target Angular Velocity: %.2f",twist.angular.z)
        rospy.loginfo("Target Linear Velocity: %.2f",twist.linear.x)
        update = False
    rate.sleep()

    # getKey is the fuction that waits for a user input on the keyboard.
    key = getKey()
    if key == '1' :
        mode = 1
        rospy.loginfo("Selected mode 1: Bang Bang")
    elif key == '2' :
        mode = 2
        acumulated_error = 0
        rospy.loginfo("Selected mode 2: PID")
    elif key == 'w' :
        error = error + 1
        rospy.loginfo("Increased error to: %d", error)
    elif key == 's' :
        error = error - 1
        if error < 2:
            error = 1
        rospy.loginfo("Decreased error to: %d", error)
    elif key == 'e' :
        bang_step = bang_step + 0.01
        rospy.loginfo("Increased speed to: %.2f", bang_step)
    elif key == 'q' :
        bang_step = bang_step - 0.01
        if bang_step < 0.01:
            bang_step = 0.01
        rospy.loginfo("Decreased speed to: %.2f", bang_step)
    elif key == 'f' :
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        update = True        
    else:
        if (key == '\x03'):
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
            # rospy.loginfo("Target Angular Velocity: %.2f",twist.angular.z)
            break