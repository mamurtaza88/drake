#! /usr/bin/env python3

import rospy
from rospy import Time 
from geometry_msgs.msg import Point

import cv2
import sys
import pyzed.sl as sl
import ogl_viewer.viewer as gl
import cv_viewer.tracking_viewer as cv_viewer
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

# Create a Camera object
zed = sl.Camera()

# Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
init_params.coordinate_units = sl.UNIT.METER          # Set coordinate units
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

# If applicable, use the SVO given as parameter
# Otherwise use ZED live stream
if len(sys.argv) == 2:
    filepath = sys.argv[1]
    print("Using SVO file: {0}".format(filepath))
    init_params.svo_real_time_mode = True
    init_params.set_from_svo_file(filepath)

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)

# Enable Positional tracking (mandatory for object detection)
positional_tracking_parameters = sl.PositionalTrackingParameters()
# If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
positional_tracking_parameters.set_as_static = True
zed.enable_positional_tracking(positional_tracking_parameters)

obj_param = sl.ObjectDetectionParameters()
obj_param.enable_body_fitting = True            # Smooth skeleton move
obj_param.enable_tracking = True                # Track people across images flow
obj_param.detection_model = sl.DETECTION_MODEL.HUMAN_BODY_FAST 
obj_param.body_format = sl.BODY_FORMAT.POSE_18  # Choose the BODY_FORMAT you wish to use

# Enable Object Detection module
zed.enable_object_detection(obj_param)

obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
obj_runtime_param.detection_confidence_threshold = 40

# Get ZED camera information
camera_info = zed.get_camera_information()

# 2D viewer utilities
display_resolution = sl.Resolution(min(camera_info.camera_resolution.width, 1280), min(camera_info.camera_resolution.height, 720))
image_scale = [display_resolution.width / camera_info.camera_resolution.width
                , display_resolution.height / camera_info.camera_resolution.height]

# Create OpenGL viewer
# viewer = gl.GLViewer()
# viewer.init(camera_info.calibration_parameters.left_cam, obj_param.enable_tracking,obj_param.body_format)

# Create ZED objects filled in the main loop
bodies = sl.Objects()
image = sl.Mat()
counter = 1
cnt_cos = 0

def talker():

    global wrist_num
    global cnt_cos
    global counter
    global bool_barrier

    pub = rospy.Publisher('wrist_pos', Point, queue_size=10)
    pub_found = rospy.Publisher('wrist_found', Point, queue_size=10)
    rospy.init_node('Wrist_tracker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    counter = 0
    pnt = Point(0.0,0.0,0.0)
    pnt_found = Point(0.0,0.0,0.0)
    pnt_ee = Point(0.0,0.0,0.0)
    pnt_command = Point(0.0,0.0,0.0)
    pnt_command_soft = Point(0.0,0.0,0.0)
    first = True
    while not rospy.is_shutdown():
        # Grab an image
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
            # Retrieve objects
            zed.retrieve_objects(bodies, obj_runtime_param)

            # Update GL view
            # If the Neural network found a body to track, then you can output the location in the camera frame of the body joints. 
            # wrist_num is the joint that you want. 7 is the left wrist, 4 is right wrist and 0 is the nose.
            obj_array = bodies.object_list
            if len(obj_array) > 0:
                first_object = bodies.object_list[0]
                keypoint = first_object.keypoint
                
                if(abs(keypoint[wrist_num][2])> 0.2 and abs(keypoint[wrist_num][2])< 3):
                    pnt_found.x = 1
                    if first:
                        pnt.x = keypoint[wrist_num][0]
                        pnt.y = keypoint[wrist_num][1]
                        pnt.z = keypoint[wrist_num][2]
                        first = False
                    else:
                        alpha = 0.8
                        pnt.x = pnt.x*(1-alpha) +  keypoint[wrist_num][0]*alpha
                        pnt.y = pnt.y*(1-alpha) +  keypoint[wrist_num][1]*alpha
                        pnt.z = pnt.z*(1-alpha) +  keypoint[wrist_num][2]*alpha
                    
                else:
                    pnt_found.x = 0
            else:
                pnt_found.x = 0


            
            # Update OCV view
            image_left_ocv = image.get_data()
            cv_viewer.render_2D(image_left_ocv,image_scale,bodies.object_list, obj_param.enable_tracking, obj_param.body_format)
            cv2.imshow("ZED | 2D View", image_left_ocv)
            #cv2.waitKey(1)

        
        counter = counter + 1
        # Change counter == N to adjust the publication of a new point. When counter == 1, it works at the FPS of the camera. In this case FPS = 30.
        if counter == 1:
            # Rotation to the arm frame and translated to arm origin. This is the distance from the camera frame to the robot frame.
            # pnt has the x, y and z position of the tracking point in the camera frame. We transform that point from camera frame to the manipulators frame.
            # This includes the rotation and translation
            pnt_ee.x = -pnt.z + 0.184
            pnt_ee.y = -pnt.x + 0.05
            pnt_ee.z =  pnt.y + 0.1534
            
            # pnt_ee is the goal position where we want to locate the end-effector. If we have the gripper, we need to add that distance to the point in space.
            # Since we are only doing a fixed orientation, you can hardcore the off-set of the gripper
            pnt_command.x = pnt_ee.x - 0.02 #0.134#-0.02#
            pnt_command.y = pnt_ee.y 
            pnt_command.z = pnt_ee.z + 0.203 #0.17

            rospy.loginfo("Unbounded Goal: %.2f, %.2f, %.2f",pnt_command.x, pnt_command.y, pnt_command.z)

            cnt_fails = 0
            # Since the tracking point can be anywhere in space, I constrait the point to live inside a box infront of the robot.
            # If you add the gripper, this limits should be adjusted.
            
            #Contraint in the x dimension
            if pnt_command.x<0.5:
                pnt_command.x = 0.5
                cnt_fails = cnt_fails + 1
            elif pnt_command.x>0.75:
                pnt_command.x = 0.75
                cnt_fails = cnt_fails + 1
                
            #Contraint in the y dimension
            if pnt_command.y<-0.2:
                pnt_command.y = -0.2
                cnt_fails = cnt_fails + 1
            elif pnt_command.y>0.2:
                pnt_command.y = 0.2
                cnt_fails = cnt_fails + 1
        
            #Contraint in the z dimension
            if pnt_command.z<0.25:
                pnt_command.z = 0.25
                cnt_fails = cnt_fails + 1
            elif pnt_command.z>0.75:
                pnt_command.z = 0.75
                cnt_fails = cnt_fails + 1
                

            # if cnt_fails > 1:
            #     pnt_command.x = 0.7
            #     pnt_command.y = 0.0
            #     pnt_command.z = 0.55


            # I run a low pass filter on the signal, to avoid noise from the measurement of the body pose and sudden fast moves.
            # alpha = 1 will be the fastest and alpha -> 0 will make the tracking point to move slower
            if first:
                pnt_command_soft.x = pnt_command.x
                pnt_command_soft.y = pnt_command.y
                pnt_command_soft.z = pnt_command.z
                first = False
            else:
                alpha = 0.4
                pnt_command_soft.x = pnt_command_soft.x*(1-alpha) +  pnt_command.x*alpha
                pnt_command_soft.y = pnt_command_soft.y*(1-alpha) +  pnt_command.y*alpha
                pnt_command_soft.z = pnt_command_soft.z*(1-alpha) +  pnt_command.z*alpha


            rospy.loginfo("Goal: %.2f, %.2f, %.2f",pnt_command_soft.x, pnt_command_soft.y, pnt_command_soft.z)
            # We publish the position of the wrist through LCM message. 
            msg.pnt = (pnt_command_soft.x, pnt_command_soft.y, pnt_command_soft.z)  # position values
            msg.motion_time = bool_barrier # double variable (unit: second), can be changed
            lc.publish(Main_channel, msg.encode()) # LCM publisher
            counter = 0
           
        pub.publish(pnt)

        # rospy.loginfo(pnt_found)
        pub_found.publish(pnt_found)
        rate.sleep()

        k = cv2.waitKey(10)
        # rospy.loginfo("selected %s", format(k))
        if k == 49: #number 1
            #Decrease the HoughCircles param_1 value
            wrist_num = 4 
            k = 0
            rospy.loginfo("Tracking Right Wrist")

        elif k == 50: #number 2
            #Increase the HoughCircles param_1 value
            wrist_num = 7 
            k = 0
            rospy.loginfo("Tracking Left Wrist")
        elif k == 51: #number 3
            #Increase the HoughCircles param_1 value
            wrist_num = 0
            k = 0
            rospy.loginfo("Tracking nose")
        elif k == 52:
            if bool_barrier == 1.0:
                bool_barrier = 0.0
                rospy.loginfo("Deactivate Barrier")
            else:
                bool_barrier = 1.0
                rospy.loginfo("Activate Barrier")


    
def my_shutdown_hook():
    # viewer.xit()

    image.free(sl.MEM.CPU)
    # Disable modules and close camera
    zed.disable_object_detection()
    zed.disable_positional_tracking()
    zed.close()
    rospy.loginfo("It's shutdown time!")



if __name__ == '__main__':
    try:
        rospy.on_shutdown(my_shutdown_hook)
        talker()
    except rospy.ROSInterruptException:
        pass