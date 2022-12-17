#! /usr/bin/env python3

import rospy
from rospy import Time 
from geometry_msgs.msg import Point

import cv2
import sys
# import pyzed.sl as sl
import numpy as np
#LCM
import lcm
from exlcm import example_t

wrist_num = 7

# Create LCM object
Main_channel = str("Hand_Position")
lc = lcm.LCM() # Creare a lcm object
msg = example_t()  # communication format

# Create a Camera object
# zed = sl.Camera()

# # Create a InitParameters object and set configuration parameters
# init_params = sl.InitParameters()
# init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
# init_params.coordinate_units = sl.UNIT.METER          # Set coordinate units
# init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
# init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

# # If applicable, use the SVO given as parameter
# # Otherwise use ZED live stream
# if len(sys.argv) == 2:
#     filepath = sys.argv[1]
#     print("Using SVO file: {0}".format(filepath))
#     init_params.svo_real_time_mode = True
#     init_params.set_from_svo_file(filepath)

# # Open the camera
# err = zed.open(init_params)
# if err != sl.ERROR_CODE.SUCCESS:
#     exit(1)

# # Enable Positional tracking (mandatory for object detection)
# positional_tracking_parameters = sl.PositionalTrackingParameters()
# # If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
# positional_tracking_parameters.set_as_static = True
# zed.enable_positional_tracking(positional_tracking_parameters)

# obj_param = sl.ObjectDetectionParameters()
# obj_param.enable_body_fitting = True            # Smooth skeleton move
# obj_param.enable_tracking = True                # Track people across images flow
# obj_param.detection_model = sl.DETECTION_MODEL.HUMAN_BODY_FAST 
# obj_param.body_format = sl.BODY_FORMAT.POSE_18  # Choose the BODY_FORMAT you wish to use

# # Enable Object Detection module
# zed.enable_object_detection(obj_param)

# obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
# obj_runtime_param.detection_confidence_threshold = 40

# # Get ZED camera information
# camera_info = zed.get_camera_information()

# # 2D viewer utilities
# display_resolution = sl.Resolution(min(camera_info.camera_resolution.width, 1280), min(camera_info.camera_resolution.height, 720))
# image_scale = [display_resolution.width / camera_info.camera_resolution.width
#                 , display_resolution.height / camera_info.camera_resolution.height]

# Create OpenGL viewer
# viewer = gl.GLViewer()
# viewer.init(camera_info.calibration_parameters.left_cam, obj_param.enable_tracking,obj_param.body_format)

# Create ZED objects filled in the main loop
# bodies = sl.Objects()
# image = sl.Mat()
counter = 1
cnt_cos = 0

def talker():

    global wrist_num
    global cnt_cos
    global counter

    pub = rospy.Publisher('wrist_pos', Point, queue_size=10)
    pub_found = rospy.Publisher('wrist_found', Point, queue_size=10)
    rospy.init_node('Wrist_tracker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    counter = 1
    angle = 0.1

    pnt_command = Point(0.8,0.0,0.4)
    first = True
    while not rospy.is_shutdown():
        # Grab an image
        # if zed.grab() == sl.ERROR_CODE.SUCCESS:
        #     # Retrieve left image
        #     zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
        #     # Retrieve objects
        #     zed.retrieve_objects(bodies, obj_runtime_param)

            # Update GL view
            #viewer.update_view(image, bodies) 
            # obj_array = bodies.object_list
            # if len(obj_array) > 0:
            #     first_object = bodies.object_list[0]
            #     keypoint = first_object.keypoint
                
            #     if(abs(keypoint[wrist_num][2])> 0.2 and abs(keypoint[wrist_num][2])< 3):
            #         pnt_found.x = 1
            #         if first:
            #             pnt.x = keypoint[wrist_num][0]
            #             pnt.y = keypoint[wrist_num][1]
            #             pnt.z = keypoint[wrist_num][2]
            #             first = False
            #         else:
            #             alpha = 0.2
            #             pnt.x = pnt.x*(1-alpha) +  keypoint[wrist_num][0]*alpha
            #             pnt.y = pnt.y*(1-alpha) +  keypoint[wrist_num][1]*alpha
            #             pnt.z = pnt.z*(1-alpha) +  keypoint[wrist_num][2]*alpha
                    
            #     else:
            #         pnt_found.x = 0
            # else:
            #     pnt_found.x = 0


            
            # # Update OCV view
            # image_left_ocv = image.get_data()
            # cv_viewer.render_2D(image_left_ocv,image_scale,bodies.object_list, obj_param.enable_tracking, obj_param.body_format)
            # cv2.imshow("ZED | 2D View", image_left_ocv)
            #cv2.waitKey(1)

        
        # rospy.loginfo("Goal: %.2f, %.2f, %.2f",pnt.x, pnt.y, pnt.z)
        counter = counter + 1
        # rospy.loginfo("Goal: %f",counter)
        if counter == 3:
            # Rotation to the arm frame and translated to arm origin. This is the distance from the camera frame to the robot frame.
            
            pnt_command = Point(0.6+0.1*np.sin(angle),0.02,0.28)
            rospy.loginfo("Goal: %.2f, %.2f, %.2f",pnt_command.x, pnt_command.y, pnt_command.z)
            msg.pnt = (pnt_command.x, pnt_command.y, pnt_command.z)  # position values
            
            # cnt_cos = cnt_cos + 0.1
            # msg.pnt = (0.75, 0.0, 0.45 + 0.15*np.cos(cnt_cos))  # position values
            msg.rpy = (0, 1.58, 0)
            msg.motion_time = 1.0 # double variable (unit: second), can be changed
            lc.publish(Main_channel, msg.encode()) # LCM publisher
            counter = 1
            angle = angle + 0.05
            
        
        
        # pub.publish(pnt)

        # rospy.loginfo(pnt_found)
        pub_found.publish(pnt_command)
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
        elif k == 51: #number 2
            #Increase the HoughCircles param_1 value
            wrist_num = 0
            k = 0
            rospy.loginfo("Tracking nose")



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass