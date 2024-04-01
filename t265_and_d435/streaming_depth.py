#!/usr/bin/env python
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

from unittest import skip
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
import time
from matplotlib import pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from scripts.msg import HSVParameters

#Callback function

def load_hsv_parameters():
    try:
        with open('/home/etienne/catkin_ws/src/scripts/src/HSV_Folder/hsv_parameters.txt', 'r') as f:
            values = f.readline().strip().split()
            if len(values) == 6:
                return [int(v) for v in values]
    except Exception as e:
        print("Error loading HSV parameters:", str(e))
    return None	


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    print("config1")
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    print("config2")

## Lower red color boundaries [H, S, V]
#lower1 = np.array([115, 83, 72],np.uint8)
#upper1 = np.array([179, 255, 255],np.uint8)

## HSV_data = load_hsv_parameters() # [Hmin Hmax Smin Smax Vmin Vmax]
#lower2 = np.array([0,119,0],np.uint8)
#upper2 = np.array([4,255,255],np.uint8)

# Lower red color boundaries [H, S, V]
lower1 = np.array([0, 70, 50],np.uint8)
upper1 = np.array([10, 255, 255],np.uint8)

# HSV_data = load_hsv_parameters() # [Hmin Hmax Smin Smax Vmin Vmax]
lower2 = np.array([170,70,50],np.uint8)
upper2 = np.array([180,255,255],np.uint8)

# Start streaming
pipeline.start(config)
pub = rospy.Publisher('target_position', PoseStamped, queue_size=10)
rospy.init_node('image_processing', anonymous=True)
rate = rospy.Rate(60) # 60hz
loop = True

try:
    while not rospy.is_shutdown():
	#time start
        start = time.time()
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
            #break

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        
        # Convert image from BGR to HSV
        color_imageHSV = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        # Create HSV Masks for Red
	mask1 = cv2.inRange(color_imageHSV, lower1, upper1)
	mask2 = cv2.inRange(color_imageHSV, lower2, upper2)
	mask = mask1 | mask2
	#mask = mask2

        # Merge masks and output
        output = cv2.bitwise_and(color_image, color_image, mask=mask)

        # Find contour
        ret,thresh = cv2.threshold(mask, 40, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) != 0:
            # Draw in blue the contours that were founded
            cv2.drawContours(output, contours, -1, 255, 3)
            # Find the biggest countour (c) by the area
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            # Draw the biggest contour (c) in green
            cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)
            i,j,k,l = x,y,w,h
        else:
            i,j,k,l = 0,0,0,0

        #If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap,output))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.imshow('Output', output)
        #cv2.imshow('Mask', mask)

	key = cv2.waitKey(1)
        if key == ord('q'):
            break

        if i != 0:
            hor = int(i+(k/2))
            ver = int(j+(l/2))
            zDepth = depth_frame.get_distance(hor,ver)

            if zDepth != 0 and zDepth <= 4.0:
                # Angle
                lateral_fov = np.radians(69.4)
                vertical_fov = np.radians(42.5)
                angle_lateral  = (hor-320)*lateral_fov/(320*2)
                angle_vertical = -(ver-240)*vertical_fov/(240*2)
                # Segmented Distance
                #dx1 = zDepth*np.cos(angle_lateral)
                #dx2 = zDepth*np.cos(angle_vertical)
                dy = zDepth*np.sin(angle_lateral)
                dx = zDepth*np.sin(angle_vertical)
                #dx  = (dx1+dx2)/2
                #dy  = (dy1+dy2)/2

                info_str = "[%s]Object found" % rospy.get_time()
                print("[%s]Object found" % rospy.get_time())          
                print("Distance to object: {}m".format(round(zDepth,2)))
                print("X distance:{}m".format(round(dx,2)))
                print("Y distances:{}m".format(round(dy,2)))
                print("Pixel coordinates: {},{}".format(hor,ver))
                print("Angle between camera and object: {},{}".format(round(np.degrees(angle_lateral),2),round(np.degrees(angle_vertical),2)))

                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.pose.position.x = dx
                msg.pose.position.y = dy
                pub.publish(msg)

                
            else:
                info_str = "[%s]Object not found" % rospy.get_time()  
        else:
            info_str = "[%s]Object not found" % rospy.get_time()
        

        rate.sleep()       
        cv2.waitKey(1)
        print("execution time = ", time.time() - start)
	if cv2.waitKey(1)&0xFF == ord('q'):
        	break


except rospy.ROSInterruptException or KeyboardInterrupt:
    pipeline.stop()
    loop = False
    pass
