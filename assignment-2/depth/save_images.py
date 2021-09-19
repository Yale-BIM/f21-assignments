#!/usr/bin/env python3
# Simple script to save color and depth images captured by a realsense camera

import sys
import rospy
import cv_bridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2

def callback(image, camera_info, depth):
    """
    Process data and quit.
    :param image: image message
    :param camera_info: camera info message
    :param depth: depth image message
    """
    print("Got data at {}".format(rospy.Time.now()))

    bridge = cv_bridge.CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
    cv_depth = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")

    # convert depth from mm to meters
    cv_depth = cv_depth.astype(float) / 1000.0 

    # convert image to grayscale
    cv_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    # save images
    np.savez('images.npz',
             gray=cv_gray,
             depth=cv_depth,
             width=camera_info.width,
             height=camera_info.height,
             K=camera_info.K)
    
    # once an image is saved, quit!
    rospy.signal_shutdown("We are done.")


# create subscriptions
rospy.init_node("CaptureImages")
image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)

ts = message_filters.TimeSynchronizer([image_sub, info_sub, depth_sub], 10)
ts.registerCallback(callback)

# wait until we get data
rospy.spin()
