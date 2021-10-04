#!/usr/bin/env python3
# Public tests for CPSC459/559 Assignment 3 - Part I

PKG = "shutter_track_target_public_tests"
NAME = 'test_detect_visual_target'

import sys
import unittest
import numpy as np
import time
from cv_bridge import CvBridge, CvBridgeError

import rospy
import rostest
from sensor_msgs.msg import Image
from shutter_track_target.msg import Observation

from utils import compute_import_path

import_path = compute_import_path('shutter_track_target', 'scripts')
sys.path.insert(1, import_path)
from detect_visual_target import filter_image, compute_keypoints_for_blobs

class TestDetectVisualTarget(unittest.TestCase):
    """
    Public tests for detect_visual_target.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestDetectVisualTarget, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)

        self.node_name = 'detect_visual_target'
        self.observation = None
        self.image = None

        # Subscribers
        self.sub = rospy.Subscriber('/observation', Observation, self.publish_img_callback, queue_size=5)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=5)

        # OpenCV - ROS bridge
        self.bridge = CvBridge()

        # Define image filtering values
        # Default value of 100-140 range corresponds to blue color
        self.lower_hue_value = rospy.get_param('~lower_hue_value', 100)
        self.higher_hue_value = rospy.get_param('~higher_hue_value', 140)

    def image_callback(self, msg):
        """
        Callback for reading images from bag
        """
        # convert image to opencv
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    def test_a_filter_image(self):
        """
        Check that filter image is returning cv2 image
        """
        timeout_t = time.time() + 10  # 10 seconds
        while not rospy.is_shutdown() and self.image is None and time.time() < timeout_t:
            pass
        self.assertIsNotNone(self.image, "No images published in 10 seconds. Check the playback of your bag. "
                                         "Is the left-seq1.bag in the data folder?")

        image = filter_image(self.image, self.lower_hue_value, self.higher_hue_value)
        self.assertIsInstance(image, np.ndarray, "filter_image is not returning the right type of object.\
                                                  Expected: np.ndarray. Received: {}".format(type(image)))
        self.assertEqual(len(image.shape), 2, "Image is not 2-dimensional. Expected dimensions: (height, width).\
                                               Actual dimensions: {}".format(image.shape))

    def test_b_compute_keypoints_for_blobs(self):
        """
        Check that compute_keypoints_for_blobs is computing keypoints for image
        """
        keypoints = []
        timeout_t = time.time() + 10  # 10 seconds
        while not rospy.is_shutdown() and time.time() < timeout_t:
            if self.image is not None:
                filtered_image = filter_image(self.image, self.lower_hue_value, self.higher_hue_value)
                keypoints = compute_keypoints_for_blobs(filtered_image)
                if len(keypoints) > 0:
                    break
        self.assertIsNotNone(self.image, "No images published in 10 seconds. Check the playback of your bag. "
                                         "Is the left-seq1.bag in the data folder?")
        self.assertTrue(len(keypoints)>0, "Keypoints is None. The compute_keypoints_for_blobs should return a non-empty list of keypoints"
                                          "(see https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_feature_detectors.html?highlight=keypoint)")

    def publish_img_callback(self, msg):
        """
        Callback to check for image publication
        """
        self.observation = msg

    def test_c_publish_observation(self):
        """
        Check that observations are being published
        """
        observation = None
        timeout_t = time.time() + 5  # 5 seconds
        while not rospy.is_shutdown() and observation is None and time.time() < timeout_t:
            observation = self.observation

        self.assertIsNotNone(observation, "Waited for 5 seconds... but received no messages through the /observation "
                                          "topic")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestDetectVisualTarget, sys.argv)
