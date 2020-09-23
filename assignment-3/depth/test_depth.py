#!/usr/bin/env python2.7
# Public tests for CPSC459/559 Assignment 3 - Part VI

PKG = "shutter_lookat_public_tests"
NAME = 'test_depth'

import sys
import unittest
import time
import numpy as np

import rospy
import rostest
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from utils import inspect_rostopic_info, compute_import_path

import_path = compute_import_path('depth')
sys.path.insert(1, import_path)        
from process_images import compute_depth_from_gray_image, compute_depth_from_depth_image

class TestDepth(unittest.TestCase):
    """
    Public tests for process_images.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestDepth, self).__init__(*args)
        
        rospy.init_node(NAME, anonymous=True)
        
        self.images_file = rospy.get_param("~images_file", default="image.npz")
        self.object_height = rospy.get_param("~object_height", default=0.245)
        
        self.data = np.load(self.images_file)
        self.image_coordinates = [[0,0], [self.data['height'],self.data['width']]]
        
    def test_gray_depth(self):
        """
        Check calculation of depth from grayscale image
        """
        gray=self.data['gray']
        K=np.reshape(self.data['K'], (3,3))
        
        gray_depth = compute_depth_from_gray_image(gray, self.image_coordinates, K, self.object_height)
        self.assertIsInstance(gray_depth, float, "'compute_depth_from_gray_image' is not returning a float value. value: {}".format(gray_depth))
        self.assertTrue(gray_depth < np.inf, "'compute_depth_from_gray_image' result is not finite. value: {}".format(gray_depth))

        print("Verified that 'compute_depth_from_gray_image' returns non-infinite float")

    def test_depth_image(self):
        """
        Check calculation of depth from depth image
        """
        depth=self.data['depth']
        
        avg_depth = compute_depth_from_depth_image(depth, self.image_coordinates)
        self.assertIsInstance(avg_depth, float, "'compute_depth_from_depth_image' is not returning a float value. value: {}".format(avg_depth))
        self.assertTrue(avg_depth < np.inf, "'compute_depth_from_depth_image' result is not finite. value: {}".format(avg_depth))

        print("Verified that 'compute_depth_from_depth_image' returns non-infinite float")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestDepth, sys.argv)
