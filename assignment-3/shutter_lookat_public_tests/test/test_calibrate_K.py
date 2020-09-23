#!/usr/bin/env python2.7
# Public tests for 559 Assignment 3 - Part V

PKG = "shutter_lookat_public_tests"
NAME = 'test_calibrate_K'

import sys
import unittest
import time
import numpy as np

import rospy
import rostest
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from utils import inspect_rostopic_info, compute_import_path

import_path = compute_import_path('calibration')
sys.path.insert(1, import_path)
from calibrate_K import compute_K

class TestCalibrateK(unittest.TestCase):
    """
    Public tests for calibrate_K.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestCalibrateK, self).__init__(*args)

        rospy.init_node(NAME, anonymous=True)
        self.data_file = rospy.get_param("~data_file", default="correspondences.txt")

    def test_compute_K(self):
        """
        Check calculation of K
        """
        K, error = compute_K(self.data_file)
        rospy.loginfo('[INFO] K: {}'.format(K.shape))
        self.assertTrue(K.shape==(3,3), "Shape of K is not (3,3)")
        self.assertTrue(error < np.inf, "error is not finite.")

        print("Verified that 'compute_K' returns 3x3 numpy matrix and non-infinite error")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestCalibrateK, sys.argv)
