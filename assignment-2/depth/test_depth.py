#!/usr/bin/env python3
# Public tests for CPSC459/559 Assignment 2 - Part VI

import sys
import unittest
import numpy as np
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

        self.object_height = 0.245
        self.image_coordinates = [[0, 0], [80, 60]]
        
    def test_gray_depth(self):
        """
        Check calculation of depth from grayscale image
        """
        gray = np.full((480, 640), 181, dtype=np.uint8)
        K = np.array([612.65332031,   0., 311.68063354,   0., 612.71502686, 247.28131104,   0.,   0., 1.])
        K = np.reshape(K, (3, 3))
        
        gray_depth = compute_depth_from_gray_image(gray, self.image_coordinates, K, self.object_height)
        self.assertIsInstance(gray_depth, float,
                              "The compute_depth_from_gray_image() function is not returning a float value. Output: {}"
                              .format(gray_depth))
        self.assertFalse(np.isinf(gray_depth),
                         "The output of compute_depth_from_gray_image() is not finite. Output: {}"
                         .format(gray_depth))

        print("Verified that 'compute_depth_from_gray_image' returns non-infinite float")

    def test_depth_image(self):
        """
        Check calculation of depth from depth image
        """
        depth = np.full((480, 640), 0.853, dtype='float64')
        
        avg_depth = compute_depth_from_depth_image(depth, self.image_coordinates)
        self.assertIsInstance(avg_depth, float,
                              "The compute_depth_from_depth_image() function is not returning a float value. Output: {}"
                              .format(avg_depth))
        self.assertFalse(np.isinf(avg_depth),
                         "The output of the compute_depth_from_depth_image() function is not finite. Output: {}"
                         .format(avg_depth))

        print("Verified that 'compute_depth_from_depth_image' returns non-infinite float")
