#!/usr/bin/env python3
# Public tests for CPSC459/559 Assignment 2 - Part IV

PKG = "shutter_lookat_public_tests"
NAME = 'test_fancy_virtual_camera'

import sys
import numpy as np

import rospy
import rostest

from utils import inspect_rostopic_info, compute_import_path

import_path = compute_import_path('shutter_lookat', 'scripts')
sys.path.insert(1, import_path)
from fancy_virtual_camera import draw_image, compute_q, compute_rotation_axis, rotate_q

from test_virtual_camera import TestVirtualCamera

class TestFancyVirtualCamera(TestVirtualCamera):
    """
    Public tests for fancy_virtual_camera.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestFancyVirtualCamera, self).__init__(*args)

        self.node_name = "fancy_virtual_camera_node"             # name of the node when launched for the tests
        self.radius = rospy.get_param("~radius", default=0.05)   # ball radius

    def test_compute_q(self):
        """
        Test compute_q function - IV-1
        """
        pt = np.array([-0.06777152, -0.14803813, 1.2686999])
        q = np.array([0., 0.04966305, 0.00579493])
        q_computed = compute_q(pt, self.radius)
        q_computed = np.reshape(q_computed, (3))
        self.assertTrue(np.allclose(q, q_computed),
                        "Computed q vector does not match expected value of q. Correct q: {}. Actual q {}".
                        format(q, q_computed))
        print("Computed q vector matched expected value.")

    def test_compute_rotation_axis(self):
        """
        Test computation of rotation axis - IV-3
        """
        pt = np.array([-0.06777152, -0.14803813, 1.2686999])
        rotation_axis = compute_rotation_axis(pt)

        self.assertEqual(len(rotation_axis), 3,
                         msg="Rotation axis should be 3-dimensional. You returned a vector of shape {}".format(
                             rotation_axis.shape))
        print("Validated that rotation axis is a 3-dimensional vector.")

    def test_rotate_q(self):
        """
        Test rotation of q - IV-4
        """
        pt = np.array([-0.06777152, -0.14803813, 1.2686999])
        q = np.array([0., 0.04966305, 0.00579493])
        rotation_axis = compute_rotation_axis(pt)
        angle = 30.

        rotated_q = rotate_q(q, rotation_axis, angle)

        self.assertEqual(len(rotated_q), 3,
                         msg="Rotation q should be 3-dimensional. You returned a vector of shape {}".format(
                             q.shape))

        print("Validated that the rotated vector is 3-dimensional.")


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestFancyVirtualCamera, sys.argv)
