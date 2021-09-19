#!/usr/bin/env python3
# Public tests for CPSC459/559 Assignment 2 - Part III

PKG = "shutter_lookat_public_tests"
NAME = 'test_virtual_camera'

import sys
import unittest
import time
import numpy as np

import rospy
import rostest
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from utils import inspect_rostopic_info, compute_import_path

import_path = compute_import_path('shutter_lookat', 'scripts')
sys.path.insert(1, import_path)
from virtual_camera import draw_image

class TestVirtualCamera(unittest.TestCase):
    """
    Public tests for virtual_camera.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestVirtualCamera, self).__init__(*args)

        self.camera_info_topic = "/virtual_camera/camera_info"           # camera info topic
        self.camera_image_topic = "/virtual_camera/image_raw"            # camera info topic
        self.target_topic = "/target"                                    # target topic

        self.info_success = False
        self.image_success = False

        rospy.init_node(NAME, anonymous=True)

        self.node_name = rospy.get_param("~node_name", default="virtual_camera_node")  # node name


    def test_node_connections(self):
        """
        Check the node's connections
        """
        success = False
        timeout_t = rospy.Time.now() + rospy.Duration.from_sec(5)  # 5 seconds in the future
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not success and rospy.Time.now() < timeout_t:
            if inspect_rostopic_info(self.node_name):
                success = True
            else:
                rate.sleep()

        self.assertTrue(success, "Failed to verify that the node {}.py "
                                 "subscribes to the {} topic".format(self.node_name, self.target_topic))

        print("Verified that the {}.py node subscribes to the {} topic".
              format(self.node_name, self.target_topic))

    def _camera_info_callback(self, msg):
        self.info_success = True
    
    def test_info_published(self):
        """
        Check that the information is being published on /virtual_camera/camera_info
        """
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=5)
        timeout_t = rospy.Time.now() + rospy.Duration.from_sec(10)  # 10 seconds in the future

        # wait patiently for a message
        while not rospy.is_shutdown() and rospy.Time.now() < timeout_t and not self.info_success:
            time.sleep(0.5)

        self.assertTrue(self.info_success, "Did not get any camera info message on {}.".format(self.camera_info_topic))
        print("Success. Got at least one camera info message through the {} topic!".format(self.camera_info_topic))

    def _image_callback(self, msg):
        self.image_success = True

    def test_image_published(self):
        """
        Check that the information is being published on /virtual_camera/camera_image
        """
        rospy.Subscriber(self.camera_image_topic, Image, self._image_callback, queue_size=5)
        timeout_t = rospy.Time.now() + rospy.Duration.from_sec(10)  # 10 seconds in the future

        # wait patiently for a message
        while not rospy.is_shutdown() and rospy.Time.now() < timeout_t and not self.image_success:
            time.sleep(0.1)

        self.assertTrue(self.image_success, "Did not get any image message on {}.".format(self.camera_image_topic))
        print("Success. Got at least one image message through the {} topic!".format(self.camera_image_topic))

    def test_draw_image(self):
        """
        Check the dimensions of the images output by draw_image()
        """
        x = -0.5 
        y = 0.5
        z = 1
        K = np.array([[1,1,1], [1,1,1], [1,1,1]])
        width = 256
        height = 128
        image = draw_image(x, y, z, K, width, height)

        self.assertIsNotNone(image, "The draw_image() function returned None instead of a valid image.")
        self.assertEqual(image.shape[0], height,
                         "The height of image was incorrect. Correct height: {}. Actual height: {}".
                         format(height, image.shape[0]))
        self.assertEqual(image.shape[1], width,
                         "The width of image was incorrect. Correct width: {}. Actual width: {}".
                         format(width, image.shape[1]))


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestVirtualCamera, sys.argv)
