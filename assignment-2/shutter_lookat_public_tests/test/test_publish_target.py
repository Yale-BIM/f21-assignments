#!/usr/bin/env python3
# Public tests for CPSC459/559 Assignment 2 - Part II

PKG = "shutter_lookat_public_tests"
NAME = 'test_publish_target'

import sys
import unittest
import tf2_ros

import rospy
import rostest

from utils import inspect_rostopic_info


class TestPublishTarget(unittest.TestCase):
    """
    Public tests for publish_target_relative_to_realsense_camera.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestPublishTarget, self).__init__(*args)

        self.node_name = "publish_target_relative_to_realsense_camera"   # name of the node when launched for the tests
        self.target_topic = "/target"                                    # target topic
        self.target_frame = "target"                                     # target frame

        rospy.init_node(NAME, anonymous=True)

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

        self.assertTrue(success, "Failed to verify that the node publish_target_relative_to_realsense_camera.py "
                                 "subscribes to the {} topic".format(self.target_topic))

        print("Verified that the publish_target_relative_to_realsense_camera.py node subscribes to the {} topic".
              format(self.target_topic))

    def test_frame_exists(self):
        """
        Check that the target frame exists in the tf tree
        """
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        t = None        # transform
        err = None      # error
        timeout_t = rospy.Time.now() + rospy.Duration.from_sec(10)  # 10 seconds in the future

        # wait patiently for a transform
        while not rospy.is_shutdown() and t is None and rospy.Time.now() < timeout_t:
            try:
                t = self.tf_buffer.lookup_transform("base_link",
                                                    self.target_frame,
                                                    rospy.Time(0),
                                                    rospy.Duration(1.0))  # wait for 1 second
            except tf2_ros.LookupException as e:
                err = e
                continue
            except tf2_ros.ConnectivityException as e:
                err = e
                continue
            except tf2_ros.ExtrapolationException as e:
                err = e
                continue

        if err is not None:
            err_str = "Got error: {}".format(err)
        else:
            err_str = ""

        self.assertIsNotNone(t, "Failed to find a transformation between base_link and {}.{}\nCheck how the node "
                                "is publishing the transform for the target.".format(self.target_frame, err_str))

        print("Success. Found the frame {} in the tf tree!".format(self.target_frame))


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPublishTarget, sys.argv)
