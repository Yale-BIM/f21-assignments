#!/usr/bin/env python3
# Public tests for CPSC459/559 Assignment 3 - Part I

PKG = "shutter_track_target_public_tests"
NAME = 'test_bag_recording'

import sys
import unittest
import time
import subprocess

import rospy
import rostest

class TestBagRecording(unittest.TestCase):
    """
    Public tests for detect_visual_target.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestBagRecording, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)
        time.sleep(3)

    def check_topic(self, topic):
        out = subprocess.Popen(['rostopic', 'list'],
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
        stdout, stderr = out.communicate()

        if stderr is not None:
            print("Failed to run rostopic list. Error:\n{}".format(stderr.decode('utf-8')))
            return False

        stdout = stdout.decode('utf-8')
        for line in stdout.split('\n'):
            line = line.strip()
            if line in topic and line != '':
                return True

        return False

    def test_topic_tracked_image(self):
        """
        Test to check if /tracked_image topic is being published
        """
        found_topic = self.check_topic('/tracked_image')
        self.assertTrue(found_topic, "Did not find topic /tracked_image in playback of the ROS bag.")


    def test_topic_observation_image(self):
        """
        Test to check if /observation_image topic is being published
        """
        found_topic = self.check_topic('/observation_image')
        self.assertTrue(found_topic, "Did not find topic /observation_image in playback of the ROS bag.")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestBagRecording, sys.argv)
