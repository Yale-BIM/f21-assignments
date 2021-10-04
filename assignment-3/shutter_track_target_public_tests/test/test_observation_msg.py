#!/usr/bin/env python3
# Public tests for CPSC459/559 Assignment 3 - Part I

PKG = "shutter_track_target_public_tests"
NAME = 'test_observation_message'

import sys
import unittest
import subprocess

import rospy
import rostest

class TestObservationMsg(unittest.TestCase):
    """
    Public tests for publish_target_relative_to_realsense_camera.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        super(TestObservationMsg, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)
        self.msg_name = 'shutter_track_target/Observation'

    def test_observation_msg(self):
        """
        Helper function to check a node's connections
        :return: True if the node subscribes to the target_topic
        """
        out = subprocess.Popen(['rosmsg', 'show', self.msg_name],
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
        stdout, stderr = out.communicate()

        if stderr is not None:
            msg = "Failed to run rosmsg show. Error:\n{}".format(stderr.decode('utf-8'))
            self.assertTrue(False, msg)
        
        stdout = stdout.decode('utf-8')
        headers = ['std_msgs/Header header','float64 x', 'float64 y'] 
        found_headers = []

        found_msg = True
        for line in stdout.split('\n'):
            line = line.strip()
            # rospy.logwarn('\n\n\n{}\n\n\n'.format(line))  # print output of rosnode info
            if line not in headers:
                if line not in ['', 'uint32 seq', 'time stamp', 'string frame_id']:
                    found_msg = False
            else:
                found_headers.append(line)

        self.assertTrue(found_msg and len(found_headers)==len(headers), "Could not verfy correct Observation message. Msg received: {}".format(stdout))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestObservationMsg, sys.argv)
