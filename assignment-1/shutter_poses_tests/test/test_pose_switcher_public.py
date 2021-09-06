#!/usr/bin/env python3
# Public tests for CPSC459/559 Assignment 2

PKG = "shutter_poses_tests"
NAME = 'test_pose_switcher_public'

import sys
import unittest
import time

import rospy
import rostest
from std_msgs.msg import Float64


class TestPoseSwitcher(unittest.TestCase):
    """
    Public tests for shutter_poses
    """

    def __init__(self, *args):
        super(TestPoseSwitcher, self).__init__(*args)
        self.sleep_time = 0.5
        self.subs = None
        self.joint_1 = None
        self.joint_2 = None
        self.joint_3 = None
        self.joint_4 = None

        rospy.init_node(NAME, anonymous=True)
        self.get_subs()

    def _callback(self, msg, k):
        if getattr(self, 'joint_{}'.format(k)) != msg.data:
            self.last_update = time.time()
            setattr(self, 'joint_{}'.format(k), msg.data)

    def check_none_joints(self, joints):
        none_joint = False
        for joint in joints:
            if joint is None:
                none_joint = True
        return none_joint

    def get_subs(self):
        if self.subs is not None:
            return self.subs

        self.subs = {}
        for k in range(1, 5):
            self.subs[k] = rospy.Subscriber('/joint_{}/command'.format(k), Float64, self._callback, k)

    def test_data_published(self):
        """Check that joint commands are being published. We should receive some data from all joints"""
        success = False
        timeout_t = time.time() + 10.0 #10 seconds
        while not rospy.is_shutdown() and not success and time.time() < timeout_t:
            time.sleep(self.sleep_time)
            joints = [self.joint_1, self.joint_2, self.joint_3, self.joint_4]

            if self.check_none_joints(joints):
                continue

            success = True
        self.assertTrue(success)

    def test_joint_1(self):
        """Check that joint 1 is close enough to zero, since this is true for all required poses"""
        joint = None
        timeout_t = time.time() + 5 #5 seconds
        while not rospy.is_shutdown() and joint is None and time.time() < timeout_t:
            joint = self.joint_1

        self.assertIsNotNone(joint, "Received no command for joint 1 in a lapse of 5 seconds")
        self.assertAlmostEqual(joint, 0.0, places=4, msg="The command for joint 1 was not 0.0 radians, "
                                                         "but {} radians. Check the value that is being "
                                                         "commanded".format(joint))


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPoseSwitcher, sys.argv)
