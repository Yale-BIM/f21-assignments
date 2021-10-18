#!/usr/bin/env python3
# Public tests to evaluate behavior cloning model

PKG = "shutter_behavior_cloning"
NAME = 'test_eval_policy'

import sys
import os
import numpy as np
import unittest
import rospy
import rospkg
import rostest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from threading import Lock
from urdf_parser_py.urdf import URDF
import tf2_ros
from visualization_msgs.msg import Marker


r = rospkg.RosPack()
shutter_bc = r.get_path("shutter_behavior_cloning")
sys.path.insert(1, os.path.join(shutter_bc, "src"))
from generate_random_target import dflt_x_min, dflt_x_max, dflt_y_min, dflt_y_max, dflt_z_min, dflt_z_max
from expert_opt import ExpertNode, transform_msg_to_T, make_joint_rotation, target_in_camera_frame


def make_pose(x, y, z, frame_id="base_footprint"):
    """
    Generate random target within a 3D space
    :param x: x coordinate
    :param y: y coordinate
    :param z: z coordinate
    :param frame_id: frame id
    :return PoseStamped with (x,y,z) position
    """
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    pose_msg.pose.orientation.w = 1.0
    return pose_msg


def move_angle_to_pi_range(a):
    """
    Move angle to [pi, -pi) range
    :param a: angle (in radians)
    :return: radians
    """
    result = a
    if a >= 2 * np.pi:
        result = a - np.floor(a / (2 * np.pi)) * 2 * np.pi
    elif a <= -2 * np.pi:
        result = a + np.floor(-a / (2 * np.pi)) * 2 * np.pi

    if result <= -np.pi:
        result = 2 * np.pi + result
    elif result > np.pi:
        result = -2 * np.pi + result

    return result


def min_angle_diff(a1, a2):
    """
    Compute minimum angle difference (a1 - a2)
    :param a1: angle (radians)
    :param a2: angle (radians)
    :return: a1 - a2
    """
    a1 = move_angle_to_pi_range(a1)
    a2 = move_angle_to_pi_range(a2)
    d = a1 - a2
    if d > np.pi:
        d = d - 2 * np.pi
    elif d < -np.pi:
        d = d + 2 * np.pi
    return d


def check_joint_moved(joints_list, threshold=1e-5):
    """
    Check if a joint changed position
    :param joints_list: list of joint angles
    :param threshold: threshold for comparison in radians
    """
    if len(joints_list) < 2:
        return False

    # compare each consecutive set of joints in the list
    j = joints_list[0]
    for i in range(1, len(joints_list)):
        current_joint = joints_list[i]
        diff = min_angle_diff(j, current_joint)
        if np.fabs(diff) > threshold:
            return True
        j = current_joint

    return False


class TestPolicy(unittest.TestCase, ExpertNode):
    """
    Public tests for run_policy.py
    """

    def __init__(self, *args):
        """
        Constructor
        """
        unittest.TestCase.__init__(self, *args)
        rospy.init_node(NAME, anonymous=True)
        rospy.on_shutdown(self.cleanup)

        # params
        self.base_link = rospy.get_param("~base_link", "base_link")
        self.biceps_link = rospy.get_param("~biceps_link", "biceps_link")
        self.camera_link = rospy.get_param("~camera_link", "camera_color_optical_frame")
        self.max_wait = rospy.get_param("~max_wait", 3)  # in seconds
        self.publish_marker = rospy.get_param("~publish_marker", True)
        self.output_file = rospy.get_param("~output_file", "test_policy_output.txt")

        # prepare output file
        self.fid = open(self.output_file, 'w')

        # get robot model
        self.robot = URDF.from_parameter_server()
        self.move = True

        # make positions repeatable
        np.random.seed(0)

        # generate samples
        num_samples = 25
        self.positions = np.column_stack((np.random.uniform(low=dflt_x_min, high=dflt_x_max, size=(num_samples, 1)),
                                          np.random.uniform(low=dflt_y_min, high=dflt_y_max, size=(num_samples, 1)),
                                          np.random.uniform(low=dflt_z_min, high=dflt_z_max, size=(num_samples, 1))))

        # buffer for joint positions
        self.record_joints = False
        self.joints_1 = []
        self.joints_3 = []
        self.joints_mutex = Lock()

        # error codes
        self.err = {"NO_EXPERT_SOLUTION": "Failed to compute the expert's solution",
                    "NO_JOINT_STATES": "Failed to get new joint states for the robot.",
                    "NO_MOTION": "The robot did not move."}

        # setup node connections
        rospy.Subscriber('/joint_states', JointState, self.joints_callback, queue_size=5)

        # tf subscriber
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # wait for key joints
        try:
            self.tf_buffer.lookup_transform("base_link", "base_footprint", rospy.Time(0), rospy.Duration(10.0))
            self.tf_buffer.lookup_transform("camera_color_optical_frame", "base_link", rospy.Time(0), rospy.Duration(10.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.fail("Failed to get transforms in the robot body.")

        self.target_pub = rospy.Publisher('/target', PoseStamped, queue_size=5)
        self.marker_pub = rospy.Publisher('/target_marker', Marker, queue_size=5)

        rospy.loginfo("Waiting for a few seconds so that all other nodes start...")
        rospy.sleep(3)

    def joints_callback(self, msg):
        """
        Joints callback
        :param msg: joint state
        """
        joint1_idx = -1
        joint3_idx = -1
        for i in range(len(msg.name)):
            if msg.name[i] == 'joint_1':
                joint1_idx = i
            elif msg.name[i] == 'joint_3':
                joint3_idx = i
        assert joint1_idx >= 0 and joint3_idx > 0, \
            "Missing joints from joint state! joint1 = {}, joint3 = {}".format(joint1_idx, joint3_idx)

        with self.joints_mutex:
            if not self.record_joints:  # empty the queue so that it's only size 1; otherwise, grow the queue
                self.joints_1 = []
                self.joints_3 = []

            # store joint
            self.joints_1.append(msg.position[joint1_idx])
            self.joints_3.append(msg.position[joint3_idx])

    def publish_marker_msg(self, pose_msg):
        # publish a marker to visualize the target in RViz
        marker_msg = Marker()
        marker_msg.header = pose_msg.header
        marker_msg.action = Marker.ADD
        marker_msg.color.a = 0.5
        marker_msg.color.b = 1.0
        marker_msg.lifetime = rospy.Duration(20.0)
        marker_msg.id = 0
        marker_msg.ns = "target"
        marker_msg.type = Marker.SPHERE
        marker_msg.pose = pose_msg.pose
        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        self.marker_pub.publish(marker_msg)

    def publish_sample(self, x, y, z):
        """
        Helper function to publish a sample an evaluate the motion of the robot
        :param x: x coordinate
        :param y: y coordinate
        :param z: z coordinate
        :return: abs diff j1, abs diff j2, str error code
        """
        # ensure we are not recording joints
        with self.joints_mutex:
            self.record_joints = False

        # make target
        pose_msg = make_pose(x, y, z)

        # get solution from expert
        self.joint1 = self.joints_1[-1]
        self.joint3 = self.joints_3[-1]
        solution = self.compute_joints_position(pose_msg)
        if solution is None:
            rospy.logerr("Failed to compute expert's solution for target {}".format((x, y, z)))
            return None, None, self.err['NO_EXPERT_SOLUTION']
        j1, j3 = solution

        # publish new target
        self.target_pub.publish(pose_msg)
        if self.publish_marker:
            self.publish_marker_msg(pose_msg)

        # start saving joints
        with self.joints_mutex:
            self.record_joints = True

        # wait until for max duration for the robot to reach a new pose
        current_time = rospy.Time.now()
        end_time = current_time + rospy.Duration.from_sec(self.max_wait)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            rospy.sleep(1)  # sleep for a second

        # analyze result... Did the robot reach the expert's solution?
        if len(self.joints_1) < 1 or len(self.joints_3) < 1:
            return None, None, self.err['NO_JOINT_STATES']

        # j1_moved = check_joint_moved(self.joints_1)
        # j3_moved = check_joint_moved(self.joints_3)

        diff1 = np.fabs(min_angle_diff(j1, self.joints_1[-1]))
        diff3 = np.fabs(min_angle_diff(j3, self.joints_3[-1]))

        return diff1, diff3, None

    def test_reaching_targets(self, max_ang_difference=0.00872665):
        """
        Test reaching the targets
        :param max_ang_difference: maximum angular difference to consider a trial successful (in radians, dflt: 0.5 deg)
        """
        headers = ["TRIAL", "X", "Y", "Z", "DIFFJ1", "DIFFJ3", "ACCEPTABLE"]
        self.fid.write("".join([x.ljust(12) for x in headers]) + "\n")

        for i in range(self.positions.shape[0]):
            p = self.positions[i]
            d1, d3, err = self.publish_sample(p[0], p[1], p[2])
            self.assertIsNone(err, "An error occurred while testing the model: {}".format(err))

            if d1 < max_ang_difference and d3 < max_ang_difference:
                acceptable = 1
            else:
                acceptable = 0

            # print info...
            table = ["%4d" % i,
                     "%.3f" % p[0],
                     "%.3f" % p[1],
                     "%.3f" % p[2],
                     "%.5f" % d1,
                     "%.5f" % d3,
                     "%1d" % acceptable]
            self.fid.write("".join([x.ljust(12) for x in table]) + "\n")
            self.fid.flush()


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPolicy, sys.argv)



