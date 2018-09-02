#!/usr/bin/env python
import sys
import rospy
import copy

import tf2_ros

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class LookAtTargetNode():

    def __init__(self):
        rospy.init_node('lookat_target')

        # tf
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.latest_joint_states = None

        # joint publishers
        self.joint1_pub = rospy.Publisher('/joint_1/command', Float64, queue_size=5)
        self.joint3_pub = rospy.Publisher('/joint_3/command', Float64, queue_size=5)

        # joint subscriber
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=5)

        # main loop
        while not rospy.is_shutdown():

            if self.latest_joint_states is None:
                # Wait until we get some information about the current state of the robot
                rospy.sleep(0.5)
                continue

            # copy the joint states
            current_joint_states = copy.deepcopy(self.latest_joint_states)

            # compute yaw and send command to joint 1
            ok = self.solve_for_yaw(current_joint_states)
            if not ok:
                rospy.sleep(0.5)
                continue

            # compute pitch and send command to joint 3
            ok = self.solve_for_pitch(current_joint_states)
            if not ok:
                rospy.sleep(0.5)
                continue

            rospy.sleep(1/30.0)


    def joint_states_callback(self, msg):
        """
        Get current joint states from robot driver. Save them as a dictionary for easy access.
        :param msg: joint states message
        """
        states_dict = {}
        for i, name in enumerate(msg.name):
            states_dict[name] = msg.position[i]
        self.latest_joint_states = states_dict


    def get_transform(self, parent_link, child_link, time=rospy.Time(0)):
        """
        Query the transform from tf that converts points from a child to a parent link
        :param tf_buffer: tf buffer
        :param parent_link: name of resulting frame
        :param child_link: name of original frame
        :param time: time at which we want to query the transform
        :return: transform
        """
        try:
            transform = self.tf_buffer.lookup_transform(parent_link,
                                                        child_link,
                                                        time,
                                                        rospy.Duration(1.0))  # wait for 1 second
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return None

        return transform


    def solve_for_yaw(self, current_joint_states):
        """
        Solve for the yaw angle (joint 1) that we want the robot to have and send command to robot driver
        :param current_joint_states: dictionary with current join values
        :return: True if everything went well; False otherwise.
        """

        # complete this function as part of the assignment

        return True


    def solve_for_pitch(self, current_joint_states):
        """
        Solve for the pitch angle (joint 3) that we want the robot to have and send command to robot driver
        :param current_joint_states:
        :return: True if everything went well; False otherwise.
        """

        # complete this function as part of the assignment

        return True

if __name__ == '__main__':
    try:
        node = LookAtTargetNode()
    except rospy.ROSInterruptException:
        pass