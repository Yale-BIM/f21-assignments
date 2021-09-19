#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class LookForwardNode():
    """Node that makes Shutter's wrist_1_link rotate to make the Zed camera look forward."""

    def __init__(self):

        # Init the node
        rospy.init_node('generate_target', anonymous=True)

        # publish rate
        publish_rate = 1 # Hz

        # parameters for the joint of interest
        self.joint_name = "joint_4"
        self.desired_joint_position = 0
        self.joint_reached_desired_position = False

        # Publishers
        self.joint_pub = rospy.Publisher("/joint_4/command", Float64, queue_size=5)

        # Subscribers
        self.joints_sub = rospy.Subscriber("/joint_states", JointState, self.joints_callback, queue_size=5)


        rate = rospy.Rate(publish_rate)
        while not rospy.is_shutdown():

            if not self.joint_reached_desired_position:
                msg = Float64()
                msg.data = self.desired_joint_position
                self.joint_pub.publish(msg)

            else:
                break # end the execution of the node

            rate.sleep()


    def joints_callback(self, msg):


        # find the index of the joint 4
        idx = -1
        for i, name in enumerate(msg.name):
            if name == self.joint_name:
                idx = i

        # sanity check
        assert idx >= 0 and idx < 4, \
            "Failed to find the joint of interest ({})".format(joint)

        # current joint position
        joint_position = msg.position[idx]
        print("joint position: {}".format(joint_position))

        if np.fabs(joint_position - self.desired_joint_position) < 1e-2:
            self.joint_reached_desired_position = True
        else:
            self.joint_reached_desired_position = False

        print("reached? {}".format(self.joint_reached_desired_position))


if __name__ == '__main__':
    try:
        node = LookForwardNode()
    except rospy.ROSInterruptException:
        pass

    sys.exit(0)
