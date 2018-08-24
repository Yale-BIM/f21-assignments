#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import Vector3Stamped

# Add imports here...



class SimulatedObject(object):
    """
    Simulated object that moves on a circular path in front of the robot.
    The path is contained in a plane parallel to the y-z plane (i.e., x is constant for all points in the path).
    """

    def __init__(self):
        """
        Constructor
        """
        self.x = 1.5                    # constant x coordinate for the object
        self.center_y = 0.0             # y coordinate for the center of the object's path
        self.center_z = 0.25            # z coordinate for the center of the object's path
        self.angle = 0.0                # current angle for the object in its circular path (relative to the y axis)
        self.radius = 0.1               # radius of the object's circular path
        self.frame = "base_footprint"   # frame in which the coordinates of the object are computed

    def step(self, publish_rate):
        """
        Update the position of the target based on the publishing rate of the node
        :param publish_rate: node's publish rate
        """
        self.angle += 2.0 * np.pi / (10.0 * publish_rate)  # 1 full revolution in 10 secs


# Publishing rate for the node
publish_rate = 10


def generate_target():
    """
    Main function. Publishes the target at a constant frame rate.
    """
    global target_angle

    # Init the node
    rospy.init_node('generate_target')

    # Define publishers
    vector_pub = rospy.Publisher('/target', Vector3Stamped, queue_size=5)

    # Define tf2 transform broadcaster here...

    # Create the simulated object
    object = SimulatedObject()

    # Publish the target at a constant ratetarget
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():

        # publish the location of the target as a Vector3Stamped
        vector_msg = Vector3Stamped()
        vector_msg.header.stamp = rospy.Time.now()
        vector_msg.header.frame_id = object.frame
        vector_msg.vector.x = object.x
        vector_msg.vector.y = object.center_y + np.sin(object.angle)*object.radius
        vector_msg.vector.z = object.center_z + np.cos(object.angle)*object.radius
        vector_pub.publish(vector_msg)

        # publish a tf frame for the marker here...


        # update the simulated object state
        object.step(publish_rate)

        # sleep to keep the desired publishing rate
        rate.sleep()


if __name__ == '__main__':
    try:
        generate_target()
    except rospy.ROSInterruptException:
        pass
