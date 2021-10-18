#!/usr/bin/env python3
import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


# default volume for targets
dflt_x_min = 0.5
dflt_x_max = 3.5
dflt_y_min = -3.0
dflt_y_max = 3.0
dflt_z_min = 3.0
dflt_z_max = 0.0


def random_pose(x_min, x_max, y_min, y_max, z_min, z_max):
    """
    Generate random target within a 3D space
    :param x_min: minimum x
    :param x_max: maximum x
    :param y_min: minimum y
    :param y_max: maximum y
    :param z_min: minimum z
    :param z_max: maximum z
    :return PoseStamped with random pose
    """
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "base_footprint"
    pose_msg.pose.position.x = np.random.uniform(low=x_min, high=x_max, size=None)
    pose_msg.pose.position.y = np.random.uniform(low=y_min, high=y_max, size=None)
    pose_msg.pose.position.z = np.random.uniform(low=z_min, high=z_max, size=None)
    pose_msg.pose.orientation.w = 1.0
    return pose_msg


def generate_random_target():
    """
    Main function. Publishes the target at a constant frame rate.
    """

    # Init the node
    rospy.init_node('generate_random_target', anonymous=True)

    # Get ROS params
    x_min = rospy.get_param("~x_min", default=dflt_x_min)
    x_max = rospy.get_param("~x_max", default=dflt_x_max)
    y_min = rospy.get_param("~y_min", default=dflt_y_min)
    y_max = rospy.get_param("~y_max", default=dflt_y_max)
    z_min = rospy.get_param("~z_min", default=dflt_z_min)
    z_max = rospy.get_param("~z_max", default=dflt_z_max)
    publish_rate = rospy.get_param("~publish_rate", default=1)

    # Define publishers
    target_pub = rospy.Publisher('/target', PoseStamped, queue_size=5)
    marker_pub = rospy.Publisher('/target_marker', Marker, queue_size=5)

    # Publish the target at a constant ratetarget
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():

        # publish the location of the target as a PoseStamped
        pose_msg = random_pose(x_min, x_max, y_min, y_max, z_min, z_max)
        target_pub.publish(pose_msg)

        # publish a marker to visualize the target in RViz
        marker_msg = Marker()
        marker_msg.header = pose_msg.header
        marker_msg.action = Marker.ADD
        marker_msg.color.a = 0.5
        marker_msg.color.b = 1.0
        marker_msg.lifetime = rospy.Duration(1.0)
        marker_msg.id = 0
        marker_msg.ns = "target"
        marker_msg.type = Marker.SPHERE
        marker_msg.pose = pose_msg.pose
        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        marker_pub.publish(marker_msg)

        # sleep to keep the desired publishing rate
        rate.sleep()


if __name__ == '__main__':
    try:
        generate_random_target()
    except rospy.ROSInterruptException:
        pass
