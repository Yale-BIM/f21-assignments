#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from shutter_lookat.msg import Target


def project_3D_point(x, y, z, K):
    """
    Project 3D point [x, y, z]^T in camera frame onto the camera image using the intrinsic parameters K
    :param x: x coordinate of the 3D point (in the camera frame)
    :param y: y coordinate of the 3D point (in the camera frame)
    :param z: z coordinate of the 3D point (in the camera frame)
    :param K: 3x3 numpy array with intrinsic camera parameters
    :return: tuple with 2D coordinates of the projected 3D point on the camera plane
    """
    # complete this function....
    pass  # change this pass for an appropriate return statement, e.g., return pixel_x, pixel_y


def draw_image(x, y, z, K, width, height, **kwargs):
    """
    Project the target and create a virtual camera image
    :param x: x coordinate of the 3D point (in the camera frame)
    :param y: y coordinate of the 3D point (in the camera frame)
    :param z: z coordinate of the 3D point (in the camera frame)
    :param K: 3x3 numpy array with intrinsic camera parameters
    :param width: desired image width
    :param height: desired image height
    :param kwargs: extra parameters for the function (used in Part IV of the assignment)
    :return: color image as a numpy array. The image should have dimensions height x width x 3 and be of type uint8.
    """

    # NOTE: You should project the position of the moving object on the camera image using
    # the project_3D_point() function. You need to complete it as part of this assignment.

    pass  # change this pass for an appropriate return statement, e.g., return image (where image is a numpy array)


class VirtualCameraNode:
    """
    Node for Part III for the assignment 1. Renders an image from a virtual camera.
    """

    def __init__(self):
        """
        Constructor
        """

        # Init the node
        rospy.init_node('virtual_camera')

        # Complete this constructor with instance variables that you need for your code and any
        # additional publishers or subscribers...


        # finally, start receiving target messages in your node...
        rospy.Subscriber('target', Target, self.target_callback, queue_size=5)
        rospy.spin()


    def target_callback(self, target_msg):
        """
        Target callback
        :param target_msg: target message
        """

        # Convert target message to "camera_color_optical_frame" frame to get the target's x,y,z coordinates
        # relative to the camera...



        # Draw the camera image. Use the draw_image(x, y, z, K, width, height) function to this end....



        # Publish the resulting image....



        pass  # remove this pass statement when you implement your node logic.


if __name__ == '__main__':
    try:
        node = VirtualCameraNode()
    except rospy.ROSInterruptException:
        pass
