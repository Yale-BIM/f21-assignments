#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from shutter_track_target.msg import Observation
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def filter_image(cv_image, lower_hue_value, higher_hue_value):
    """
    Convert image to HSV color space and filter on a given hue range
    :param cv_image: input image
    :param lower_hue_value: min hue value
    :param higher_hue_value: max hue value
    :return: filtered image
    """

    # TODO. Remove the "return cv_image" line below and complete this function as indicated in the assignment's README.md file.
    return cv_image


def compute_keypoints_for_blobs(filtered_image):
    """
    Compute keypoints for blobs of a given color in the image.
    :param cv_image: input image
    :return: list of cv2.Keypoint object (see https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_feature_detectors.html?highlight=keypoint)
    """

    # TODO. Remove the "return []" line below and complete this function as indicated in the assignment's README.md file
    return []


class DetectTarget():
    """ROS node that detects visual targets of a given color"""

    def __init__(self):

        # Init the node
        rospy.init_node('detect_visual_target')

        # Node parameters
        self.lower_hue_value = rospy.get_param('~lower_hue_value', 100)   # default value of 100-140 range corresponds to blue color
        self.higher_hue_value = rospy.get_param('~higher_hue_value', 140)
        self.add_noise = rospy.get_param('~add_noise', False)             # add noise to the detected blob? (only set to true for the last part of the assignment!)

        # Publisher
        self.obs_pub = rospy.Publisher("/observation", Observation, queue_size=5)
        self.obs_image_pub = rospy.Publisher("/observation_image", Image, queue_size=5)

        # Subscriber
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=5)

        # OpenCV - ROS bridge
        self.bridge = CvBridge()

        # main thread just waits now..
        rospy.spin()


    def image_callback(self, image_msg):
        """
        Main routine in the node. Receives an RGB image, converts to HSV color space, thresholds based on hue (color),
        and identifies blobs in the thresholded image. The biggest blob is then published as a target observation.
        An image with the detected blob is also published for debugging purposes.
        :param image_msg: input Image message
        """

        # convert image to opencv
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # filter image in HSV space
        filtered_image = filter_image(cv_image, self.lower_hue_value, self.higher_hue_value)

        # detect blobs
        keypoints = compute_keypoints_for_blobs(filtered_image)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(filtered_image, keypoints, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # get the observation: the biggest keypoint
        kid = -1
        max_size = -1
        for i, k in enumerate(keypoints):
            if k.size > max_size:
                k.size = max_size
                kid = i

        if kid >= 0:

            # add noise to the observation?
            if self.add_noise:
                keypoints[kid].pt = (keypoints[kid].pt[0] + np.random.normal(loc=0.0, scale=10),
                                     keypoints[kid].pt[1] + np.random.normal(loc=0.0, scale=10))

            # draw the main target on the im_with_keypoints output image
            cv2.drawMarker(im_with_keypoints, (int(keypoints[kid].pt[0]), int(keypoints[kid].pt[1])), (0, 0, 255),
                           cv2.MARKER_CROSS, 100, 4)

            # publish the observation with the same stamp as the image that was received
            self.publish_observation(keypoints[kid].pt, image_msg.header)

        # publish the observation image
        try:
            obs_image_msg = self.bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        obs_image_msg.header = image_msg.header
        self.obs_image_pub.publish(obs_image_msg)


    def publish_observation(self, position_tuple, header):
        """
        Publish the observation location through the self.obs_pub topic
        :param position_tuple: (x,y) location for the target in the image
        :param header: header for the observation message
        """
        # TODO. Complete this function as indicated in the assignment's README.md file


if __name__ == '__main__':
    try:
        node = DetectTarget()
    except rospy.ROSInterruptException:
        pass

