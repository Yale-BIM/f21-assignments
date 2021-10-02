#! /usr/bin/env python3

import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class PlayVideoNode(object):
    """
    Node to play videos from file and publish to ROS
    """

    def __init__(self):
        # Init the node
        rospy.init_node('play_video_node')

        # Get video file
        self.video_file = rospy.get_param("~video_file")            # video file to play
        self.frame_rate = rospy.get_param("~frame_rate", 30)        # video frame rate (hz)
        self.show_video = rospy.get_param("~show_video", False)     # show video?
        rospy.loginfo('Video file: {}'.format(self.video_file))

        self.bridge = CvBridge()                           # OpenCV - ROS bridge

        # Publishers
        self.im_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=5)
        self.publish_frame()

    def publish_frame(self):
        """
        Create a VideoCapture object and read from input file
        """
        cap = cv2.VideoCapture(self.video_file)

        # Check if camera opened successfully
        if (cap.isOpened()== False):
            rospy.logerr("Error opening video stream or file")

        rate = rospy.Rate(self.frame_rate)

        # Read until video is completed
        while(cap.isOpened()):
            # Capture frame-by-frame
            ret, frame = cap.read()
            if ret == True:
                scale = 480./frame.shape[1]
                width = int(frame.shape[1] * scale)
                height = int(frame.shape[0] * scale)
                frame = cv2.resize(frame,(width, height))

                # convert CV image to Image message
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    # publish tracked path image
                    self.im_pub.publish(image_msg)
                    # show video?
                    if self.show_video:
                        cv2.imshow('frame', frame)
                        cv2.waitKey(1)
                        
                except CvBridgeError as e:
                    rospy.logerr('[ERROR] - {}'.format(e))
                    return

                rate.sleep()

            # Break the loop
            else:
                break

        # When everything done, release the video capture object
        cap.release()

        # Closes all the frames
        cv2.destroyAllWindows()


if __name__ == '__main__':

    try:
        node = PlayVideoNode()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS Interrupt.")
        pass

