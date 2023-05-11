import rospy
import numpy as np
import cv2
import signal
import sys

from sensor_msgs.msg import Image
from std_msgs.msg import Header

from cv_bridge import CvBridge, CvBridgeError

class DetermineColor:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.color_pub = rospy.Publisher('/rotate_cmd', Header, queue_size=10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            msg = Header()
            msg = data.header
            msg.frame_id = '0'
            img = image.copy()

            # Convert image to HSV
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Define range of blue and red colors in HSV
            lower_blue = np.array([100, 50, 50])
            upper_blue = np.array([130, 255, 255])
            lower_red = np.array([-10, 100, 100])
            upper_red = np.array([10, 255, 255])

            # Threshold the HSV image to get only blue and red colors
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            mask_red = cv2.inRange(hsv, lower_red, upper_red)

            # Calculate the percentage of blue and red pixels in the image
            total_pixels = img.shape[0] * img.shape[1]
            blue_pixels = np.sum(mask_blue == 255)
            red_pixels = np.sum(mask_red == 255)
            unknown_pixels = total_pixels - blue_pixels - red_pixels
            blue_percent = blue_pixels / total_pixels
            red_percent = red_pixels / total_pixels
            unknown_percent = unknown_pixels / total_pixels

            # Determine the background color based on the majority color
            if blue_percent > red_percent and blue_percent > unknown_percent:
                background_color = "BLUE"
            elif red_percent > blue_percent and red_percent > unknown_percent:
                background_color = "RED"
            else:
                background_color = "unknown"
           


            # Set the frame ID based on the background color
            if background_color == "BLUE":
                msg.frame_id = '+1'
            elif background_color == "RED":
                msg.frame_id = '-1'
            else:
                msg.frame_id = '0'

            self.color_pub.publish(msg)

        except CvBridgeError as e:
            print(e)

        cv2.imshow('Image', image)
        cv2.waitKey(1)

    def shutdown(self):
        rospy.signal_shutdown("Shutdown")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('CompressedImages1', anonymous=False)
    detector = DetermineColor()

    signal.signal(signal.SIGINT, detector.shutdown)
    signal.signal(signal.SIGTERM, detector.shutdown)

    rospy.spin()
