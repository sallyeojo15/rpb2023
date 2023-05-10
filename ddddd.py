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

            pixel = img.reshape(-1, 3)  # Reshape image to a 2D array of pixels
            pixel = pixel.astype(np.float32)  # Normalize pixel values to the range [0, 1]

            unique_colors, color_counts = np.unique(pixel, axis=0, return_counts=True)
            max_count_index = np.argmax(color_counts)
            background = unique_colors[max_count_index]

            def get_color_name(b, g, r):
                if (b > 200) and (g < 200) and (r < 200):
                    color = "BLUE"
                elif (r > 200) and (g < 200) and (b < 200):
                    color = "RED"
                else:
                    color = "unknown"
                return color

            background_color = get_color_name(background[0], background[1], background[2])

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

