import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class DetermineColor:
  def __init__(self):
    self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
    self.color_pub = rospy.Publisher('/rotate_cmd', Header, queue_size=10)
    self.bridge = CvBridge()
    self.count = 0
  

  def callback(self, data):
    try:
      image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
      msg = Header()
      msg = data.header
      msg.frame_id = '0'
      img=cv2.imread(image)
      img_horizental, img_vertical = img.shape[0], img.shape[1]
      pixel=[]
      for i in range(img_horizental):
        for j in range(img_vertical):
          pixel.append(img[i,j])
    
      pixel_set=set(pixel)
      
      max_color ="0"
      max_count=0
      for i in pixel_set:
        if max_count<pixel.count(i):
          max_count=pixel.count(i)
          max_color=i
      
      def get_color_name(b,g,r) :
        if (b>200) & (g<200) & (r<200) :
          color="BLUE"
        elif (r>200) & (g<200) & (b<200) :
          color="RED"
        else :
          color="unknown"
        return color
        
      background_color=get_color_name(max_color[0],max_color[1],max_color[2])

      if background_color=="BLUE":
        msg.frame_id='+1'
      elif background_color=="RED":
        msg.frame_id='-1'
      else:
        msg.frame_id='0'

      self.color_pub.publish(msg)

    except CvBridgeError as e:
      print(e)
    cv2.imshow('Image',image)
    cv2.waitKey(1)

  def rospy_shutdown(self, signal, frame):
    rospy.signal_shutdown("shut down")
    sys.exit(0)

      
if __name__ == '__main__':
  detector = DetermineColor()
  rospy.init_node('CompressedImages1', anonymous=False)
  rospy.spin()
