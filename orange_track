#!/usr/bin/env python
import roslib
roslib.load_manifest('processing')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_converter/output_video",Image,queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/hd/image_color",Image,self.callback,queue_size=1)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    lower = np.array([5, 150, 170])
    upper = np.array([20, 255, 255])
    colors = np.array([0, 0, 255])

    blurred = cv2.GaussianBlur(cv_image, (7, 7), 0)  
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) 

    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) 

    contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(contour)>0:
        c = max(contour, key=cv2.contourArea)
        (x,y), radius = cv2.minEnclosingCircle(c)

        # Moments
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Specify min size of object to be detected
        if radius > 1:
            cv2.circle(cv_image, (int(x), int(y)), int(radius), colors, 2)
            cv2.putText(cv_image, "OBJECT", (int(x - (radius+1)), int(y - (radius+1))), cv2.FONT_HERSHEY_COMPLEX, 0.8, colors, 2)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
