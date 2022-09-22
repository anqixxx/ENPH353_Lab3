#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # The way that python defines object oriented classes, a constructor
    # Self is reference to object
  def __init__(self):
    # All are objects

    # A publisher is created, the input is what channel it will publish
    self.twist_pub = rospy.Publisher('/cmd_vel', Twist)

    self.move = Twist()

    # Bridge is translator, converts
    self.bridge = CvBridge()

    # What it is information coming from, which topic is the information coming from
    self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw',Image,self.callback)


  # More similar to an interupt
  def callback(self,data):
    try:
    # Translates from ROS images to CV image
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Also should publish how the robot moves
    
    (rows,cols,channels) = cv_image.shape


    #Perform operations until we can let out self object know how to move
    edges = cv2.Canny(cv_image, 100, 200)

    # After information is published, self.twist_pub.publish(self.move)

    # Use if statements and move.linear and move.angular to find it


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

