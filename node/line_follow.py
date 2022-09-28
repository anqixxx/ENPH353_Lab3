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
import matplotlib.pyplot as plt


class image_converter:

    # The way that python defines object oriented classes, a constructor
    # Self is reference to object
  def __init__(self):
    # All are objects

    # A publisher is created, the input is what channel it will publish
    self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    self.move = Twist()

    # Bridge is translator, converts
    self.bridge = CvBridge()

    # What it is information coming from, which topic is the information coming from
    self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw',Image,self.callback)


  # More similar to an interupt
  def callback(self,data):
    #print("Callback")
    try:
    # Translates from ROS images to CV image
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Also should publish how the robot moves
    
    (rows,cols,channels) = cv_image.shape

    #cv2.imshow('cv image',cv_image)

    # Perform operations until we can let out self object know how to move
    # Image Preprocessing
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Thresholding
    ret,bin_img = cv2.threshold(gray,100,255,cv2.THRESH_BINARY_INV)
    xdim = bin_img.shape[0]
    ydim = bin_img.shape[1]

    # Define the move object
    move = Twist()
    first = 0
    second = 0

    # Finds the location of the first white dot, which is where the line starts
    for x in range(0, xdim):
      if (bin_img[-1][x] == 255):
        first = x
        break

    # Finds the location of the first black dot after the white, which is where the line ends
    for x in range(first, xdim):
      if (bin_img[-1][x] == 0):
        second = x
        break

    # Code for iteration if the robot is not directly started on the line
    # y = -1
    # while first==0 and second ==0:
    #   y = y-1
    #   for x in range(0, xdim):
    #     if (bin_img[-2][x] == 255):
    #       first = x
    #       break

    #   for x in range(first, xdim):
    #     if (bin_img[-2][x] == 0):
    #       second = x
    #       break
      

    xMax = second
    xMin = first
    
    # Fiinds the difference between the right edge of the line and the end of the frame
    maxDiff = xdim - xMax

    # If the line is more to the right then the left, move to the left
    if (maxDiff > (xMin-10)):
      move.angular.z = 0.25
      self.twist_pub.publish(move)

    # If the line is more to the left then the right, move to the right
    if (xMin > (maxDiff +10)):
      move.angular.z = -0.25
      self.twist_pub.publish(move)

    # If the line is near the center, move straight
    if (xMin < maxDiff + 10 or xMin > maxDiff -10):
      move.linear.x = .25
      self.twist_pub.publish(move)


    # Use if statements and move.linear and move.angular to find it

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

