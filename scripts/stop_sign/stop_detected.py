#!/usr/bin/env python3
from __future__ import print_function
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def octagon_detection(image_msg):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, cv2.arcLength(cnt, True) * 0.02, True)
        if len(approx) == 8:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w)/h
            if aspect_ratio >= 0.9 and aspect_ratio <= 1.1:
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.imshow("Octagon Detection", image)
                cv2.waitKey(0)
                return 1
                
    cv2.imshow("Octagon Detection", image)
    cv2.waitKey(0)
    return 0

def callback(image_msg):
    result = octagon_detection(image_msg)
    if result == 1:
        print("Octagon detected!")

def main(args):
  rospy.init_node('octagon_detector')
  imagen_sub = rospy.Subscriber("/camera/rgb/raw", Image, callback)
    
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
