#!/usr/bin/env python3
from __future__ import print_function
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Stop Sign Cascade Classifier xml
stop_sign = cv2.CascadeClassifier('/home/navandree/Desktop/stop_sign_detection/cascade_stop_sign.xml')

def stop_detected(image_msg):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding = 'bgr8')
    signal = image[250:450, 1050:]
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    stop_sign_scaled = stop_sign.detectMultiScale(gray, 1.3, 5)

    # Detect the stop sign, x,y = origin points, w = width, h = height
    for (x, y, w, h) in stop_sign_scaled:
        # Draw rectangle around the stop sign
        stop_sign_rectangle = cv2.rectangle(image, (x,y), (x+w, y+h), (0, 255, 0), 3)
        # Write "Stop sign" on the bottom of the rectangle
        stop_sign_text = cv2.putText(img=stop_sign_rectangle, text="Stop Sign", org=(x, y+h+30), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                                     fontScale=1, color=(0, 0, 255),thickness=2, lineType=cv2.LINE_4)

    cv2.imshow("Octagon Detection", signal)
    cv2.waitKey(1)

def main(args):
  rospy.init_node('octagon_detector')
  imagen_sub = rospy.Subscriber("/camera/rgb/raw", Image, stop_detected)
    
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
