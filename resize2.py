#!/usr/bin/env python

import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def get_image():
    
    cap = cv.VideoCapture(0)
    pub1 = rospy.Publisher('Imagen_RGB', Image, queue_size=10)
    pub2 = rospy.Publisher('Imagen_Grey', Image, queue_size=10)
    rospy.loginfo("Setting up the node...")
    rospy.init_node('Convert_image')
    rate = rospy.Rate(30) # 30hz
    
    while not rospy.is_shutdown():
         ret, frame = cap.read()
         alto, ancho = frame.shape[:2]
         factor = 2
         n_alto = alto*factor
         n_ancho = ancho*factor
         frame = cv.resize(frame, (n_ancho, n_alto), interpolation=cv.INTER_CUBIC)
         frame = np.asarray(frame)
         image_message1 = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
         pub1.publish(image_message1)
         print(frame.shape)
         gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
         image_message2 = bridge.cv2_to_imgmsg(gray_frame, encoding="passthrough")
         pub2.publish(image_message2)
         
         rate.sleep()
         
if __name__ == '__main__':
     try:
         get_image()
     except rospy.ROSInterruptException:
         pass