import numpy as np
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import sleep

bridge = CvBridge()

def callback_signal(img):

    cv_image = bridge.imgmsg_to_cv2(img, "rgb8")
    signal = cv_image[275:450,675:950]
    cv2.imshow("Image window", signal)
    cv2.waitKey(1)


def main(args):
  rospy.init_node('image_converter', anonymous = True)
  image_sub = rospy.Subscriber("/camera/rgb/raw", Image, callback_signal)

  try:
    rospy.spin()

  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)