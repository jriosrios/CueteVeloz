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


def signal_stop(img):
  stop_sign = cv2.CascadeClassifier('/home/inda/Desktop/CueteVeloz/scripts/cascade_stop_sign.xml')

  
  if(gir > -1.1 and gir < 1.1):
    if(gir > 0.1):
       kurv = img[325:500,675:825]
    if(gir > -0.22):
       kurv = img[300:450,800:950]

  gray = cv2.cvtColor(kurv, cv2.COLOR_BGR2GRAY)
  cv2.imshow("semaforo", gray)

  stop_sign_scaled = stop_sign.detectMultiScale(gray, 1.3, 5)
  
  """
  for (x, y, w, h) in stop_sign_scaled:
    # Draw rectangle around the stop sign
    stop_sign_rectangle = cv2.rectangle(img, (x,y),
                                        (x+w, y+h),
                                        (0, 255, 0), 3)
    
    # Write "Stop sign" on the bottom of the rectangle
    stop_sign_text = cv2.putText(img = stop_sign_rectangle,
                                    text = "Stop Sign",
                                    org = (x, y+h+30),
                                    fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                                    fontScale = 1, color=(0, 0, 255),
                                    thickness = 2, lineType=cv2.LINE_4)
  """
  
  if stop_sign_scaled != ():
    sw = 1
  else:
    sw = 0

  return sw


def callback_signal(img):
    vel = rospy.Publisher('/speed', Float64,queue_size = 10)    #publicador de velocidad

    cv_image = bridge.imgmsg_to_cv2(img, "rgb8")
    sw = signal_stop(cv_image)


    if(sw == 0):  #switch para parar el carro
        
        if(gir > -1.1 and gir < 1.1):
            if(gir > -0.22):
                vel.publish(40)
                print("go")
                print(gir)
            if(gir > 0.16):
                vel.publish(5)
    else:
        print("stop")
        vel.publish(0)
        sleep(4)

    cv2.waitKey(1)

def medidas(msg):
  global gir
  gir = msg.data

def main(args):
  rospy.init_node('image_converter', anonymous = True)
  scan = rospy.Subscriber("/steering",Float64, medidas) #para extraer el giro del topic
  image_sub = rospy.Subscriber("/camera/rgb/raw", Image, callback_signal)

  try:
    rospy.spin()

  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
  