#!/usr/bin/env python3
from __future__ import print_function
import numpy as np
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String,Int16,Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib as plt

bridge = CvBridge()
file = open('dataset2.csv', 'w')

#para la sub de extraer el valor del topic de direccion

def medidas(msg):
  global gir
  gir = msg.data


def lines(image):
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #escala de grises
  blur = cv2.medianBlur(gray,5) #filtro para reduccion de ruido
  _,thresh = cv2.threshold(blur,110,255,cv2.THRESH_BINARY) #umbral de binarizacion
  canny = cv2.Canny(thresh,60,70,apertureSize=3) #
  linesP = cv2.HoughLinesP(canny,rho=1, theta=np.pi/180,threshold=20,minLineLength=50,maxLineGap=400)
  return linesP
 

def callback(img):
  dir = rospy.Publisher('/steering', Float64,queue_size = 10) #publicador de direccion
  vel = rospy.Publisher('/speed', Float64,queue_size = 10) #publicador de velocidad
  
  #cv.image = bridge.cv2_to_imgmsg(img encoding="passthrough")    #carro fisico
  cv_image = bridge.imgmsg_to_cv2(img, "rgb8")                    #simulador
  image = cv_image[550:900,60:] #recorte de imagen
  gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) #para dibujar las lineas obtenidas
  
  linesP = lines(image) #obtencion de lineas

  linesL=[]#para guardar todas las lineas de la izquierda detectadas
  linesR=[]#para guardar todas las lineas de la derecha detectadas
  
  if linesP is not None: #si existe alguna linea entra en la condicion 
    #Diferencia entre lineas izquierdas y derechas
    for i in range(0, len(linesP)):
      l = linesP[i][0]#selecciona una linea a la vez
      m = (l[3]-l[1])/(l[2]-l[0])#pendiente de las lineas
      
      if(m!=0):#filtrar lineas completamente verticales
        if(m>0):#si es positiva es linea izquierda
          linesR.append(linesP[i][0])#agrega la linea a una lista
          
        else:#si es negativa es linea derecha
          linesL.append(linesP[i][0])#agrega la linea a una lista

    var=[]
    
    if linesL != var:#si existe mas de 0 lineas se promedian
      LL=np.mean(linesL,axis=0)
    else:#si no la linea es igual a 0,0,0,0
      LL=[0,0,0,0]
    
    if linesR != var:#si existe mas de 0 lineas se promedian
      LR=np.mean(linesR,axis=0)
    else:#si no la linea es igual a 0,0,0,0
      LR=[0,0,0,0]
    
    if(int(LL[2])-int(LL[0])!=0):#para evitar dividir entre 0 en la pendiente
      mL = (LL[3]-LL[1])/(LL[2]-LL[0])
      if(mL<-0.1):#filtro para pendientes muy horizontales
        bL = (LL[1])-(mL*(LL[0]))
        x1_L = int((349-bL)/mL)
        y1_L = 349
        x2_L = int((0-bL)/mL)
        y2_L = 0
      else:#si es horizontal es 0
        x1_L = 0
        y1_L = 0
        x2_L = 0
        y2_L = 0
    else:
      x1_L = 0
      y1_L = 0
      x2_L = 0
      y2_L = 0

    if(int(LR[2])-int(LR[0])!=0):
      mR = (LR[3]-LR[1])/(LR[2]-LR[0])
      bR = (LR[1])-(mR*(LR[0]))
      
      x1_R = int((349-bR)/mR)
      y1_R = 349
      x2_R = int((0-bR)/mR)
      y2_R = 0
    else:
      x1_R = 0
      y1_R = 0
      x2_R = 0
      y2_R = 0
    
    #linea para el dataset
    file.write(f"{x1_L},{y1_L},{x2_L},{y2_L},{x1_R},{y1_R},{x2_R},{y2_R},{gir}\n")

    cv2.line(gray, (x1_L, y1_L), (x2_L, y2_L), (255,255,255),3,cv2.LINE_AA)
    cv2.line(gray, (x1_R, y1_R), (x2_R, y2_R), (  0,  0,255),3,cv2.LINE_AA)
    #                 x1    y1      x2    y2
  
  cv2.imshow("Image window", gray)
  cv2.waitKey(1)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  scan = rospy.Subscriber("/steering",Float64, medidas) #para extraer el giro del topic
  image_sub = rospy.Subscriber("/camera/rgb/raw", Image, callback)
    
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)