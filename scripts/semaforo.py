#!/usr/bin/env python3
from __future__ import print_function
import numpy as np
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib as plt
from time import sleep

bridge = CvBridge()

def lines(image):
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #escala de grises
  blur = cv2.medianBlur(gray,5) #filtro para reduccion de ruido
  _,thresh = cv2.threshold(blur,110,255,cv2.THRESH_BINARY) #umbral de binarizacion
  canny = cv2.Canny(thresh,60,70,apertureSize=3) #
  linesP = cv2.HoughLinesP(canny,rho=1, theta=np.pi/180,threshold=20,minLineLength=50,maxLineGap=400)
  return linesP
 
def signal_stop(image):
  signal = image[250:450,1050:]
  gray = cv2.cvtColor(signal, cv2.COLOR_BGR2GRAY) #escala de grises
  blur = cv2.medianBlur(gray,5) #filtro para reduccion de ruido
  _,thresh_signal = cv2.threshold(blur,125,255,cv2.THRESH_BINARY)
  contours,_ = cv2.findContours(thresh_signal, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  var=()
  if contours != var:
    for cnt in contours:
      # Calcular aproximación poligonal del contorno
      approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
      # Obtener número de lados de la figura
      lados = len(approx)
      # Dibujar contorno y texto con la figura geométrica detectada


      if lados == 14:
        figura = "Figura con más de 6 lados"
        area = cv2.contourArea(cnt)

        if area > 2000:
          figura = "Circulo"
          cv2.drawContours(signal, [cnt], 0, (0, 255, 0), 3)
          print(lados)
          cv2.putText(signal, figura, (cnt[0][0][0], cnt[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
          cir = 1
        else:
          cir=0
      else:
        cir=0
  else:
    cir=0
  cv2.imshow("signal", signal)
  cv2.waitKey(1)
  return cir

def callback(img):
  dir = rospy.Publisher('/steering', Float64,queue_size = 10) #publicador de direccion
  vel = rospy.Publisher('/speed', Float64,queue_size = 10) #publicador de velocidad
  
  #cv.image = bridge.cv2_to_imgmsg(img encoding="passthrough")    #carro fisico
  cv_image = bridge.imgmsg_to_cv2(img, "rgb8")                 #simulador
  #color= cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
  draw = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) #escala de grises
  
  cir= signal_stop(cv_image)
  print(cir)
  image = cv_image[550:900,60:] #recorte de imagen
  #draw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #escala de grises
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
    
    gir = 0.00001557*(x1_L) - 0.0002931*(y1_L) - 0.00001961*(x2_L)  - 0.00008441*(x1_R)  - 0.00005383*(x2_R) + 0.2925 

    if(cir==0):
      if(gir>-1.1 and gir<1.1):
        if(gir>-0.22):
          dir.publish(gir-0.04)
          vel.publish(40)
          print("go")
        if(gir>0.16):
          dir.publish(gir)
          print("bajo velocidad")
          vel.publish(5)
    else:
      dir.publish(0)
      vel.publish(0)
      print("parar")
      sleep(10)

    #cv2.line(draw, (x1_L, y1_L), (x2_L, y2_L), (255,255,255),3,cv2.LINE_AA)
    #cv2.line(draw, (x1_R, y1_R), (x2_R, y2_R), (  0,  0,255),3,cv2.LINE_AA)
    #                 x1    y1      x2    y2
  
  #cv2.imshow("Image window", draw)
  #cv2.waitKey(1)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  image_sub = rospy.Subscriber("/camera/rgb/raw", Image, callback)
    
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)