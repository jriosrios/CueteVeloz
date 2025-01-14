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
from time import sleep

bridge = CvBridge()

def lines(image):
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)    #escala de grises
  blur = cv2.medianBlur(gray,5)   #filtro para reduccion de ruido
  _,thresh = cv2.threshold(blur,110,255,cv2.THRESH_BINARY)    #umbral de binarizacion
  canny = cv2.Canny(thresh,60,70,apertureSize=3)    #detector de bordes
  linesP = cv2.HoughLinesP(canny,rho=1, theta=np.pi/180,threshold=20,minLineLength=50,maxLineGap=400)   #detector de lineas
  return linesP
 
def signal_stop(image):
  signal = image[250:450,1050:]   #recorte de imagen para semaforo
  gray = cv2.cvtColor(signal, cv2.COLOR_BGR2GRAY)   #escala de grises
  blur = cv2.medianBlur(gray,5)   #filtro para reduccion de ruido
  _,thresh_signal = cv2.threshold(blur,125,255,cv2.THRESH_BINARY)   #umbral de binarizacion
  contours,_ = cv2.findContours(thresh_signal, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)    #deteccion de contornos
  var = ()
  sw = 0
  if contours != var:
    for cnt in contours:
      approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
      sides = len(approx)
      if sides > 14:
        figure = "circulo"
        area = cv2.contourArea(cnt)
        if area > 2000:
          cv2.drawContours(signal, [cnt], 0, (0, 255, 0), 3)
          cv2.putText(signal, figure, (cnt[0][0][0], cnt[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
          sw = 1
  cv2.imshow("signal", signal)
  cv2.waitKey(1)
  return sw

def callback(img):
  dir = rospy.Publisher('/steering', Float64,queue_size = 10)   #publicador de direccion
  vel = rospy.Publisher('/speed', Float64,queue_size = 10)    #publicador de velocidad
  
  #cv.image = bridge.cv2_to_imgmsg(img encoding="passthrough")    #imagen para carro fisico
  cv_image = bridge.imgmsg_to_cv2(img, "rgb8")
  
  sw = signal_stop(cv_image)

  image = cv_image[550:900,60:]   #recorte de imagen
  drawlines = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #para dibujar las lineas
  linesP = lines(image)   #se llama la funcion de obtencion de lineas

  linesL=[]   #para guardar todas las lineas de la izquierda detectadas
  linesR=[]   #para guardar todas las lineas de la derecha detectadas
  
  if linesP is not None:    #si existe alguna linea entra en la condicion 
    for i in range(0, len(linesP)):
      l = linesP[i][0]    #selecciona una linea a la vez
      m = (l[3]-l[1])/(l[2]-l[0])   #pendiente de las lineas
      
      if(m!=0):   #filtrar lineas completamente verticales
        if(m>0):    #si es positiva es linea izquierda
          linesR.append(linesP[i][0])   #agrega la linea a una lista
        else:   #si es negativa es linea derecha
          linesL.append(linesP[i][0])   #agrega la linea a una lista

    var=[]
    
    if linesL != var:   #si existe mas de 0 lineas se promedian
      LL=np.mean(linesL,axis=0)
    else:   #si no la linea es igual a 0,0,0,0
      LL=[0,0,0,0]
    
    if linesR != var:   #si existe mas de 0 lineas se promedian
      LR=np.mean(linesR,axis=0)
    else:   #si no la linea es igual a 0,0,0,0
      LR=[0,0,0,0]
    
    if(int(LL[2])-int(LL[0])!=0):   #para evitar dividir entre 0 en la pendiente
      mL = (LL[3]-LL[1])/(LL[2]-LL[0])
      if(mL<-0.1):    #filtro para pendientes muy horizontales
        bL = (LL[1])-(mL*(LL[0]))
        #se extienden las lineas hasta los bordes de la imagen
        x1_L = int((349-bL)/mL)
        y1_L = 349
        x2_L = int((0-bL)/mL)
        y2_L = 0
      else:   #si es muy cercano a una linea horizontal es 0
        x1_L = 0
        y1_L = 0
        x2_L = 0
        y2_L = 0
    else:   #si se divide entre 0 es 0
      x1_L = 0
      y1_L = 0
      x2_L = 0
      y2_L = 0

    if(int(LR[2])-int(LR[0])!=0):   #condicion para evitar division entre 0
      mR = (LR[3]-LR[1])/(LR[2]-LR[0])
      bR = (LR[1])-(mR*(LR[0]))
      #se extienden las lineas hasta los bordes de la imagen
      x1_R = int((349-bR)/mR)
      y1_R = 349
      x2_R = int((0-bR)/mR)
      y2_R = 0
    else    :#si se divide entre 0 es 0
      x1_R = 0
      y1_R = 0
      x2_R = 0
      y2_R = 0

    cv2.line(drawlines, (x1_L, y1_L), (x2_L, y2_L), (255,255,255),3,cv2.LINE_AA)    #dibja la linea izquierda extendida
    cv2.line(drawlines, (x1_R, y1_R), (x2_R, y2_R), (  0,  0,255),3,cv2.LINE_AA)    #dibuja la linea derecha extendida
    #                     x1    y1      x2    y2
    
    gir = 0.00001557*(x1_L) - 0.0002931*(y1_L) - 0.00001961*(x2_L)  - 0.00008441*(x1_R)  - 0.00005383*(x2_R) + 0.2925 # giro del carro obtenido de la regresion lineal

    if(sw==0):  #switch para parar el carro
      if(gir>-1.1 and gir<1.1):
        if(gir>-0.22):
          dir.publish(gir-0.04)
          vel.publish(40)
          print("go")
        if(gir>0.16): #condicion para reducir la velocidad en curbas cerradas
          dir.publish(gir)
          print("bajo velocidad")
          vel.publish(5)
    else:
      dir.publish(0)
      vel.publish(0)
      print("parar")
      sleep(10)
  
  cv2.imshow("Image window", drawlines)
  cv2.waitKey(1)


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