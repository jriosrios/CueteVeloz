import cv2
import numpy as np

img = cv2.imread('imagen1.jpg')
print(img.shape)
rojo = img[:,:,1]
azul = img[:,:,0]
verde = img[:,:,2]
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5,5), 0)
edges = cv2.Canny(blur, 50, 150, apertureSize=3)

# Dilatación para cerrar pequeñas brechas en los bordes
kernel = np.ones((3,3), np.uint8)
dilation = cv2.dilate(edges, kernel, iterations=1)

contours, _ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

"""
# Buscar octágonos
octagons = []
for contour in contours:
    approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)
    if len(approx) == 8:
        area = cv2.contourArea(approx)
        if area > 1000: # Seleccionar solo octágonos con un área mínima
            octagons.append(approx)

"""
#cv2.drawContours(img, octagons, -1, (0, 255, 0), 2)
cv2.imshow('canal 2', rojo)
cv2.imshow('rgb', img)
cv2.imshow('canal 3', verde)
cv2.imshow('canal 1', azul)
cv2.waitKey(0)
cv2.destroyAllWindows()