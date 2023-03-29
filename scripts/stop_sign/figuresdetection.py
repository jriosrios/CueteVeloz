import cv2
import numpy as np

# Cargar imagen
img = cv2.imread('img.jpeg')

# Convertir a escala de grises
gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Aplicar umbralización
umbral, binaria = cv2.threshold(gris, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

# Buscar contornos en la imagen binaria
contornos, jerarquia = cv2.findContours(binaria, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Recorrer cada contorno y detectar su figura geométrica
for cnt in contornos:
    # Calcular aproximación poligonal del contorno
    approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
    # Obtener número de lados de la figura
    lados = len(approx)
    # Dibujar contorno y texto con la figura geométrica detectada

    if lados == 8:
        figura = "Octágono"
        area = cv2.contourArea(cnt)
        if area > 1000:
            figura = "octagono"
            cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)
            print(lados)
            cv2.putText(img, figura, (cnt[0][0][0], cnt[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            octa = cnt

    elif lados == 14:
        figura = "Figura con más de 6 lados"
        area = cv2.contourArea(cnt)
        if area > 1000:
            figura = "Circulo"
            cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)
            print(lados)
            cv2.putText(img, figura, (cnt[0][0][0], cnt[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cir = cnt
            
# Mostrar imagen con las figuras geométricas detectadas
cv2.imshow('Imagen', img)
cv2.waitKey(0)
cv2.destroyAllWindows()