# CueteVeloz Team

## Prueba de conduccion autonoma sin obstaculos
autonomous_final.py

Ejecutar el algorithmo para la prueba de conduccion autonoma sin obstaculos ni señales de trafico

## Prueba de conduccion autonoma con deteccion de señales de alto
stopsign_steering_final.py
stopsign_speed_final.py

Ejecutar primero el script *stopsign_steering_final.py* e inmediatamente despues el scrpt *stopsign_speed_final.py*

###Consideraciones para esta prueba
En el script *stopsign_speed_final.py* cambiar la ubicacion del archivo *cascade_stop_sign.xml*
a donde se haya descargado el repositorio en su ordenador.

Se encuentra en la linea 14

stop_sign = cv2.CascadeClassifier('/home/inda/Desktop/CueteVeloz/scripts/cascade_stop_sign.xml')
