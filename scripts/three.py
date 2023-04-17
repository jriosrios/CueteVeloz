#gir > 0 -> izquierda
#gir < 0 -> derecha

#carro antiguo
#900 izquierda
#2000 derecha

def convertir_old(gir):
    valor_minimo_original = -1.1
    valor_maximo_original = 1.1
    valor_minimo_convertido = 2000
    valor_maximo_convertido = 900

    rango_original = valor_maximo_original - valor_minimo_original
    rango_convertido = valor_maximo_convertido - valor_minimo_convertido
    
    gir = (((gir - valor_minimo_original) * rango_convertido) / rango_original) + valor_minimo_convertido
    return gir


def convertir_new(gir):
    valor_minimo_original = -1.1
    valor_maximo_original = 1.1
    valor_minimo_convertido = 70
    valor_maximo_convertido = 110

    rango_original = valor_maximo_original - valor_minimo_original
    rango_convertido = valor_maximo_convertido - valor_minimo_convertido
    
    gir = (((gir - valor_minimo_original) * rango_convertido) / rango_original) + valor_minimo_convertido
    return gir

gir = convertir_old(1)
print("giro carro viejo: "+str(int(gir)))
gir = convertir_new(1)
print("giro carro nuevo: "+str(int(gir)))