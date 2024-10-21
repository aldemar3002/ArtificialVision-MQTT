import cv2
import numpy as np
import paho.mqtt.client as mqtt
import math

def callback(x):
    pass

# Configuración de MQTT (descomentar para usar MQTT)
#broker_address = "tu_broker_mqtt"
#mqtt_topic = "robot/control"
#client = mqtt.Client("TomatoDetector")
#client.connect(broker_address)

# Conectar con la cámara
cap = cv2.VideoCapture('http://172.18.1.61:4747/video')

# Definir el umbral de área mínima y máxima para la detección del jitomate
min_area = 40000  # Ajustar según las pruebas
max_area = 50000  # Ajustar según las pruebas

while True:
    # Leer la imagen de la webcam
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir la imagen de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definir los valores fijos del filtro HSV para jitomate (ajustar según pruebas)
    h_min, s_min, v_min = 0, 116, 86
    h_max, s_max, v_max = 179, 255, 226

    # Definir los límites del filtro HSV
    lower_hsv = np.array([h_min, s_min, v_min])
    upper_hsv = np.array([h_max, s_max, v_max])

    # Aplicar el filtro HSV y crear una máscara
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # Aplicar Gaussian Blur para reducir ruido
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Limpiar la máscara con erosión y dilatación
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Encontrar contornos en la máscara
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Dibujar los contornos en la imagen original
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            # Obtener la aproximación del contorno
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Dibujar el contorno
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)

            # Publicar el mensaje a través de MQTT según el área detectada
            if area < max_area:
                # Verificar si la forma es similar a un óvalo
                if len(approx) > 5:  # Asegurarse de que no es un polígono simple
                    # Ajustar una elipse al contorno
                    ellipse = cv2.fitEllipse(contour)
                    (x, y), (MA, ma), angle = ellipse

                    # Relación de los ejes mayor y menor de la elipse
                    aspect_ratio = ma / MA
                    if 0.75 <= aspect_ratio <= 1.25:  # Ajustar este valor según pruebas
                        mensaje = f"Tomate detectado, tamaño: {area} pixeles, forma: óvalo"
                        print(mensaje)
                        # client.publish(mqtt_topic, mensaje)

    # Mostrar la imagen original y la máscara
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)

    # Salir cuando se presiona 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la captura y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
