import cv2
import numpy as np
import paho.mqtt.client as mqtt
import time

def callback(x):
    pass

# Configuración de MQTT
broker_address = "172.18.0.92"
mqtt_topic = "servos/sensor"
client = mqtt.Client("TomatoDetector")
client.connect(broker_address)

# Conectar con la cámara
cap = cv2.VideoCapture('http://172.18.0.5:4747/video')

# Definir el umbral de área mínima y máxima para la detección del jitomate
min_area = 40000  # Ajustar según las pruebas
max_area = 50000  # Ajustar según las pruebas

# Tiempo para controlar el envío de mensajes (en segundos)
last_detection_time = 0
message_interval = 3
detection = False

while True:
    # Leer la imagen de la cámara
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir la imagen de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definir los valores fijos del filtro HSV para jitomate
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

    detection = False  # Reiniciar la detección para cada cuadro

    # Dibujar los contornos en la imagen original
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            # Obtener la aproximación del contorno
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Dibujar el contorno
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)

            # Verificar si la forma es similar a un óvalo
            if area < max_area and len(approx) > 5:
                # Ajustar una elipse al contorno
                ellipse = cv2.fitEllipse(contour)
                (x, y), (MA, ma), angle = ellipse

                # Relación de los ejes mayor y menor de la elipse
                aspect_ratio = ma / MA
                if 0.75 <= aspect_ratio <= 1.25:  # Ajustar según pruebas
                    detection = True
                    current_time = time.time()

                    # Si han pasado más de 3 segundos desde el último mensaje, enviar mensaje de detección
                    if current_time - last_detection_time > message_interval:
                        mensaje = "cosechar"
                        print(mensaje)
                        client.publish(mqtt_topic, mensaje)
                        last_detection_time = current_time

    # Si no se detectó ningún jitomate y han pasado más de 3 segundos, enviar un mensaje de "nada detectado"
    if not detection and time.time() - last_detection_time > message_interval:
        mensaje = "nada detectado"
        print(mensaje)
        client.publish(mqtt_topic, mensaje)
        last_detection_time = time.time()

    # Mostrar la imagen original y la máscara
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)

    # Salir cuando se presiona 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la captura y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
