import cv2
import numpy as np
import paho.mqtt.client as mqtt
import time

# Configuración de MQTT
broker_address = "192.168.176.61"
mqtt_topic_tomato = "laptop/tomate/x"
mqtt_topic_square = "laptop/cuadrado/x"
mqtt_topic_alignment = "laptop/ajuste/x"
client = mqtt.Client("ObjectDetector")
client.connect(broker_address, port=1883)

# Conectar con la cámara
cap = cv2.VideoCapture('http://192.168.176.251:4747/video')

# Definir el umbral de área mínima y máxima para la detección de jitomates y cuadrados
min_area_tomato = 10000  # Ajustar 
max_area_tomato = 30000

min_area_square = 2000  # Ajustar 
max_area_square = 10000

# Tiempo para controlar el envío de mensajes (en segundos)
last_detection_time_tomato = 0
last_detection_time_square = 0
last_alignment_time = 0
message_interval = 3

# Crear el kernel para operaciones morfológicas
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

while True:
    # Leer la imagen de la cámara
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir la imagen de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definir los valores fijos del filtro HSV para objetos rojos
    h_min, s_min, v_min = 0, 116, 86
    h_max, s_max, v_max = 179, 255, 226

    # Definir los límites del filtro HSV
    lower_hsv = np.array([h_min, s_min, v_min])
    upper_hsv = np.array([h_max, s_max, v_max])

    # Aplicar el filtro HSV y crear una máscara
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # Aplicar Gaussian Blur para reducir ruido
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    # Aplicar operaciones morfológicas para mejorar la calidad de la máscara
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Rellenar huecos
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Eliminar ruido

    # Encontrar contornos en la máscara
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detection_tomato = False
    detection_square = False

    # Variables para almacenar los centroides
    tomato_cX = None
    square_cX = None

    # Procesar los contornos
    for contour in contours:
        area = cv2.contourArea(contour)

        # Detección de jitomate (elíptico)
        if min_area_tomato < area < max_area_tomato:
            if len(contour) >= 5:  # Se necesita al menos 5 puntos para ajustar una elipse
                ellipse = cv2.fitEllipse(contour)
                (x, y), (MA, ma), angle = ellipse

                aspect_ratio = ma / MA
                if 0.75 <= aspect_ratio <= 1.25:  # Forma aproximadamente circular
                    # Dibujar contorno y elipse
                    cv2.drawContours(frame, [contour], -1, (0, 255, 0), 3)
                    cv2.ellipse(frame, ellipse, (255, 0, 0), 2)

                    # Calcular el centroide
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        tomato_cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        # Dibujar el centroide
                        cv2.circle(frame, (tomato_cX, cY), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"X: {tomato_cX}, Y: {cY}", (tomato_cX + 10, cY - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                        detection_tomato = True
                        current_time = time.time()

                        if current_time - last_detection_time_tomato > message_interval:
                            mensaje_tomato = f"{tomato_cX}"
                            print("Tomate -", mensaje_tomato)
                            client.publish(mqtt_topic_tomato, mensaje_tomato)
                            last_detection_time_tomato = current_time

        # Detección de cuadrado
        elif min_area_square < area < max_area_square:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

            if len(approx) == 4:  # Contorno con 4 lados
                # Dibujar contorno
                cv2.drawContours(frame, [approx], -1, (0, 255, 255), 3)

                # Calcular el centroide
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    square_cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Dibujar el centroide
                    cv2.circle(frame, (square_cX, cY), 5, (255, 0, 255), -1)
                    cv2.putText(frame, f"X: {square_cX}, Y: {cY}", (square_cX + 10, cY - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    detection_square = True
                    current_time = time.time()

                    if current_time - last_detection_time_square > message_interval:
                        mensaje_square = f"{square_cX}"
                        print("Pinza -", mensaje_square)
                        client.publish(mqtt_topic_square, mensaje_square)
                        last_detection_time_square = current_time

    # Comparar las posiciones en X y enviar mensaje de ajuste
    if tomato_cX is not None and square_cX is not None:
        alignment_message = ""
        if square_cX < tomato_cX - 10:  # Margen de 10 píxeles
            alignment_message = "izquierda"
        elif square_cX > tomato_cX + 10:
            alignment_message = "derecha"
        else:
            alignment_message = "centrado"

        current_time = time.time()
        if current_time - last_alignment_time > message_interval:
            client.publish(mqtt_topic_alignment, alignment_message)
            print(f"Ajuste - {alignment_message}")
            last_alignment_time = current_time

    # Mostrar la imagen original y la máscara
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)

    # Salir cuando se presiona 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la captura y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
