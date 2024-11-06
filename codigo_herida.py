import cv2
import numpy as np
import serial
import time
#TODO: Output points coordinates img to world coordinates

# Bloque 1: Conversión y segmentación de color
def apply_red_mask(hsv):
    # Definir los rangos del color rojo en HSV
    red_lower1 = np.array([0, 120, 70])
    red_upper1 = np.array([10, 255, 255])
    
    red_lower2 = np.array([170, 120, 70])
    red_upper2 = np.array([180, 255, 255])

    # Crear máscaras para los dos rangos de color rojo
    mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
    mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)

    # Combinar las máscaras
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    return mask_red

# Bloque 2: Procesamiento de la máscara para limpiar ruido
def clean_mask(mask_red):
    # Aplicar desenfoque gaussiano
    mask_red = cv2.GaussianBlur(mask_red, (5, 5), 0)

    # Operaciones morfológicas para limpiar la máscara
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)  # Cerrar huecos pequeños
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)   # Remover ruido pequeño

    return mask_red

# Bloque 3: Detección de contornos
def find_contours(mask_red):
    contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

# Bloque 4: Colocación de puntos de sutura
def place_suture_points(contours, frame, padding=10, point_distance=40):
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Ignorar áreas pequeñas para evitar ruido
            # Dibujar el contorno
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)  # Contorno verde

            # Longitud total del contorno
            contour_length = cv2.arcLength(contour, True)

            # Calcular el número de puntos de sutura
            num_points = int(contour_length // point_distance)
            num_points = num_points - 1 if num_points % 2 != 0 else num_points

            # Colocar puntos en el contorno
            if num_points > 0:
                actual_distance = contour_length / num_points
                return draw_points_on_contour(contour, actual_distance, padding, frame)
    return frame

# Bloque 5: Dibujar puntos en el contorno a intervalos uniformes
def draw_points_on_contour(contour, actual_distance, padding, frame):
    point_positions = []
    current_distance = 0
    prev_point = contour[0][0]
    
    for i in range(1, len(contour)):
        current_point = contour[i][0]
        dist = np.linalg.norm(current_point - prev_point)
        current_distance += dist

        while current_distance >= actual_distance:
            ratio = (current_distance - actual_distance) / dist
            interp_x = int(prev_point[0] + ratio * (current_point[0] - prev_point[0]))
            interp_y = int(prev_point[1] + ratio * (current_point[1] - prev_point[1]))

            dx, dy = current_point[0] - prev_point[0], current_point[1] - prev_point[1]
            length = np.sqrt(dx**2 + dy**2)
            normal_x, normal_y = (-dy / length, dx / length) if length != 0 else (0, 0)

            # Aplicar el padding
            x_padded = int(interp_x + normal_x * padding)
            y_padded = int(interp_y + normal_y * padding)

            point_positions.append((x_padded, y_padded))

            current_distance -= actual_distance
        prev_point = current_point

    # Dibujar los puntos en el frame
    unique_points = set(point_positions)
    for point in unique_points:
        cv2.circle(frame, point, 5, (0, 0, 0), -1)  # Punto negro
    
    return frame

# Bloque 6: Segmentación completa y generación de puntos
def segment_red(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red = apply_red_mask(hsv)
    mask_red = clean_mask(mask_red)
    contours = find_contours(mask_red)
    frame_with_points = place_suture_points(contours, frame)

    return frame_with_points, mask_red, frame

# Bloque 7: Captura de video
def start_video_capture():
    # Configurar la conexión serie
    arduino_port = '/dev/tty.usbserial-110'  # Cambia esto por tu puerto serie (COM3 en Windows, /dev/ttyUSB0 o /dev/ttyACM0 en Linux)
    baud_rate = 9600  # Debe coincidir con el baud rate configurado en Arduino

    # Iniciar la conexión
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # Esperar a que Arduino se reinicie y esté listo para enviar datos
    capture = cv2.VideoCapture(0)
    generate_points = False

    while True:
        ret, frame = capture.read()
        if not ret:
            break

        if generate_points:
            frame_with_contours, mask_red, frame_with_points = segment_red(frame)
            cv2.imshow('Camera with Points', frame_with_points)
            generate_points = False
        else:
            cv2.imshow('Camera', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            generate_points = True
        else:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').rstrip()  # Leer y decodificar la línea
                print(data)
                if data == '1':
                    generate_points = True

    capture.release()
    cv2.destroyAllWindows()

# Ejecutar la captura de video
start_video_capture()
