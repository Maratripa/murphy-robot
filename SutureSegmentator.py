import cv2
import numpy as np
import time

class SutureSegmentator:
    # Bloque 1: Conversión y segmentación de color
    def _apply_red_mask(self,hsv):
        red_lower1 = np.array([0, 120, 70])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 120, 70])
        red_upper2 = np.array([180, 255, 255])

        mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
        mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)

        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        return mask_red

    # Bloque 2: Procesamiento de la máscara para limpiar ruido
    def _clean_mask(self,mask_red):
        mask_red = cv2.GaussianBlur(mask_red, (5, 5), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        return mask_red

    # Bloque 3: Detección del contorno de la herida
    def _find_contours(self,mask_red):
        contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    # Bloque 4: Dibujar los puntos
    def _place_suture_points(self,contours, frame, padding=10, point_distance=40):
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                contour_length = cv2.arcLength(contour, True)
                num_points = int(contour_length // point_distance)
                num_points = num_points - 1 if num_points % 2 != 0 else num_points

                if num_points > 0:
                    actual_distance = contour_length / num_points
                    return self._draw_points_on_contour(contour, actual_distance, padding, frame)
        return frame

    # Bloque 5: Dibujar puntos en el contorno a intervalos uniformes
    def _draw_points_on_contour(self,contour, actual_distance, padding, frame):
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

                x_padded = int(interp_x + normal_x * padding)
                y_padded = int(interp_y + normal_y * padding)

                point_positions.append((x_padded, y_padded))

                current_distance -= actual_distance
            prev_point = current_point

        unique_points = set(point_positions)
        for point in unique_points:
            cv2.circle(frame, point, 5, (0, 0, 0), -1)
        
        return frame, unique_points

    # Segmentación completa y generación de puntos
    def draw_points(self,frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red = self._apply_red_mask(hsv)
        mask_red = self._clean_mask(mask_red)
        contours = self._find_contours(mask_red)
        frame_with_points, unique_points = self._place_suture_points(contours, frame)
        return frame_with_points, mask_red, frame, unique_points