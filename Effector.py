import cv2

class Effector:
     # Dibujar la posición del lapiz
    def draw_effector_position(self,frame):
        # Dimensiones del frame
        height, width, _ = frame.shape

        x_size = 150
        y_size = 430
        top_left_x = (width - 100) // 2
        top_left_y = (height) // 2
        bottom_right_x = top_left_x + x_size
        bottom_right_y = top_left_y + y_size
        cv2.rectangle(frame, (top_left_x, 0), (bottom_right_x, y_size), (0, 255, 0), 2)
        
        return frame