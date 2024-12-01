import numpy as np
import cv2
from Algorithms import (pose_calc, inverse_kinematics,
                        flip_y, find_wound, find_pencil,
                          degree_to_radian, radian_to_degree,
                            translate_wound_geometry, define_stitching_points)
import serial
import time


class Brazo():

    #Clase que define un brazo robotico de dos grados de libertad 2D
    #Asume que el brazo es un eslabon de longitud L1 y otro de longitud L2
    #El brazo está centrado en el origen

    def __init__(self, length_1: float, length_2: float, precision: float, pencil_diff: float, camera_dist: float,
                  min_q1: float, max_q1: float, min_q2: float, max_q2: float, max_steps = 50000):

        self.L1 = length_1 #Longitud del primer eslabon
        self.L2 = length_2 #Longitud del segundo eslabon
        self.pencil_diff = pencil_diff #Distancia entre la punta del lapiz y el extremo del brazo
        self.camera_dist = camera_dist #Distancia entre la camara y el extremo del brazo
        self.precision = precision #Precision de la solucion

        self.q1 = 0 #Angulo de la articulacion 1
        self.q2 = 0 #Angulo de la articulacion 2

        self.max_steps = max_steps #Numero maximo de iteraciones calculo posición

        self.min_q1 = min_q1 #Angulo minimo de la articulacion 1
        self.max_q1 = max_q1 #Angulo maximo de la articulacion 1
        self.min_q2 = min_q2 #Angulo minimo de la articulacion 2
        self.max_q2 = max_q2 #Angulo maximo de la articulacion 2
    
    def pose(self):
        #Calcula la posicion actual del brazo robotico
        info = {"q1": self.q1, "q2": self.q2, "L1": self.L1, "L2": self.L2, "pencil_diff": self.pencil_diff, "camera_dist": self.camera_dist}
        return pose_calc(2, info)
    
    def get_angles(self, pose_deseada):

        return inverse_kinematics(2, {"pose_deseada": pose_deseada, "L1": self.L1, "L2": self.L2, "precision": self.precision, "max_steps": self.max_steps, "q1": self.q1, "q2": self.q2, "pencil_diff": self.pencil_diff, "camera_dist": self.camera_dist})
        
class Ojos():

    def __init__(self, Image_X: int, Image_Y: int, precision: float, FOV: float, tree_num: int, search_depth: int, min_matches: int):

        self.is_ready = False #Indica si el punto deseado está justo debajo
        self.MAX_X = Image_X #Ancho de la imagen
        self.MAX_Y = Image_Y #Alto de la imagen
        self.precision = precision #Precision de la solucion visual
        self.x_values = np.arange(0, self.MAX_X) #Posibles posiciones de x en la imagen
        self.y_values = np.arange(0, self.MAX_Y) #Posibles posiciones de y en la imagen
        self.center = np.array([Image_X/2, Image_Y/2]) #Centro de la imagen
        self.fov = FOV #Campo de vision de la camara
        self.wound_center = np.array([0, 0]) #Centro de la herida en la imagen
        self.wound_geometry = None #Geometria de la herida para poder trasladarla despues de encontrarla
        self.initial_stitches = np.array([np.nan]) #Puntos de sutura en imagen donde se detectaron por primera vez
        self.sift = cv2.SIFT_create() #Detector de puntos de interes

        FLANN_INDEX_KDTREE = 1
        index_params = {"algorithm": FLANN_INDEX_KDTREE, "trees": tree_num}
        search_params = {"checks": search_depth}
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self.min_matches = min_matches
        
    def find_wound_center_pixel(self, imagen):
        #Calcula la posicion del objetivo en la imagen en pixeles
        self.wound_center, _ = find_wound(imagen)
        return self.wound_center
    
    def get_image_file(self, imagen, show = False):

        self.imagen = cv2.imread(imagen)
        if show:
            cv2.imshow("Imagen", self.imagen)
            cv2.waitKey(0)

    def find_pencil_pixel(self, imagen):

        info = {"MAX_X": self.MAX_X, "MAX_Y": self.MAX_Y, "x_values": self.x_values, "y_values": self.y_values, "center": self.center}
        return find_pencil(imagen, info)
    
    def pixel_to_mm(self, height):

        mm_dist_hor = height * np.tan(degree_to_radian(self.fov/2))
        pix_to_mm = mm_dist_hor / self.center[0]
        return pix_to_mm
    
    def find_wound_geometry(self, imagen):

        wound_keypoints, wound_descriptors = self.sift.detectAndCompute(imagen, None)
        return [wound_keypoints, wound_descriptors]
    
    def save_wound_geometry(self, wound_geometry):
        self.wound_geometry = [wound_geometry[0], wound_geometry[1]]

    def define_stiching_points(self, image, height: float):
        info = {"pix_to_mm": self.pixel_to_mm(height), 
                "MAX_X": self.MAX_X, "MAX_Y": self.MAX_Y}
        self.initial_stitches = define_stitching_points(image, info)
        return 

    def translate_stiching_points(self, image, target: int):
        
        info = {"sift": self.sift, "flann": self.flann, "wound_descriptor": self.wound_geometry[1],
                 "minimum_matches": self.min_matches, "wound_keypoints": self.wound_geometry[0],
                   "initial_stitches": self.initial_stitches, "target": target}
        return translate_wound_geometry(image, info)
    
    def find_object(self, target, image):

        new_pixel = self.translate_stiching_points(image, target)

        if new_pixel == None:
            return None
        elif new_pixel[0] > self.MAX_X or new_pixel[1] > self.MAX_Y or new_pixel[0] < 0 or new_pixel[1] < 0:
            return None
        else:
            return new_pixel

class Comunicacion():

    def __init__(self, Tx: int, Rx: int, usb_port: str, timeout : float):

        self.port = usb_port #Puerto USB de la camara
        self.baudrate = 115200 #Baudrate esp32
        self.Tx = Tx #Puerto Tx de la Raspberry
        self.Rx = Rx #Puerto Rx de la Raspberry
        self.timeout = timeout #Tiempo de espera para la comunicacion
        self.ser = serial.Serial(self.port, self.baudrate, timeout = self.timeout) #Instancia del puerto serial 

    def send_data_gpio(self, data):

        message = data + "\n"
        message = message.encode("utf-8")
        return self.ser.write(message)
    
    def read_data_gpio(self):

        message = self.ser.readline().decode("utf-8")
        #Hay que definir un sistema de mensajes para la comunicacion para los sensores de distancia y el finished
        return(message)

    def read_usb(self):

        #Metodo para leer la camara, tengo que ver como está en el repo
        pass

    def send_motor_data(self, data):

        q1 = data[0]
        q2 = data[1]
        z = data[2]
        message = f"m;{q1};{q2};{z}"
        self.send_data_gpio(message)
        return self.read_data_gpio()

    def send_stop(self):

        message = "s"
        self.send_data_gpio(message)
        return self.read_data_gpio()

    def send_home(self):

        message = "h"
        self.send_data_gpio(message)
        return self.read_data_gpio()

    def send_request(self, data, num):

        message = f"r;{data};{num}"
        self.send_data_gpio(message)
        return self.read_data_gpio()

class Robot():

    def __init__(self, brazo: Brazo, ojos: Ojos, comunicacion: Comunicacion, precision: float,
                  height: float, screw_thread: float, Kp_2D: float, Kp_3D: float, min_z: float, max_z: float, max_steps = 100000):

        self.brazo = brazo #Instancia de la clase Brazo
        self.ojos = ojos #Instancia de la clase Ojos
        self.comunicacion = comunicacion #Instancia de la clase Comunicacion
        self.precision = precision #Precision de la solucion
        self.max_steps = max_steps #Maximo numero de iteraciones para calcular la posicion
        self.pencil_height = height #Altura del lapiz efector
        self.screw_thread = screw_thread #Distancia que avanza el tornillo por cada vuelta
        self.z = 0 #Altura del brazo
        self.min_z = min_z #Altura minima del brazo
        self.max_z = max_z #Altura maxima del brazo
        self.Kp_2D = Kp_2D #Constante de proporcionalidad para el control visual 2D
        self.Kp_3D = Kp_3D #Constante de proporcionalidad para el control visual 3D
        #Permite cerrar el lazo de control visualmente/contacto y no solo por posición estimada
        self.finised = False
         
    def pose(self):
        info = {"q1": self.brazo.q1, "q2": self.brazo.q2, "L1": self.brazo.L1, "L2": self.brazo.L2, "pencil_diff": self.brazo.pencil_diff, "camera_dist": self.brazo.camera_dist}
        return pose_calc(3, info)
    
    def get_state(self, pose_deseada):
        return inverse_kinematics(3, {"pose_deseada": pose_deseada, "L1": self.brazo.L1, "L2": self.brazo.L2, "precision": self.precision, "max_steps": self.max_steps, "q1": self.brazo.q1, "q2": self.brazo.q2, "pencil_diff": self.brazo.pencil_diff, "camera_dist": self.brazo.camera_dist, "pencil_height": self.pencil_height})

    def move(self, pose_deseada):

        #Mueve el brazo a la posición deseada
        state = self.get_state(pose_deseada)
        if self.is_valid_state(state)[0]:
            self.brazo.q1 = state[0]
            self.brazo.q2 = state[1]
            self.z = state[2]
            self.comunicacion.send_motor_data([self.brazo.q1, self.brazo.q2, self.z])
            return True
        else:
            print(self.is_valid_state(state)[1])
            return False
        
    def is_valid_state(self, state):

        if state[0] == None or state[1] == None or state[2] == None:
            return [False, None]
        elif state[0] < self.brazo.min_q1 or state[0] > self.brazo.max_q1 or state[1] < self.brazo.min_q2 or state[1] > self.brazo.max_q2 or state[2] < self.min_z or state[2] > self.max_z:
            return [False, "Angulo fuera de rango"]
        else:
            return [True, "Valido"]

    def center_wound(self):

        #Centra la herida en la vista de la camara
        image = self.comunicacion.read_usb()
        pixel_wound = flip_y(self.ojos.find_wound_center_pixel(image), self.ojos.MAX_Y)
        pixel_center = self.ojos.center

        move_vector = pixel_wound - pixel_center
        distance = np.linalg.norm(move_vector)
        
        while distance > self.ojos.precision:
            
            new_pose = self.brazo.pose() + self.Kp_2D*move_vector
            self.move(new_pose)

            image = self.comunicacion.read_usb()
            pixel_wound = flip_y(self.ojos.find_wound_center_pixel(image), self.ojos.MAX_Y)
            move_vector = pixel_wound - pixel_center
            distance = np.linalg.norm(move_vector)

        return True
    
    def move_pencil_to_object(self, target: int):
        
        #Mueve el lapiz a la posición deseada en pixeles
        imagen = self.comunicacion.read_usb()
        pixel_pencil = flip_y(self.ojos.find_pencil_pixel(imagen), self.ojos.MAX_Y)
        pixel_deseado = self.ojos.find_object(target, imagen)
        pixel_deseado = flip_y(pixel_deseado, self.ojos.MAX_Y)
        altura_actual = self.comunicacion.send_request("z2", 1000)

        move_vector_2D = pixel_deseado - pixel_pencil
        pix_to_mm_ratio = self.ojos.pixel_to_mm(altura_actual)
        move_vector_2D = move_vector_2D * pix_to_mm_ratio
        move_vector = np.array([move_vector_2D[0], move_vector_2D[1], altura_actual - self.pencil_height])
        distance = np.linalg.norm(move_vector)
        
        while distance > self.precision:

            new_pose = self.pose() + self.Kp_3D*move_vector
            self.move(new_pose)

            imagen = self.comunicacion.read_usb()
            pixel_deseado = self.ojos.find_object(target, imagen)
            altura_actual = self.comunicacion.send_request("z2", 1000)

            move_vector_2D = pixel_deseado - pixel_pencil
            pix_to_mm_ratio = self.ojos.pixel_to_mm(altura_actual)
            move_vector_2D = move_vector_2D * pix_to_mm_ratio
            move_vector = np.array([move_vector_2D[0], move_vector_2D[1], altura_actual - self.pencil_height])
            distance = np.linalg.norm(move_vector)
        
        return True
    
    def move_pencil_to_pos(self, pos_deseada):

        altura_actual = self.comunicacion.send_request("z2", 1000)

        move_vector = np.array([pos_deseada[0] - self.pose()[0], pos_deseada[1] - self.pose()[1], altura_actual - pos_deseada[2]])
        distance = np.linalg.norm(move_vector)
        
        while distance > self.precision:

            new_pose = self.pose() + self.Kp_3D*move_vector
            self.move(new_pose)

            altura_actual = self.comunicacion.send_request("z2", 1000)

            move_vector = np.array([pos_deseada[0] - self.pose()[0], pos_deseada[1] - self.pose()[1], altura_actual - pos_deseada[2]])
            distance = np.linalg.norm(move_vector)

        return True
    
    def stitch(self):

        n_stitches = len(self.ojos.initial_stitches)

        for i in range(n_stitches):
            touched = self.move_pencil_to_object(i)
            if touched:
                time.sleep(0.5)
                moved = self.move_pencil_to_pos(self.pose() + np.array([0, 0, 50]))
                if moved:
                    time.sleep(0.5)
                else:
                    print("Error al alejar el lapiz")
            else:
                print("Error al tocar el punto de sutura")

        self.finished = True

    def move_to_pos_list(self, pos_list):

        for pos in pos_list:
            moved = self.move_pencil_to_pos(pos)
            if not moved:
                print("Error al mover a la posición deseada")
                return False
        return True

    

if __name__ == "__main__":

    print("Hello World")
