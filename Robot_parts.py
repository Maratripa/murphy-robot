import numpy as np
import cv2
from Algorithms import (pose_calc, inverse_kinematics,
                        flip_y, find_wound, find_pencil,
                          degree_to_radian, radian_to_degree,
                            translate_wound_geometry, define_stitching_points)
import serial
import time
import threading


WAIT = 0.5


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
        if self.wound_center is None:
            print("no wound found")
            return None
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
        return True

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
        
    def estimate_stitch_pose(self, stitches: np.array, height: float, actual_pose: np.array):

        pix_to_mm = self.pixel_to_mm(height)
        stitches_pos = np.zeros((stitches.shape[0],3))
        print(stitches.shape)
        for i in range(stitches.shape[0]):
            pix_diff = stitches[i] - self.center
            mm_diff = pix_diff * pix_to_mm
            stitches_pos[i] = actual_pose + np.array([mm_diff[0], mm_diff[1], -height])

        return stitches_pos

class Comunicacion():

    def __init__(self, serial_port: str, timeout : float):

        self.port = serial_port #Puerto USB de la camara
        self.baudrate = 115200 #Baudrate esp32
        self.timeout = timeout #Tiempo de espera para la comunicacion
        self.ser = serial.Serial(self.port, self.baudrate) #Instancia del puerto serial
        self.info = None
        self.info_lock = threading.Lock()
        self.init_read()


    def send_data_gpio(self, data):

        message = data + ";"
        message = message.encode()
        return self.ser.write(message)
    
    def read_data_gpio(self):
        
        while self.info is None:
            time.sleep(0.01)
        
        
        return_info = self.info

        with self.info_lock:
            self.info = None

        return return_info


    def _receive_data(self):
        recieved = ""
        while True:
            while self.ser.in_waiting:
                caracter = self.ser.read()
                recieved += caracter.decode()
                if recieved[-1] == ";":
                    recieved = recieved[:-1]
                    with self.info_lock:
                        if recieved[0] == "z":
                            info = recieved.split(",")
                            sensor = info[0]
                            data = float(info[1])
                            self.info = data
                        elif recieved[0] == "G":
                            self.info = True
                        elif recieved[0] == "B":
                            self.info = False
                    recieved = ""
                    caracter = "0"


    def init_read(self):
        Myth = threading.Thread(target=self._receive_data)
        Myth.daemon = True
        Myth.start()
            
    def read_usb(self):

        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        return frame

    def send_motor_data(self, data):

        q1 = data[0]
        q2 = data[1]
        z = data[2]
        message = f"m,{q2},{q1},{z}"
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

        message = f"r,{data},{num}"
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
        info = {"q1": self.brazo.q1, "q2": self.brazo.q2,
         "L1": self.brazo.L1, "L2": self.brazo.L2,
          "pencil_diff": self.brazo.pencil_diff,
           "camera_dist": self.brazo.camera_dist,
           "height": self.z,
        "pencil_height": self.pencil_height}
        return pose_calc(3, info)
    
    def get_state(self, pose_deseada):
        return inverse_kinematics(3, {"pose_deseada": pose_deseada, "L1": self.brazo.L1, "L2": self.brazo.L2, "precision": self.precision, "max_steps": self.max_steps, "q1": self.brazo.q1, "q2": self.brazo.q2, "pencil_diff": self.brazo.pencil_diff, "camera_dist": self.brazo.camera_dist, "pencil_height": self.pencil_height, "z": self.z})

    def move(self, pose_deseada, angulo_deseado = [None]):
        #Mueve el brazo a la posición deseada
        if angulo_deseado[0] != None:
            is_valid = self.is_valid_state(angulo_deseado)
            if is_valid[0]:
                internal_height = self.comunicacion.send_request("z1", 4)
                delta = angulo_deseado[2] - self.z
                self.brazo.q1 = angulo_deseado[0]
                self.brazo.q2 = angulo_deseado[1]
                self.z = angulo_deseado[2]
                z_send = internal_height + delta
            
                moved = self.comunicacion.send_motor_data([self.brazo.q1, self.brazo.q2, z_send])
                if moved:
                    return True
                else:
                    return False
            else:
                print(is_valid[1])
                return False
        else:
            state = self.get_state(pose_deseada)
            if self.is_valid_state(state)[0]:
                internal_height = self.comunicacion.send_request("z1", 4)
                delta = state[2] - self.z
                self.brazo.q1 = state[0]
                self.brazo.q2 = state[1]
                self.z = state[2]
                z_send = internal_height + delta
                
                moved = self.comunicacion.send_motor_data([self.brazo.q1, self.brazo.q2, z_send])
                if moved:
                    return True
                else:
                    return False
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

        search_pose = np.array([120,129,self.max_z])
        self.move([0,0,0],angulo_deseado = search_pose)
        #Centra la herida en la vista de la camara
        image = self.comunicacion.read_usb()
        wound_center = self.ojos.find_wound_center_pixel(image)
        if wound_center is None:
            return False
        pixel_wound = flip_y(wound_center, self.ojos.MAX_Y)
        pixel_center = self.ojos.center

        move_vector = pixel_center - pixel_wound
        distance = np.linalg.norm(move_vector)
        
        while distance > self.ojos.precision:
            
            new_pose = self.brazo.pose() + self.Kp_2D*move_vector
            send_pose = np.array([new_pose[0],new_pose[1], self.z])
            print(send_pose)
            self.move(send_pose)

            image = self.comunicacion.read_usb()
            pixel_wound = flip_y(self.ojos.find_wound_center_pixel(image), self.ojos.MAX_Y)
            move_vector = pixel_center - pixel_wound
            distance = np.linalg.norm(move_vector)

        return True
    
    def move_pencil_to_object(self, target: int):
        
        #Mueve el lapiz a la posición deseada en pixeles
        imagen = self.comunicacion.read_usb()
        pixel_pencil = flip_y(self.ojos.find_pencil_pixel(imagen), self.ojos.MAX_Y)
        pixel_deseado = self.ojos.find_object(target, imagen)
        pixel_deseado = flip_y(pixel_deseado, self.ojos.MAX_Y)
        altura_actual = self.comunicacion.send_request("z2", 4)

        move_vector_2D = pixel_deseado - pixel_pencil
        pix_to_mm_ratio = self.ojos.pixel_to_mm(altura_actual)
        move_vector_2D = move_vector_2D * pix_to_mm_ratio
        move_vector = np.array([move_vector_2D[0], move_vector_2D[1], altura_actual - self.pencil_height])
        distance = np.linalg.norm(move_vector)
        
        while distance > self.precision:

            new_pose = self.pose() + self.Kp_3D*move_vector
            moved = self.move(new_pose)
            while not moved:
                self.move(new_pose)

            imagen = self.comunicacion.read_usb()
            pixel_deseado = self.ojos.find_object(target, imagen)
            altura_actual = self.comunicacion.send_request("z2", 4)

            move_vector_2D = pixel_deseado - pixel_pencil
            pix_to_mm_ratio = self.ojos.pixel_to_mm(altura_actual)
            move_vector_2D = move_vector_2D * pix_to_mm_ratio
            move_vector = np.array([move_vector_2D[0], move_vector_2D[1], altura_actual - self.pencil_height])
            distance = np.linalg.norm(move_vector)
        
        return True
    
    def move_pencil_to_pos(self, pos_deseada):
        
        moved = self.move(pos_deseada)
        while not moved:
            self.move(pos_deseada)

        return True
    
    def stitch_pix(self):

        n_stitches = len(self.ojos.initial_stitches)

        for i in range(n_stitches):
            touched = self.move_pencil_to_object(i)
            if touched:
                time.sleep(WAIT)
                moved = self.move_pencil_to_pos(self.pose() + np.array([0, 0, 50]))
                if moved:
                    time.sleep(WAIT)
                else:
                    print("Error al alejar el lapiz")
            else:
                print("Error al tocar el punto de sutura")

        self.finished = True

    def stitch_pos(self):

        pix_stitches = self.ojos.initial_stitches
        pos_stitches = self.ojos.estimate_stitch_pose(pix_stitches, self.comunicacion.send_request("z2", 4), self.pose())
        self.move_to_pos_list(pos_stitches)


    def move_to_pos_list(self, pos_list):

        for pos in pos_list:
            moved = self.move_pencil_to_pos(pos)
            if not moved:
                print("Error al mover a la posición deseada")
                return False
            else: 
                time.sleep(WAIT)
                #Alejar el lapiz hacia arriba para que pueda moverse
                moved = self.move_pencil_to_pos(self.pose() + np.array([0, 0, 50])) 
                if moved:
                    time.sleep(WAIT)
                else:
                    print("Error al alejar el lapiz")
                    return False
        return True

    

if __name__ == "__main__":

    print("Hello World")
