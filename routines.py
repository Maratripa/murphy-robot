from Robot_parts import Brazo, Ojos, Comunicacion, Robot
import numpy as np
import time
import cv2
from Algorithms import pose_calc

#Distancias en mm
#Angulos en grados
PARAMETROS = {
"L1": 200,
"L2": 100,
"precision_brazo": 5,
"diferencia_lapiz": 10,
"distancia_camara": 20,
"min_q1": 0,
"max_q1": 270,
"min_q2": 0,
"max_q2": 258,
"Imagen_width": 640,
"Imagen_height": 480,
"precision_ojos": 5,
"FOV": 55,
"puerto_serial": "/dev/ttyS0",
"time_out": 1,
"precision_robot": 5,
"altura_lapiz": 107,
"screw_thread": 2,
"Kp_2D": 1,
"Kp_3D": 0.2,
"min_z": 10,
"max_z": 127,
"tree_num": 5,
"search_depth": 10,
"min_matches": 10
}
def precision_test(robot: Robot, pose, num = 5):

    robot.move(np.array([0,0,100]))
    for i in range(num):
        robot.move(pose)
        time.sleep(1)
        random_pose_x = np.random.uniform(-20,20)
        random_pose_y = np.random.uniform(180,220)
        random_pose_z = np.random.uniform(0,70)
        random_pose = np.array([random_pose_x, random_pose_y, random_pose_z])
        robot.move(random_pose)
        time.sleep(1)


def work_area(robot: Robot):

    point1 = np.array([-125,119,100])
    point2 = np.array([-125,239,100])
    point3 = np.array([125,239,100])
    point4 = np.array([125,239,100])

    points = [point1, point2, point3, point4]
    for point in points:
        robot.move(point)
        time.sleep(3)


def start(robot: Robot):

    home = robot.comunicacion.send_home()
    robot.brazo.q1 = 0
    robot.brazo.q2 = 0
    robot.z = robot.comunicacion.send_request("z1", 10)
    
    found = robot.center_wound()

    return True

def find_wound_initial(robot: Robot):

    imagen = robot.comunicacion.read_usb()

    wound = robot.ojos.find_wound_geometry(imagen)
    while wound == None:
        wound = robot.ojos.find_wound_geometry(imagen)
    robot.ojos.save_wound_geometry(wound)

    height = robot.comunicacion.send_request("z2", 4)
    

    found_stitches = robot.ojos.define_stiching_points(imagen, height)
    while not found_stitches:
        found_stitches = robot.ojos.define_stiching_points(imagen)

    return True

def stitching_pixel(robot: Robot):
    started = start(robot)
    if not started:
        print("Error al iniciar el robot")
    else:
        found_wound = find_wound_initial(robot)
        if not found_wound:
            print("Error al encontrar la herida")
        else:
            print("Robot listo para coser")
            while not robot.finised:
                robot.stitch_pix()

            print("Terminado")
            stop = robot.comunicacion.send_stop()
            while not stop:
                robot.comunicacion.send_stop()
            print("Robot detenido")

    return True

def stitching_pos(robot: Robot):
    started = start(robot)
    if not started:
        print("Error al iniciar el robot")
    else:
        found_wound = find_wound_initial(robot)
        if not found_wound:
            print("Error al encontrar la herida")
        else:
            print("Robot listo para coser")
            while not robot.finised:
                robot.stitch_pos()

            print("Terminado")
            #Sube 50 mm el lapiz para que no se quede en la herida
            robot.move_pencil_to_pos(robot.pose() + np.array([0, 0, 50]))
            #Detiene el robot
            stop = robot.comunicacion.send_stop()
            while not stop:
                robot.comunicacion.send_stop()
            print("Robot detenido")

    return True

def wound_testing(robot: Robot, image, height: float):

    found_stitches = robot.ojos.define_stiching_points(image, height)
    while not found_stitches:
        found_stitches = robot.ojos.define_stiching_points(image)





if __name__ == "__main__":

    brazo = Brazo(PARAMETROS["L1"], PARAMETROS["L2"], PARAMETROS["precision_brazo"], PARAMETROS["diferencia_lapiz"], PARAMETROS["distancia_camara"], PARAMETROS["min_q1"], PARAMETROS["max_q1"], PARAMETROS["min_q2"], PARAMETROS["max_q2"])        
    ojos = Ojos(PARAMETROS["Imagen_width"], PARAMETROS["Imagen_height"], PARAMETROS["precision_ojos"], PARAMETROS["FOV"], PARAMETROS["tree_num"], PARAMETROS["search_depth"], PARAMETROS["min_matches"])
    comunicacion = Comunicacion(PARAMETROS["puerto_serial"], PARAMETROS["time_out"])
    murphy = Robot(brazo, ojos, comunicacion, PARAMETROS["precision_robot"], PARAMETROS["altura_lapiz"], PARAMETROS["screw_thread"], PARAMETROS["Kp_2D"], PARAMETROS["Kp_3D"], PARAMETROS["min_z"], PARAMETROS["max_z"])

    time.sleep(1) #Espera 1 segundo despues de prender para asegurarse que todo prenda bien
    
    #murphy.comunicacion.send_home()
    #murphy.comunicacion.send_motor_data((70,100,120))

    
    # info = {"height": 80, "q1": 70, "q2": 100, "L1": murphy.brazo.L1, "L2": murphy.brazo.L2, "pencil_diff": PARAMETROS["diferencia_lapiz"], "pencil_height": murphy.pencil_height}
    # base_height = murphy.comunicacion.send_request("z1", 10)
    # efector_height = murphy.comunicacion.send_request("z2", 10)
    # delta = efector_height - base_height
    # info["height"] = info["height"] + delta
    # print(base_height)
    # print(efector_height)
    # go_to = pose_calc(3,info)
    # print(go_to)
    # murphy.move(go_to)
    
    # image = murphy.comunicacion.read_usb()
    # efector_height = murphy.comunicacion.send_request("z2", 10)
    
    # wound_testing(murphy, image, efector_height)

    # stitches = murphy.ojos.initial_stitches
    # stitches_pos = murphy.ojos.estimate_stitch_pose(stitches, efector_height, murphy.pose())
    # print(f"murphy pose: {murphy.pose()}")
    # print(f"stitches pose:\n{stitches_pos}")
    

    """
    efector_height = 197

    image = murphy.comunicacion.read_usb()
    wound_testing(murphy, image, efector_height)
    """
    
    #murphy.center_wound()

    work_area(murphy)
    