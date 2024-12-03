from Robot_parts import Brazo, Ojos, Comunicacion, Robot
import numpy as np
import time

#Distancias en mm
#Angulos en grados
PARAMETROS = {
"L1": 200,
"L2": 50,
"precision_brazo": 5,
"diferencia_lapiz": 10,
"distancia_camara": 20,
"min_q1": 0,
"max_q1": 180,
"min_q2": 0,
"max_q2": 180,
"Imagen_width": 640,
"Imagen_height": 480,
"precision_ojos": 5,
"FOV": 60,
"puerto_serial": "/dev/ttyS0",
"time_out": 1,
"precision_robot": 5,
"altura_lapiz": 10,
"screw_thread": 2,
"Kp_2D": 0.1,
"Kp_3D": 0.2,
"min_z": 10,
"max_z": 200,
}

def start(robot: Robot):

    home = robot.comunicacion.send_home()
    while not home:
        home = robot.comunicacion.send_home()
    robot.brazo.q1 = 0
    robot.brazo.q2 = 0
    robot.z = robot.comunicacion.send_request("z1", 4)
    
    found = robot.center_wound()
    while not found:
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
            #Sube 100 mm el lapiz para que no se quede en la herida
            robot.move_pencil_to_pos(robot.pose() + np.array([0, 0, 100]))
            #Detiene el robot
            stop = robot.comunicacion.send_stop()
            while not stop:
                robot.comunicacion.send_stop()
            print("Robot detenido")

    return True


if __name__ == "__main__":

    brazo = Brazo(PARAMETROS["L1"], PARAMETROS["L2"], PARAMETROS["precision_brazo"], PARAMETROS["diferencia_lapiz"], PARAMETROS["distancia_camara"], PARAMETROS["min_q1"], PARAMETROS["max_q1"], PARAMETROS["min_q2"], PARAMETROS["max_q2"])
    ojos = Ojos(PARAMETROS["Imagen_width"], PARAMETROS["Imagen_height"], PARAMETROS["precision_ojos"], PARAMETROS["FOV"])
    comunicacion = Comunicacion(PARAMETROS["puerto_serial"], PARAMETROS["time_out"])
    murphy = Robot(brazo, ojos, comunicacion, PARAMETROS["precision_robot"], PARAMETROS["altura_lapiz"], PARAMETROS["screw_thread"], PARAMETROS["Kp_2D"], PARAMETROS["Kp_3D"], PARAMETROS["min_z"], PARAMETROS["max_z"])

    time.sleep(5) #Espera 5 segundos despues de prender para asegurarse que todo prenda bien
    initial_time = time.time()
    stitching_pixel(murphy)
    final_time = time.time()

    print("Tiempo total: ", final_time - initial_time)