import socket
import json
from threading import Thread

from Robot_parts import Brazo, Ojos, Comunicacion, Robot
import numpy as np
import time
from routines import PARAMETROS, precision_test, work_area


class Server(Thread):
    def __init__(self, callback, host="localhost", port=6969):
        super().__init__()
        self.host = host
        self.port = port
        self.socket_servidor = None
        self.conectado = False

        self.client_callback = callback

    def run(self):
        self.socket_servidor = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_servidor.bind((self.host, self.port))
        self.socket_servidor.listen()
        self.conectado = True

        while self.conectado:
            try:
                client_socket, _ = self.socket_servidor.accept()
                thread_cliente = Thread(target=self.escuchar_cliente, args=(client_socket,))
                thread_cliente.start()
            except ConnectionError as e:
                print(f"Error: {e}")
                return

    def escuchar_cliente(self, socket_cliente: socket.socket):
        while True:
            try:
                mensaje = self.recibir_mensaje(socket_cliente)
                self.client_callback(mensaje)
            except ConnectionError as e:
                self.eliminar_cliente(socket_cliente)
                return

    def recibir_mensaje(self, socket_cliente: socket.socket):
        recivido = socket_cliente.recv(1024)
        decoded = recivido.decode("utf-8")
        # return json.loads(decoded)
        message = decoded[decoded.find('{'):]
        print(f"Mensaje decodeado: {json.loads(message)}")
        return ""

    def eliminar_cliente(self, socket_cliente: socket.socket):
        socket_cliente.close()


if __name__ == "__main__":
    brazo = Brazo(PARAMETROS["L1"], PARAMETROS["L2"], PARAMETROS["precision_brazo"], PARAMETROS["diferencia_lapiz"], PARAMETROS["distancia_camara"], PARAMETROS["min_q1"], PARAMETROS["max_q1"], PARAMETROS["min_q2"], PARAMETROS["max_q2"])        
    ojos = Ojos(PARAMETROS["Imagen_width"], PARAMETROS["Imagen_height"], PARAMETROS["precision_ojos"], PARAMETROS["FOV"], PARAMETROS["tree_num"], PARAMETROS["search_depth"], PARAMETROS["min_matches"])
    comunicacion = Comunicacion(PARAMETROS["puerto_serial"], PARAMETROS["time_out"])
    murphy = Robot(brazo, ojos, comunicacion, PARAMETROS["precision_robot"], PARAMETROS["altura_lapiz"], PARAMETROS["screw_thread"], PARAMETROS["Kp_2D"], PARAMETROS["min_z"], PARAMETROS["max_z"])

    def callback(message):
        comando = message['command']

        print(f"Comando recivido: {comando}")

        if comando == "startRoutine":
            pass

        elif comando == "testArea":
            work_area(murphy)

        elif comando == "testPosition":
            q1 = message['q1']
            q2 = message['q2']
            z = message['z']

            murphy.move(None, np.array([q1, q2, z]))

        elif comando == "testRepeatability":
            precision_test(murphy, np.array([30, 200, 50]), 5)

        else:
            print(f"Comando desconocido: {comando}")


    server = Server(callback, 'localhost', 6969)
    server.start()
    server.join()
    