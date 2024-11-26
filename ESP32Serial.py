import time 
import serial
import datetime
import threading

class ESP32Serial:
    def __init__(self):
        self.ser = serial.Serial ("/dev/ttyS0", 115200)
    
    def send_data(self, msg):
        if (msg == ""):
            return

        self.ser.write(msg.encode("utf-8"))

    def read_data(self):
        return self.ser.readline().decode("utf-8")