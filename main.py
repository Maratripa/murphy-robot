import cv2
import numpy as np
import serial
import time
from Effector import Effector
from SutureSegmentator import SutureSegmentator
from ESP32Serial import ESP32Serial

def start_process():
    capture = cv2.VideoCapture(0)
    suture_segmentator = SutureSegmentator()
    effector = Effector()
    #DESCOMENTAR PARA COMUNICARSE CON ESP32
    # TODO: Implementar comunicación ESP32 y Raspberry
    esp32_serial = ESP32Serial()

    while True:
        ret, frame = capture.read()
        frame = effector.draw_effector_position(frame)
        cv2.imshow('Camera', frame)
        key = cv2.waitKey(1) & 0xFF
        esp32_serial.send_data('hola')
        if key == ord('q'):
            break
        if key == ord('s'):
            frame_with_points, mask_red, frame, unique_points = suture_segmentator.draw_points(frame)
            print(unique_points)
            cv2.imshow('Points', frame_with_points)

def main():
    start_process()

if __name__ == "__main__":
    main()