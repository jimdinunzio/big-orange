# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 18:40:21 2021

@author: LattePanda
"""

import time
from playsound import playsound
import cv2


def show_picture_mat(mat):
    cv2.imshow("Snapshot", mat)
    cv2.waitKey(10000)
    cv2.destroyWindow("Snapshot")

def take_picture(filename):
    camera_port = 0
    camera = cv2.VideoCapture(camera_port)
    time.sleep(0.5)  # If you don't wait, the image will be dark
    return_value, image = camera.read()
    playsound("sounds/camera-shutter.wav")
    print("saveing picture to " + "pictures_taken/"+filename)
    cv2.imwrite("pictures_taken/"+filename, image)
    del(camera)  # so that others can use the camera as soon as possible
    return image

mat = take_picture("capture_" + time.ctime().replace(' ', '-', -1).replace(":","-",-1) +".jpg")
show_picture_mat(mat)

