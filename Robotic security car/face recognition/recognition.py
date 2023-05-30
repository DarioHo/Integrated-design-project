from re import T
from selectors import EpollSelector
import face_recognition
import os
import cv2
import numpy as np
import glob
import dlib
import time

from os import path

import serial

#Detect arduino serial path (Cater for different USB-Serial Chips)
# if path.exists("/dev/ttyACM0"):
# 	arduino = serial.Serial(port = '/dev/ttyACM0',baudrate = 115200)
# elif path.exists("/dev/ttyUSB0"):
# 	arduino = serial.Serial(port = '/dev/ttyUSB0',baudrate = 115200)
# else:
# 	print("Please plug in the Arduino")
# 	exit()
# if not(arduino.isOpen()):
#     arduino.open()

def get_jetson_gstreamer_source(capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=60, flip_method="rotate-180"):
    """
    Return an OpenCV-compatible video source description that uses gstreamer to capture video from the camera on a Jetson Nano
    """
    return (
            f'nvarguscamerasrc ! video/x-raw(memory:NVMM), ' +
            f'width=(int){capture_width}, height=(int){capture_height}, ' +
            f'format=(string)NV12, framerate=(fraction){framerate}/1 ! ' +
            f'nvvidconv flip-method={flip_method} ! ' +
            f'video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! ' +
            'videoconvert ! video/x-raw, format=(string)BGR ! appsink'
            )
print(dlib.DLIB_USE_CUDA)

# Get a reference to webcam #0 (the default one)
# video_capture = cv2.VideoCapture(get_jetson_gstreamer_source(), cv2.CAP_GSTREAMER)
video_capture = cv2.VideoCapture(0)

knownfaces_dir = "image/"
known_encodings = []

images = glob.glob(knownfaces_dir+"*.jpeg")
for image in images:
    with open(image, 'rb') as file:
        img = face_recognition.load_image_file(file)
        known_encodings.append(face_recognition.face_encodings(img)[0])


# Initialize some variables
face_locations = []
face_encodings = []
process_this_frame = True

# face_cascades = cv2.CascadeClassifier("model/opencv-cascades.xml")
Fps = 0
t = time.time()

while video_capture.isOpened():
    _, frame = video_capture.read()

    if process_this_frame:

        is_unknown = False

        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        for idx, face_encoding in enumerate(face_encodings):
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_encodings, face_encoding, 0.4)

            if all(matches) == False:
                is_unknown = True
                unknown_location = face_locations[idx]
                unknown_encoding = face_encoding
                break

    process_this_frame = not process_this_frame

    dt = time.time()-t
    t = time.time()
    fps = 1/dt
    Fps = 0.9*Fps + 0.1*fps
    Fps = round(Fps,2)

    if is_unknown == True:
        top, right, bottom, left = unknown_location
        
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        # top *= 4
        # right *= 4
        # bottom *= 4
        # left *= 4
        # center_x = (right+left)//2
        # center_y = (top+bottom)//2

        center_x = (right+left)*2
        center_y = (top+bottom)*2

        # (h,w) = frame.shape[:2]
        # h = 720, w = 1280
        # cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)

        # threshold -> = 1280/2+30
        # threshold <- = 1280/2-30

        if (center_x > 670):
            # print("Go Right!")
            arduino.write('R'.encode())

        elif (center_x < 610):
            # print("Go Left!")
            arduino.write('L'.encode())
        
        elif (center_y > 390):
            # print("Go Forward!")
            arduino.write('F'.encode())

        elif (center_y < 330):
            # print("Go Back!")
            arduino.write('B'.encode())
        
        else:
            # print("Stop")
            arduino.write('S'.encode())      
            
    
    cv2.putText(frame,str(Fps)+" fps",(15,30),cv2.FONT_HERSHEY_DUPLEX, 1, (0,255,255),2)
    # Display the resulting image
    cv2.imshow('Video', frame)


    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()
arduino.close()