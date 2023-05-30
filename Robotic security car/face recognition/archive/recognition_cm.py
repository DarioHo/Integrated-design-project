import face_recognition
import os
import cv2
import numpy as np
import glob
import dlib

from os import path

import serial

#Detect arduino serial path (Cater for different USB-Serial Chips)
if path.exists("/dev/ttyACM0"):
	arduino = serial.Serial(port = '/dev/ttyACM0',baudrate = 115200)
elif path.exists("/dev/ttyUSB0"):
	arduino = serial.Serial(port = '/dev/ttyUSB0',baudrate = 115200)
else:
	print("Please plug in the Arduino")
	exit()
if not(arduino.isOpen()):
    arduino.open()

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
video_capture = cv2.VideoCapture(get_jetson_gstreamer_source(), cv2.CAP_GSTREAMER)

# Load a sample picture and learn how to recognize it.

knownfaces_dir = "image/"

known_encodings = []
# face_name = []

images = glob.glob(knownfaces_dir+"*.jpeg")
for image in images:
    with open(image, 'rb') as file:
        img = face_recognition.load_image_file(file)
        # person_name = image.strip(".jpeg").replace(knownfaces_dir,"").replace("-"," ")

        known_encodings.append(face_recognition.face_encodings(img)[0])
        # face_name.append(person_name)

# Initialize some variables
face_locations = []
face_encodings = []
# face_names = []
process_this_frame = True

while video_capture.isOpened():
    # Grab a single frame of video
    ret, frame = video_capture.read()

    # Only process every other frame of video to save time
    if process_this_frame:

        is_unknown = False

        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        # face_names = []
        for idx, face_encoding in enumerate(face_encodings):
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_encodings, face_encoding, 0.4)

            if all(matches) == False:
                is_unknown = True
                unknown_location = face_locations[idx]
                print("Unknown!")
                break


            # name = "Unknown"
            # face_distances = face_recognition.face_distance(known_encodings, face_encoding)
            # print(face_distances)
            
            # best_match_index = np.argmin(face_distances)
            # if matches[best_match_index]:
            #     name = face_name[best_match_index]

            # face_names.append(name)

    process_this_frame = not process_this_frame

    # Display the results
    # for (top, right, bottom, left), name in zip(face_locations, face_names):
    if is_unknown == True:
        top, right, bottom, left = unknown_location
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        top *= 10
        right *= 10
        bottom *= 10
        left *= 10

        center_x = (right+left)//2
        center_y = (top+bottom)//2

        (h,w) = frame.shape[:2]
        # cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)

        if (center_x > w//2+30):
            print("Go Right!")

        elif (center_x < w//2-30):
            print("Go Left!")
        
        else:
            print("Ok!")
        # # Draw a box around the face
        # cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        # # Draw a label with a name below the face
        # cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        # font = cv2.FONT_HERSHEY_DUPLEX
        # cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

    # Display the resulting image
    cv2.imshow('Video', frame)

    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()
arduino.close()