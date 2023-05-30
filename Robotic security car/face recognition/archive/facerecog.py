import face_recognition
import os
import cv2
import numpy as np
import glob

from os import path

# import serial

# #Detect arduino serial path (Cater for different USB-Serial Chips)
# if path.exists("/dev/ttyACM0"):
# 	arduino = serial.Serial(port = '/dev/ttyACM0',baudrate = 115200)
# elif path.exists("/dev/ttyUSB0"):
# 	arduino = serial.Serial(port = '/dev/ttyUSB0',baudrate = 115200)
# else:
# 	print("Please plug in the Arduino")
# 	exit()
# if not(arduino.isOpen()):
#     arduino.open()

# Get a reference to webcam #0 (the default one)
video_capture = cv2.VideoCapture(0)

# Load a sample picture and learn how to recognize it.

knownfaces_dir = "/users/yokolau/Desktop/testdir/"

known_encodings = []
known_names = []

images = glob.glob(knownfaces_dir+"*.jpg")
for image in images:
    with open(image, 'rb') as file:
        img = face_recognition.load_image_file(file)
        person_name = image.strip(".jpg").replace(knownfaces_dir,"").replace("-"," ")
        known_encodings.append(face_recognition.face_encodings(img)[0])
        known_names.append(person_name)

# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True

while video_capture.isOpened():
    # Grab a single frame of video
    ret, frame = video_capture.read()
    is_Stranger = False

    # Only process every other frame of video to save time
    if process_this_frame:
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_encodings, face_encoding, 0.5)
            name = "Unknown"

            face_distances = face_recognition.face_distance(known_encodings, face_encoding)
            # print(matches)
            # if all(matches) == False:
            #     is_Stranger = True
            #     stranger_encoding = face_encodings[matches.index(False)]
            #     stranger_location = face_locations[face_encodings.index(face_encoding)]
            #     print(stranger_location)
            #     break


            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_names[best_match_index]

            face_names.append(name)



    process_this_frame = not process_this_frame

    if is_Stranger is True:
        # Display the results
        for (top, right, bottom, left), name in zip(stranger_location, name):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

    # Display the resulting image
    cv2.imshow('Video', frame)



    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()