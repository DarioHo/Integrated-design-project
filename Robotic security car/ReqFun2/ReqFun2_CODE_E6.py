#!/usr/bin/python3
#
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

# ----------------------------------------------------------------------
# Data descriptors for Detection = <type 'jetson.inference.detectNet.Detection'>
# Area - Area of bounding box
# Bottom - Bottom bounding box coordinate
# Center - Center (x,y) coordinate of bounding box
# ClassID - Class index of the detected [object]
# Confidence - Confidence value of the detected [object]
# Height - Height of bounding box
# Instance - Instance index of the detected [object]
# Left -  Left bounding box coordinate
# Right -  Right bounding box coordinate
# Top -   Top bounding box coordinate
# Width -  Width of bounding box
# ----------------------------------------------------------------------

import jetson.inference
import jetson.utils

import argparse
import sys
from os import path
import serial
import time


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

#Variables for command and control
width = 1280 # width of the camera view
height = 720 # height of the camera view
objX = width/2 # target x-coordinate
objY = height/2 # target y-coordinate

error_tolerance = 10 # tolerance for centering the object
confidence_threshold = 0.6 # minimum confidence needed to send out signals to arduino 

tilt_offset = 30
pan_prev =90
tilt_prev = 90
pan =90
tilt = 90
pan_max = 100
pan_min = 80
tilt_max = 100
tilt_min = 80

# ----------------------------------------------------------------------
# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", \
    formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +\
    jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

#More arguments (For commandline arguments)
parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use")  

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

#Print help when no arguments given
try:
    opt = parser.parse_known_args()[0]
except:
    print("")
    parser.print_help()
    sys.exit(0)
# ----------------------------------------------------------------------

# load the object detection network
net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)

# create video sources & outputs
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)

# ----------------------------------------------------------------------

# process frames until the user exits
while True:

    # reset the buffer
    arduino.reset_output_buffer()

    # capture the next image
    img = input.Capture()

    # detect objects in the image (with overlay)
    detections = net.Detect(img, overlay=opt.overlay)

    # print the detections
    print("detected {:d} objects in image".format(len(detections)))
    
    #Initialize the object coordinates and area
    objX = width/2
    objY = height/2
    Area = 0
    Confidence = 0.0
    
    #Find largest detected objects (in case of deep learning confusion)
    for detection in detections:
        print(detection)
        if(int(detection.Area)>Area):
            objX = int(detection.Center[0])
            objY = int(detection.Center[1])
            Area = int(detection.Area)
            Confidence = detection.Confidence # optional for a threshold

    # ----------------------------------------------------------------------
  
    # Camera Adjustments 

    #Determine the adjustments needed to make to the camera
    panOffset = objX - (width/2)   # pan to make it center
    tiltOffset = objY - (height/2) # tilt to make it center
    
    #Puting the values in margins
    if (abs(panOffset)>error_tolerance):
        pan = pan-panOffset/100
    if (abs(tiltOffset)>error_tolerance):
        tilt = tilt+tiltOffset/100
    if pan>pan_max:
        pan = pan_max
    if pan<pan_min:
        pan=pan_min
    if tilt>tilt_max:
        tilt=tilt_max
    if tilt<tilt_min:
        tilt=tilt_min
        
    #Rounding them off
    pan = int(pan)
    tilt = int(tilt) + tilt_offset
    Area = int(Area)

    #Setting up command string
    myString = '(' +  str(pan) + ',' + str(tilt) + ',' + str(Area) + ')'
    print("myString = %s" %myString)

    # ----------------------------------------------------------------------

    # Print strings sent by arduino, if there's any
    # decode every line in utf-8
    if arduino.inWaiting():
        print("From Arduino serial: %s" %arduino.readline().decode('utf-8'))
        arduino.flushInput()
        arduino.flushOutput()

    panOffset = objX - (width/2)  
    # panOffset > 0 : the station is at the car's right-hand side
    # panOffset = 0 : the station is at the car's center
    # panOffset < 0 : the station is at the car's left-hand side

    # Allow error tolerance for centering the object to reduce trivial movements
    if (abs(panOffset) > error_tolerance): 
        if (Area > 0 and Area < 300000) and Confidence > confidence_threshold: 
            # if the target detected has an area with confidence level greater than threshold
            if panOffset > 0:
                arduino.write("L") # send signals to shift left
            elif panOffset < 0:
                arduino.write("R") # send signals to shift right
            else: # panOffset == 0 (at center)
                arduino.write("S") # send signals to stop shifting left or right

    # render the image
    smallImg = jetson.utils.cudaAllocMapped(width=img.width*0.5, height=img.height*0.5, format=img.format)
    jetson.utils.cudaResize(img, smallImg)
    output.Render(smallImg)

    # update the title bar
    output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))
    
    # print out performance info
    net.PrintProfilerTimes()

    # exit on input/output EOS
    if not input.IsStreaming() or not output.IsStreaming():
        break


