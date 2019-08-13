#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys
import cv2
import numpy as np
import cscore
import random
import math
# from scipy import ndimage

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance, NetworkTables
from deepspacecargo import DeepSpaceCargo

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
cameraWidths = []
cameraHeights = []
server = False
cameraConfigs = []

DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi

CAMERA_FOV_Y = 40.2 # degrees
TARGET_HEIGHT = 5.75

CAMERA_OFFSET = 32

"""Report parse error."""
def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

"""Read single camera configuration."""
def readCameraConfig(config):
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

"""Read configuration file."""
def readConfig():
    global team
    global cameraWidths
    global cameraHeights
    global server

    # parse file
    try:
        with open(configFile, "rt") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        cameraWidths.append(camera["width"])
        cameraHeights.append(camera["height"])
        if not readCameraConfig(camera):
            return False

    return True

"""Start running the camera."""
def startCamera(config, i):
    print("Starting camera '{}' on {}".format(config.name, config.path))

    cameraName = "Camera_%s" % i

    inst = CameraServer.getInstance()

    # inst.addSwitchedCamera(cameraName)

    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=False)

    camSink = inst.getVideo(name=config.name)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    #if config.streamConfig is not None:
    #    server.setConfigJson(json.dumps(config.streamConfig))

    camSource = inst.putVideo(cameraName, 160, 120)

    return camera, camSink, camSource

# TARGET_HEIGHT_TIMES_CAMERA_HEIGHT = TARGET_HEIGHT * cameraHeights[0]
# HEIGHT_TO_DISTANCE_RATIO = math.tan(math.radians(CAMERA_FOV_Y))

def calculate_distance(target_pixel_height): # calculate distance of target from 

    # normalizedHeight = 2 * target_pixel_height / cameraHeights[0]
    # distance = TARGET_HEIGHT / (normalizedHeight * math.tan(math.radians(CAMERA_FOV_Y))) # distance from camera to target

    distance = TARGET_HEIGHT_TIMES_CAMERA_HEIGHT / ( target_pixel_height * HEIGHT_TO_DISTANCE_RATIO) # distance from camera to target

    print("Target Pixel Height: {}".format(target_pixel_height))

    return distance # - CAMERA_OFFSET # subtract off the distance from the camera to the front of the robot


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()


    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)

    print("Getting vision table")
    visionTable = NetworkTables.getTable("Vision")
    print("Got vision table")

    isTargetFoundEntry = visionTable.getEntry("isTargetFound")
    targetCountEntry = visionTable.getEntry("targetCount")
    boundingRectxywhEntry = visionTable.getEntry("boundingRectxywh")
    targetOffsetEntry = visionTable.getEntry("targetOffset")
    distanceToTargetEntry = visionTable.getEntry("distanceToTarget")

    '''
    isVisionOn = False
    print("Getting entry")
    
    isVisionOn = visionTable.getBoolean("isVisionOn", isVisionOn)
    testEntry = visionTable.getEntry("test")
    testEntry.setBoolean(True)

    print("isVisionOn: {}".format(isVisionOn))
    print(dir(visionTable))

    print(visionTable.getKeys())
    '''

    PI_ADDRESS = "10.46.62.10"
    CAMERA_ADDRESS_DELIMETER = ","
    
    # start cameras
    cameras = []
    i = 1

    for cameraConfig in cameraConfigs:

        PORT = 1180 + 2 * i

        camera, camSink, camSource = startCamera(cameraConfig, i)

        camera_dict = {
            "camera" : camera,
            "sink" : camSink,
            "source" : camSource
        }

        cameras.append(camera_dict)
        i += 1

    img1 = np.zeros(shape=(120, 160, 3), dtype=np.uint8)
    img2 = np.zeros(shape=(120, 160, 3), dtype=np.uint8)

    isVisionOn = False

    deepSpaceCargo = DeepSpaceCargo()

    TARGET_HEIGHT_TIMES_CAMERA_HEIGHT = TARGET_HEIGHT * cameraHeights[0]
    HEIGHT_TO_DISTANCE_RATIO = math.tan(math.radians(CAMERA_FOV_Y))

    # loop forever
    print("Looping")
    while True:
        camera1, camera2 = cameras[:2]
        time1, img1 = camera1["sink"].grabFrame(img1)
        time2, img2 = camera2["sink"].grabFrame(img2)

        if time1 == 0:
            continue

        # image processing logic here

        # color is in b, g, a
        
        isVisionOn = visionTable.getBoolean("isVisionOn", isVisionOn)

        isTargetFound = False
        objects_found = 0
        contours = []
        x_sum = 0
        x_avg = 0

        y_sum = 0
        y_avg = 0
        offset = 0

        distance = 0

        x,y,w,h = 0, 0, 0, 0

        if isVisionOn:

            deepSpaceCargo.process(img1) # take blobs that look like the reflective tape
            contours = deepSpaceCargo.filter_contours_output # take the result from the image processing
            objects_found = len(contours)

            #np.sort(contours, order='area')
            #sorted(contours, key=attrgetter('area'), reverse=True)

            if (objects_found >= 2):
                isTargetFound = True

                for i in range(2): # draw a bounding box around the first 2 blobs
                    r1 = cv2.boundingRect(contours[i])
                    x,y,w,h = r1

                    x_sum += x + w / 2
                    y_sum += h

                    cv2.rectangle(img1, (x, y), (x + w, y + h), (0, 255, 0))

                    print("Hieght: {}".format(h))

                '''for i in range(2, min(4, objects_found)): # draw bounding box around 2 other blobs if found
                    r2 = cv2.boundingRect(contours[i])
                    x2, y2, w2, h2

                    cv2.rectangle(img1, (x2, y2), (x2 + w2, y2 + h2), (0, 0, 255))'''

                x_avg = x_sum / 2
                y_avg = y_sum / 2
                camera1_width = cameraWidths[0]
                offset = x_avg - (camera1_width / 2)

                distance = calculate_distance(y_avg)

            # drawing a rectangle for manual targeting
            cv2.rectangle(img1, (105, 95), (215, 135), (0, 0, 0))

        targetOffsetEntry.setNumber(offset)
        isTargetFoundEntry.setBoolean(isTargetFound)
        targetCountEntry.setNumber(objects_found)
        boundingRectxywhEntry.setNumberArray([x, y, w, h])
        distanceToTargetEntry.setNumber(distance)

        # writing text
        # cv2.putText(img, "Epic", (1, 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 0))

        # changing individual pixels using python
        # for r in range(len(img)):
        #     for c in range(len(img[r])):
        #         pixel = img[r, c].astype(np.uint16)
        #         img[r, c] = np.array([255, pixel[1], pixel[2]], dtype=np.uint8)

        # update the camera feed with the edited image
        camera1["source"].putFrame(img1)
        camera2["source"].putFrame(img2)
