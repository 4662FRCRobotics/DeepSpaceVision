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
# from scipy import ndimage

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance

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
server = False
cameraConfigs = []

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

    #server = MjpegServer("CVstream")

    camSink = inst.getVideo(name=config.name)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    #if config.streamConfig is not None:
    #    server.setConfigJson(json.dumps(config.streamConfig))

    camSource = inst.putVideo(cameraName, 160, 120)

    #server.setSource(camSource)

    return camera, camSink, camSource

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


    PI_ADDRESS = "10.46.62.10"
    CAMERA_ADDRESS_DELIMETER = ","

    
    
    # start cameras
    cameras = []
    i = 1

    for cameraConfig in cameraConfigs:

        PORT = 1180 + 2 * i

        # NetworkTablesInstance.getDefault().getEntry("/CameraPublisher/PiCamera/Streams").setStringArray("mjpg:http://%s:%s/?action=stream" % (PI_ADDRESS, PORT))

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

    # loop forever
    print("Looping")
    while True:
        camera1, camera2 = cameras[:2]
        time1, img1 = camera1["sink"].grabFrame(img1)
        time2, img2 = camera2["sink"].grabFrame(img2)

        if time1 == 0:
            continue

        # image processing logic here
        
        
        
        # drawing a rectangle
        # cv2.rectangle(img, (25, 25), (100, 100), (255, 127, 0))

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
