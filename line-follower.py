import io
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import steppers
from imgProcess import imgProcess
#image = cv2.imread("./photos/25-04-2017_12:46:54.jpg")
resheight = 600
reswidth = 800
#scalefac = resheight/image.shape[0]
#warped = cv2.warpPerspective(image, M, (reswidth, resheight))
#smallimage = cv2.resize(image, (reswidth, resheight))
#resolution = (3280, 2464)
resolution = (1640, 1232)
motors = steppers.stepperDrive()
with PiCamera() as camera:
    camera.resolution = resolution
    rawCapture = PiRGBArray(camera, size=resolution)
    for frame in camera.capture_continuous(rawCapture, format ='bgr', use_video_port = True):
        image = frame.array
        #########################
        ### findcontours here ###
        #########################
        path = imgProcess(image)
        print(path)
        ######################
        ### OPENCV + MOTOR ###
        ###   STUFF HERE   ###
        ######################


        #cv2.imshow("Image", warpCap)
        #key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        #if key == ord("q"):
        #    break
