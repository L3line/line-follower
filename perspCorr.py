# import the necessary packages
import io
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
square = np.array([[-50,   271],
                   [-50, 171],
                   [50,     271],
                   [50,   171]],
                  dtype="float32")
xoffset = 200
yoffset = 0
ppmm = 2 #pixels per millimeter

src = np.array([[566, 437],
                [385, 1013],
                [1194, 460],
                [1339, 1058]],
               dtype="float32")

dest = (square + np.array([xoffset, yoffset], dtype="float32")) * ppmm
#for i in src:
    # image = cv2.circle(image, i, 10, (0, 255, 0))
#use M for debug viewing, perspMatrix for actual usage
M = cv2.getPerspectiveTransform(src, dest)

perspMatrix = cv2.getPerspective(src, square)

#image = cv2.imread("./photos/25-04-2017_12:46:54.jpg")
resheight = 600
reswidth = 800
#scalefac = resheight/image.shape[0]
#warped = cv2.warpPerspective(image, M, (reswidth, resheight))
#smallimage = cv2.resize(image, (reswidth, resheight))
#resolution = (3280, 2464)
if __name__ == "__main__"
    resolution = (1640, 1232)
    with PiCamera() as camera:
        camera.resolution = resolution
        rawCapture = PiRGBArray(camera, size=resolution)
        for frame in camera.capture_continuous(rawCapture, format ='bgr', use_video_port = True):
            image = frame.array
            warpCap = cv2.warpPerspective(image, M, (reswidth, resheight))
    
            ######################
            ### OPENCV + MOTOR ###
            ###   STUFF HERE   ###
            ######################
    
    
            cv2.imshow("Image", warpCap)
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
    
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

