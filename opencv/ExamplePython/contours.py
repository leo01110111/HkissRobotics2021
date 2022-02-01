
# these are the libraries I know we'll need so far
import numpy as np
import time
import math
import cv2
import imutils
from imutils.video import VideoStream

# the "0" argument refers to the 0th (first to humans) camera atached to the computer
vs = VideoStream(1).start()
# makes sure everything is ready before we move on
time.sleep(2.0)
cv2.imshow("Video Feed", vs.read())

frame = vs.read()
height = round(frame.shape[0])
width = round(frame.shape[1])
center = (round(frame.shape[1] / 2),round(frame.shape[0] / 2))

# main loop, python is slow, so we want to put as little as possible in here
while True:

    # read the the contents of the stream buffer as a still image for this frame
    frame = vs.read()
    time.sleep(0.05)
    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    #All I know is that the test version used this sytax and worked
    thresh = cv2.inRange(frameHSV,(40, 100, 0), (90,225,150))

    thresh = cv2.dilate(thresh, None, iterations=2)
    cnts = cv2.findContours(thresh.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts) != 0:
        mainTarget = max(cnts,key=cv2.contourArea)

        if cv2.contourArea(max(cnts,key=cv2.contourArea)) < 200:
            continue

        #get center of the contour
        M = cv2.moments(mainTarget)
        contX = int(M["m10"]/M["m00"])
        contY = int(M["m01"]/M["m00"])
        
        #(x, y, w, h) = cv2.boundingRect(mainTarget)
        #midpointFLOAT = ((x+(w/2)),(y+(h/2)))
        #midpointINT = (round(midpointFLOAT[0]),round(midpointFLOAT[1]))

        #Changing to use contour center instead of rect center
        midpointFLOAT = (contX,contY)
        midpointINT = (round(midpointFLOAT[0]),round(midpointFLOAT[1]))

        xCenterOffset = center[0] - (width - midpointINT[0])
        if xCenterOffset <= 0:
            offset = "Target Offset: " + str(xCenterOffset) + " from center"

        else:
            offset = "Target Offset: +" + str(xCenterOffset) + " from center"

        #cv2.rectangle(frame, (x, y), (x + w, y + h), (0,255,0), thickness=2)
        cv2.drawContours(frame, mainTarget, -1, (255, 0, 0), 2)
        cv2.rectangle(frame, (1,1), (1,1), (0, 0, 0))
        cv2.drawMarker(frame, midpointINT, (0,255,0), cv2.MARKER_CROSS, thickness=2)
    else:
        offset = "No Target!"

    cv2.putText(frame, offset, (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "Width: {} (Center: {}) ".format(width, center[0]),
        (10, 35),cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

    # show the stream with the window title "Video Feed"
    cv2.imshow("Video Feed", frame)
    cv2.imshow("Threshold", thresh)

    # records if a key was pressed this iteration, if it was "Q" we break the loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# cleanup the camera and close any open windows
vs.stop()
cv2.destroyAllWindows()

