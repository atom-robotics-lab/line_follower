from collections import deque
import numpy as np
import cv2


yellowlower = (29,86,6)
yellowupper = (64,255,255)


pts = deque(maxlen = 32)
counter = 0
(dx,dy) = (0,0)

direction = ""


cap = cv2.VideoCapture(0)


while True :



    ret , frame = cap.read()

    if ret :

        hsv = cv2.cvtColor(frame , cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv , yellowlower , yellowupper)

        cv2.imshow("mask" , mask)
        cv2.imshow("feed" , frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break 




cap.release()

cv2.destroyAllWindows()