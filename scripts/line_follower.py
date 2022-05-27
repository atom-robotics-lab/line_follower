#! /usr/bin/env python3
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
import rospy
import math



class Line_detection():
    def __init__(self):
        rospy.init_node('ERROR', anonymous=False) #initialize user default node and annoymous help in maintaining uniquness of node
        self.bridge = cv_bridge.CvBridge()
        self.pub = rospy.Publisher('geometry_msgs',Float32, queue_size=10) 
        self.sub = rospy.Subscriber('geometry_msgs',Float32, queue_size=10) 
        rate = rospy.Rate(10)
    def draw_grid(img, grid_shape, color=(0, 255, 0), thickness=1):
        h, w, _ = img.shape
        rows, cols = grid_shape
        dy, dx = h / rows, w / cols

        # draw vertical lines
        for x in np.linspace(start=dx, stop=w-dx, num=cols-1):
            x = int(round(x))
            cv2.line(img, (x, 0), (x, h), color=color, thickness=thickness)

        # draw horizontal lines
        for y in np.linspace(start=dy, stop=h-dy, num=rows-1):
            y = int(round(y))
            cv2.line(img, (0, y), (w, y), color=color, thickness=thickness)

        
    def image_callback(self):
        v = cv2.VideoCapture(0)
        # capture videoz
        while True:
            cx=320
            cy=90
            ret, frame1 = v.read()
            frame=frame1[300:480, ::]
            Line_detection.draw_grid(frame,(2,2))
            # color space change
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv=cv2.medianBlur(hsv,9)
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 30])
            mask = cv2.inRange(hsv,lower_black, upper_black)
            result=cv2.bitwise_and(frame,frame,mask=mask)
            #Contours
            contours, hierarchy = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
            image_copy = mask.copy()
            if len(contours) !=0:
                cv2.drawContours(image=result, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=1,
                                 lineType=cv2.LINE_AA)
                #Centre
                c=max(contours,key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    d=0
                    print(cX,cY)
                    if cX<cx:
                        d = math.sqrt(( cy-cY)**2 + ( cx-cX)**2)
                        print("turn left")
                        print(d)
                    if cX>cx:
                        d = -(math.sqrt(( cy-cY)**2 + ( cx-cX)**2))
                        print("turn right")
                        print(d)
                    if cX==cx:
                        print(d)
                        print("you r on right path")
                    if d==0:
                        print(d)
                        print("you r right")
                else:
                    cX, cY = 0, 0
                
                cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(frame, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            self.pub.publish(d)
            cv2.imshow('Cropped', frame)
            #cv2.imshow('Orignal',frame1)
            cv2.imshow('result', result)
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
        v.release()
        cv2.destroyAllWindows()


if __name__=='__main__':
    linedec=Line_detection()
    linedec.image_callback()