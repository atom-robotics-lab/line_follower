#! /usr/bin/env python3
from re import L
import sys
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
import rospy


class Line_detection():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('LineFollower/Video',Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
        self.twist = Twist()
        
    def image_callback(self):
        v = cv2.VideoCapture(0)
        # capture videoz
        while True:
            ret, frame1 = v.read()
            frame=frame1[180:400, 120:660]
            # color space change
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
                else:
                    cX, cY = 0, 0
                cv2.circle(result, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(result, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.imshow('Cropped', frame)
            cv2.imshow('Orignal',frame1)
            cv2.imshow('result', result)
            if cv2.waitKey(1) and 0xFF == ord('Q'):
                break
            bridge = cv_bridge.CvBridge()
            image_message = bridge.cv2_to_imgmsg(result, encoding="passthrough")
        v.release()
        cv2.destroyAllWindows()


if __name__=='__main__':
    linedec=Line_detection()
    linedec.image_callback()
