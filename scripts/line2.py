#! /usr/bin/env python3
import sys
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
import rospy
import math


class Line_Follower():
    def __init__(self):
        rospy.init_node('ERROR', anonymous=False) #initialize user default node and annoymous help in maintaining uniquness of node
        self.bridge = cv_bridge.CvBridge()
        
        self.pub = rospy.Publisher('geometry_msgs',Float32, queue_size=10) 
        self.sub = rospy.Subscriber('geometry_msgs',Float32, queue_size=10) 

        rate = rospy.Rate(10)
        
        
    def img_detection(self):
        cap = cv2.VideoCapture(0)
        cap.set(3, 160)
        cap.set(4, 120)
        # capture videoz
        while True:
            cX=80
            cY=60
            ret, frame = cap.read()
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            
            
            dst = cv2.medianBlur(hsv, 9)
            

 
            lower_brg=np.array([0,0,0])
            upper_brg=np.array([180, 255, 65])
            mask = cv2.inRange(dst,lower_brg, upper_brg)
            cropped_image1 = frame[100::, 0::]
            #cropped_image3 = inverted_image[100::, 0::]
            cropped_image3 = mask[100::, 0::]
            '''
            N = cv2.moments(brg)
            if N["m00"] != 0:
                cX = int(N["m10"] / N["m00"])
                cY = int(N["m01"] / N["m00"])
                print("cX : "+str(cX)+"  cY : "+str(cY))
                '''
            #Contours
            contours, hierarchy = cv2.findContours(cropped_image3 , 1, cv2.CHAIN_APPROX_NONE)
            if len(contours) >0:
                
                c=max(contours,key=cv2.contourArea)
                
                
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    print("CX : "+str(cx)+"  CY : "+str(cy))
                    if cx<cX:
                        d = math.sqrt(( cy-cY)**2 + ( cx-cX)**2)
                        print("turn left")
                        print(d)
                    if cx>cX:
                        d = -(math.sqrt(( cy-cY)**2 + ( cx-cX)**2))
                        print("turn right")
                        print(d)
                    if cx==cX:
                        print(d)
                        print("you r on right path")
                    if d==0:
                        print(d)
                        print("you r right")


                else:
                    cx, cy = 0, 0
                
                
                cv2.circle(cropped_image3, (cx, cy), 5, (255, 255, 255), -2)
                cv2.circle(cropped_image1, (cx, cy), 5, (255, 255, 255), -2)
                #cv2.circle(cropped_image2, (cX, cY), 5, (255, 255, 255), -2)
                
            cv2.drawContours(cropped_image1, c, -1, (0,255,0), 1)
            
            self.pub.publish(d)
            cv2.imshow("Mask",cropped_image3)
            cv2.imshow("Frame",cropped_image1)
            #cv2.imshow("Gray",cropped_image2)   
            

        
            if cv2.waitKey(10) & 0xff == ord('q'):
                break
        cap.release()

        cv2.destroyAllWindows()



if __name__=='__main__':
    a=Line_Follower()
    a.img_detection()
    