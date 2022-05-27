#! /usr/bin/env python3
from geometry_msgs.msg import Twist
import sys
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
import rospy
import math
from robot_controller import bot_control


class Line_Follower :
    def __init__(self):
        #rospy.init_node('ERROR', anonymous=False) #initialize user default node and annoymous help in maintaining uniquness of node
        self.bridge = cv_bridge.CvBridge()
        #self.pub = rospy.Publisher('geometry_msgs',Float32, queue_size=10) 
        self.sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
        
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.velocity_msg = Twist()
        self.velocity_msg.linear.y = 0
        self.velocity_msg.linear.z = 0
        self.velocity_msg.angular.x = 0
        self.velocity_msg.angular.y = 0
        #PID control/using P controller:
        self.P = 0.015 #rospy.get_param("robot_controller/pid/p")

        rate = rospy.Rate(10)
        self.bc=bot_control()
        
    #move function to move robot
    # def move(self,linear,angular):
    #     self.velocity_msg.linear.x = linear
    #     self.velocity_msg.angular.z = angular 
    #     self.pub.publish(self.velocity_msg)
    def process_image(self) :

        self.cX=80
        self.cY=60
        frame = self.cap
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)


        dst = cv2.medianBlur(hsv, 9)



        lower_brg=np.array([0,0,0])
        upper_brg=np.array([180, 255, 65])
        mask = cv2.inRange(dst,lower_brg, upper_brg)
        self.cropped_image1 = frame[100::, 0::]
        #cropped_image3 = inverted_image[100::, 0::]
        self.cropped_image3 = mask[100::, 0::]


        
        
    def callback(self,data):         

    


            
        self.cap = self.bridge.imgmsg_to_cv2(data, "bgr8")

        cv2.imshow("data" , self.cap)
        cv2.waitKey(1) 
        
        self.process_image()

        self.control_loop()
        
    def control_loop(self):
            #Contours
        contours, hierarchy = cv2.findContours(self.cropped_image3 , 1, cv2.CHAIN_APPROX_NONE)
            
        if len(contours) >0:
            
            c=max(contours,key=cv2.contourArea)
            
            
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print("CX : "+str(cx)+"  CY : "+str(cy))
                if cx<self.cX:
                    d = math.sqrt(( cy-self.cY)**2 + ( cx-self.cX)**2)
                    print("turn left")
                    print(d)
                    self.bc.fix_error(0,2)
                if cx>self.cX:
                    d = -(math.sqrt(( cy-self.cY)**2 + ( cx-self.cX)**2))
                    print("turn right")
                    print(d)
                    self.bc.fix_error(0,2)
                if cx==self.cX:
                    print(d)
                    self.bc.fix_error(0,0)
                    print("you r on right path")
                if d==0:
                    print(d)
                    self.bc.fix_error(0,0)
                    print("you r right")
            else:
                cx, cy = 0, 0
            
            
            cv2.circle(self.cropped_image3, (cx, cy), 5, (255, 255, 255), -2)
            cv2.circle(self.cropped_image1, (cx, cy), 5, (255, 255, 255), -2)
            #cv2.circle(cropped_image2, (cX, cY), 5, (255, 255, 255), -2)
            
            cv2.drawContours(self.cropped_image1, c, -1, (0,255,0), 1)
        
        #self.pub.publish(d)
            cv2.imshow("Mask",self.cropped_image3)
        #cv2.imshow("Frame",cropped_image1)
        #cv2.imshow("Gray",cropped_image2)
            cv2.waitKey(1)  
        
def main():
    rospy.init_node("line_follower",anonymous=True)
    a=Line_Follower()

    try :
        rospy.spin()
    except:
        print("error")

    cv2.destroyAllWindows()



main() 
    
