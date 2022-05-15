#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
import numpy as np
from line_follower import Line_Follower
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image

class bot_cotrol:
    def __init__(self):
        self.image_sub = Line_Follower()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.velocity_msg = Twist()
        self.velocity_msg.linear.y = 0
        self.velocity_msg.linear.z = 0
        self.velocity_msg.angular.x = 0
        self.velocity_msg.angular.y = 0
        #PID control/using P controller:
        self.P = rospy.get_param("robot_controller/pid/p")


    #move function to move robot
    def move(self,linear,angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular 
        self.pub.publish(self.velocity_msg)

    #fix error & bot position correction
    def fix_error(self, linear_error, orien_error):
        
        if linear_error != 0:
            
            # moving in straight line
            self.move(self.P*linear_error, 2)
            
        if orien_error != 0:           
            # fixing the yaw     
             self.move(0,self.P*-1*orien_error)


  
if __name__=="__main__":
    robot = bot_cotrol()