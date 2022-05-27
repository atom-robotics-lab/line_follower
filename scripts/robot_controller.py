#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
import numpy as np
from line_follower import Line_Follower
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image

class bot_control:
    
        #self.image_sub = Line_Follower()
        
    #fix error & bot position correction
    def fix_error(self, linear_error, orien_error):
        
        if linear_error != 0:
            
            # moving in straight line
            self.move(self.P*linear_error, 0)
            
        if orien_error != 0:           
            # fixing the yaw     
             self.move(2,self.P*-1*orien_error)


  
if __name__=="__main__":
    robot = bot_control()
