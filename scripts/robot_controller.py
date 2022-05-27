#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
import numpy as np
from sensor_msgs.msg import Image

class bot_control:
    def __init__(self) :
        self.P = 0.015
        self.velocity_msg = Twist()
        self.pub = rospy.Publisher('/cmd_vel' , Twist , queue_size = 10)
        self.P = 0.0001 #rospy.get_param("line_follower_controller/pid/p")
    
      #self.image_sub = Line_Follower()

    #move function to move robot
    def move(self,linear,angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular 
        self.pub.publish(self.velocity_msg)
        

        
    #fix error & bot position correction
    def fix_error(self, linear_error, orien_error):
        
        if orien_error != 0:           

            # fixing the yaw     
             self.move(0.5,self.P*orien_error)
                
        else:
            
            # moving in straight line
            self.move(1.0, 0)
            


  
if __name__=="__main__":
    robot = bot_control()
