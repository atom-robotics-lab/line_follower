#! /usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

#READ n SHOW VIDEO
cap = cv2.VideoCapture(0)

cap.set(3, 160)
cap.set(4, 120)

bridge = cv_bridge.CvBridge()
rospy.init_node('IMG', anonymous=False) #initialize user default node and annoymous help in maintaining uniquness of node
pub = rospy.Publisher('sensor_msgs',Image, queue_size=10) #rospy.pub. take 3 arg(topic(by user),msg type,rate(genrally use for long task))
rate = rospy.Rate(10) #no. of time loop ittrate in 1 sec

while not rospy.is_shutdown():

    ret, frame = cap.read()
    
    
    #IMG CONVERTER

    
    brg=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    #cv2.threshold(source, thresholdValue(mid_Pt for white n black), maxVal, thresholdingTechnique) 
    ret, thresh = cv2.threshold(brg, 90, 255, cv2.THRESH_BINARY)
    inverted_image = cv2.bitwise_not(thresh)
    

    #PERSPECTIVE

    #cropped = img[start_row:end_row, start_col:end_col]
    cropped_image1 = frame[100::, 0::]
    cropped_image3 = inverted_image[100::, 0::]

        
    #CONTOURS N CENTEROID


    contours, hierarchy = cv2.findContours(cropped_image3 , 1, cv2.CHAIN_APPROX_NONE)
    
    if len(contours) > 0 :
        c = max(contours, key=cv2.contourArea) #find largest contour or object which have largest area
        
        #find centeroid of object
        M = cv2.moments(c)
        
        if M["m00"] !=0 :
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            print("CX : "+str(cx)+"  CY : "+str(cy))

            cv2.circle(cropped_image1, (cx,cy), 5, (0,0,255), -2)
            cv2.circle(cropped_image3, (cx,cy), 5, (255,255,255), -2)
            
    

        cv2.drawContours(cropped_image1, c, -1, (0,255,0), 1)
        
        ros_image = bridge.cv2_to_imgmsg(cropped_image1 , "bgr8")
        pub.publish(ros_image)

        
        cv2.imshow("Mask",cropped_image3)
        cv2.imshow("Frame",cropped_image1)
        
    
    if cv2.waitKey(10) & 0xff == ord('q'):   # 1 is the time in ms
      
        break
cap.release()
cv2.destroyAllWindows()

