#!/usr/bin/env python

## @file camera_processing.py 
# @brief This node accesses to the camera, handles the images, founding the balls and sends all the informations about them to the state machine node. 
#


#ros + python library 
import rospy
import roslib
import time
import math
import random
import sys 

#message ros 
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped 
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import CompressedImage 
from std_msgs.msg import String, Float64, Bool 
from tf import transformations 
from gazebo_msgs.msg import LinkState 
from nav_msgs.msg import Odometry 


#import numpy and scipy 
import numpy as np 
from scipy.ndimage import filters 

import imutils

#Open CV 
import cv2 
from sensor_msgs.msg import CompressedImage 

#msg 
from exp_assignment3.msg import BallInfoMsg


global balls 

## class for imformations about balls 
class Ball: 
    def __init__(self, color, lower, upper, radius, centerx, closeball, firstdetection ): 
        self.color = color
        self.lower = lower 
        self.upper = upper 
        self.detected = False
        self.radius = 0.0 
        self.centerx = 0 
        self.closeball = 0 
        self.firstdetection = 0 

## initialization of balls 
balls = [   Ball('green',(50, 50, 20),(70, 255, 255), 0.0, 0, 0, 0),
            Ball('yellow',(25, 50, 20),(32, 255, 255), 0.0, 0, 0, 0),
            Ball('red',(0, 50, 100),(12, 255, 255), 0.0, 0, 0, 0),
            Ball('blue',(105, 50, 20),(135, 255, 255), 0.0, 0, 0, 0),
            Ball('magenta',(143, 50, 20),(160, 255, 255), 0.0, 0, 0, 0),
            Ball('black',(0, 0, 0),(179, 255, 10), 0.0, 0, 0, 0)]      

## class for detecting and processing images 
class image_feature: 
    ## initialization 
    def __init__(self): 
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", 
                                            CompressedImage, queue_size=1) 

        self.infosBall_pub = rospy.Publisher("/InfoBalls", BallInfoMsg, queue_size=1) 

        #Subscribed Topic
        self.subscriber = rospy.Subscriber(
            "/camera1/image_raw/compressed", CompressedImage, self.Det_Ball_clbk,  queue_size=1)



    ## Callback 
    def Det_Ball_clbk(self, ros_data): 

        ## direct conversion to CV2 
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        ## Compute the center and the radius if the ball is visible 

        ## for each ball 
        for ball in balls: 
            ## compute a mask 
            mask = cv2.inRange(hsv, ball.lower, ball.upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            ## only proceed if at least one contour was found
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                ## if the radius meets a minimum size the ball is not close to the robot
                if radius > 20 and radius < 50: 
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                                    (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    
                    ## if the ball is known already 
                    if ball.detected == True: 
                        ball.firstdetection = 0
                        ball.radius = radius 
                        ball.centerx = center[0]
                        ball.closeball = -1 
                    ## if the ball is not known 
                    else: 
                        ball.detected = True
                        ball.radius = radius 
                        ball.centerx = center[0] 
                        ball.closeball = -1 
                        ball.firstdetection = 1

                ## if the ball is close to the robot 
                elif radius >= 50: 
                    ## if the ball is known already 
                    if ball.detected == True: 
                        ball.firstdetection = 0
                        ball.radius = radius 
                        ball.centerx = center[0]
                        ball.closeball = 1
                    ## if the ball is not known 
                    else: 
                        ball.detected = True
                        ball.radius = radius
                        ball.closeball = 1 
                        ball.centerx = center[0] 
                        ball.firstdetection = 1
                ## if the ball is not detected 
                else: 
                    ball.detected = False 
                    ball.radius = 0 
                    ball.centerx = 0 
                    ball.closeball = 0 
                    ball.firstdetection = 0


                ## create a message 
                message = BallInfoMsg() 
                message.detected = Bool(ball.detected)
                message.color = String(ball.color) 
                message.radius = Float64(ball.radius)
                message.centerx = Float64(ball.centerx)
                message.closeball = Float64(ball.closeball)
                message.firstdetection = Float64(ball.firstdetection)
                ## publish a message 
                self.infosBall_pub.publish(message)

            ## show the image 
            cv2.imshow('window', image_np) 
            cv2.waitKey(2) 
            
## main 
def main(): 
    rospy.init_node('camera_processing', anonymous = True) 
    image_feature() 
    rospy.spin() 
    pass

if __name__ == '__main__': 
    main() 
