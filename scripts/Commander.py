#!/usr/bin/env python

## @file Commander.py 
# @brief This node chooses randomly the target and sends the command "GoTo + target" to the state machine. 
# 
# Details: The target is randomly choosen among the rooms. 
#

import rospy
import time
import math
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point

## User action function 
def UserAction(): 
    ## in loop 
    while True:      
        ## choice of the room
        target = String() 
        target = random.choice(['Entrance','LivingRoom','Bedroom', 'Bathroom', 'Closet','Kitchen'])
        ## command 
        command = String() 
        command = "GoTo " + target 
        ## publishes 
        user = rospy.Publisher("UserCommand", String, queue_size=10) 
        user.publish(command) 
        ## sleep for 20 secs 
        time.sleep(20) 

if __name__ == '__main__': 
    rospy.init_node('Commander') 
    UserAction() 


        




    


