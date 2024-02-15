#!/usr/bin/env python

## @file state_machine.py 
# @brief This node implements a state machine which permits to move around, detect the ball (if visible), go to sleep, play with the human going to a target defined by a colored ball if he knows the room location. If not, he can find it exploring the environment. 
# 


#ros + python library 
import rospy
import roslib
import time
import math
import random
import sys 

#smach library 
import smach 
import smach_ros

#message ros 
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped 
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import CompressedImage 
from std_msgs.msg import String, Float64 
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

#actionlib
import actionlib 
import actionlib.msg 

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# msg used for ball infos 
from exp_assignment3.msg import BallInfoMsg

## library for running explore package 
import matplotlib.pyplot as plt
import signal 
import subprocess 
import roslaunch


## Global variables 
global rooms
global play 
global sleep
global Finding
global Target 
global counter 
global StopAction


## Initialization and Callback for InfoBall 
InfoBall = BallInfoMsg() 
InfoBall.detected = False 
InfoBall.radius = 0 
InfoBall.centerx = 0 
InfoBall.closeball = 0
InfoBall.firstdetection = 0 

def clbk_ball(info): 
    global InfoBall
    InfoBall.detected = info.detected.data
    InfoBall.color = info.color.data
    InfoBall.radius = info.radius.data 
    InfoBall.centerx = info.centerx.data
    InfoBall.closeball = info.closeball.data
    InfoBall.firstdetection = info.firstdetection.data


## Callback for the command
usercommand = String() 
def clbk_command(data): 
    usercommand = data.data


## Callback for the odometry 
position = Point() 
yaw = 0 

def clbk_odometry(msg): 
    global position 
    position = msg.pose.pose.position
    ## yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


## client of actionlib server for moving the dog robot using MOVEBASE package
def move_dog(target): 

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]

   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)

    ## in loop 
    while True: 
        ## if the goal is reached, then cancel the goal 
        if math.fabs(position.x - target[0]) < 0.3 or math.fabs(position.y - target[1]) < 0.3:
            time.sleep(3) 
            rospy.loginfo('Goal reached!!')
            return client.cancel_all_goals() 
             
        ## if a new ball is detected, then cancel the goal 
        if (InfoBall.firstdetection == 1) and (rospy.get_param('StopAction') == 1): 
            return client.cancel_all_goals() 

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()  



## class for storing informations about rooms 
class Room: 
    def __init__(self, name, color, x, y): 
        self.name = name #name of the room
        self.color = color #color of the ball in the room  
        self.x = x #x-position of the room : Default is 0 
        self.y = y #y-position of the room : Default is 0


## initialization for rooms 
rooms = [   Room('LivingRoom', 'green', 0, 0),
            Room('Kitchen', 'yellow', 0, 0),
            Room('Closet', 'red',0, 0),
            Room('Entrance', 'blue',0, 0),
            Room('Bathroom','magenta',0, 0),
            Room('Bedroom', 'black', 0, 0)]


## User Action function 
def user_action(): 
    ## random choice between search for the ball or go to sleep 
    return random.choice(['play', 'sleep'])



## Normal State
class Normal(smach.State): 
    
    ## inizialization
    def __init__(self):
        ## 3 outcomes defined 
        smach.State.__init__(self, outcomes=['go_to_sleep','go_to_play','track_ball'])
        self.userAction = String() 

    ## execution 
    def execute(self, userdata):
        ## in loop 
        while True: 
            ## couter 
            rospy.set_param('StopAction',1)
            counter = rospy.get_param('counter')
            counter = counter + 1 
            rospy.set_param('counter', counter)
            rospy.loginfo('%s', rospy.get_param('counter'))
            time.sleep(4) 
            ## Random motion implementing a move_base action with a random goal   
            Normal = move_dog([random.randrange(-5,6), random.randrange(-5,3)])

            ## if camera detects a new ball which has not been detected before 
            if InfoBall.firstdetection == 1: 
                rospy.set_param('counter', 1)
                rospy.loginfo('the %s ball has not been detected before!', InfoBall.color)
                return 'track_ball'

                
            ## after 3 random goal achieved it can switch in sleep or play behavior (randomly choosen)
            if rospy.get_param('counter') == 2:
                rospy.set_param('counter', 0)
                ## call the function to randomly choose the next behavior
                self.userAction = user_action() 
                ## if play behavior is chosen 
                if self.userAction == "play": 
                    command = rospy.wait_for_message("UserCommand", String) 
                    rospy.loginfo('it is Time to play: %s', command.data)
                    ## look at the target 
                    res = command.data.split() 
                    rospy.set_param('Target', res[1])
                    rospy.loginfo('%s', res[1]) 
                    return 'go_to_play' 
                ## if the sleep behavoir is chosen 
                if self.userAction == "sleep": 
                    rospy.loginfo('it is time to sleep')
                    return 'go_to_sleep'


## call publisher velocity
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
vel = Twist()
vel.linear.x = 0 
vel.linear.y = 0 
vel.linear.z = 0 
vel.angular.x = 0 
vel.angular.y = 0 
vel.angular.z = 0 

## Normal_Track state
class Normal_Track(smach.State): 

    ## inizialization
    def __init__(self):
        ## one outcome defined 
        smach.State.__init__(self, outcomes=['done']) 


    ## execution 
    def execute(self, userdata):
        rospy.loginfo('follow the ball')
        time.sleep(5) 
        ## if the ball is far away, set the velocity
        while InfoBall.closeball == -1: 
            vel.angular.z = -0.002 * (InfoBall.centerx - 400) 
            vel.linear.x = -0.01 * (InfoBall.radius - 100) 
            vel_pub.publish(vel) 

        vel.linear.x = 0 
        vel.angular.z = 0 
        vel_pub.publish(vel)

        ## check the index of our room list according to the color of the detected ball to fill the position as well
        for index, room in enumerate(rooms): 
            if room.color == InfoBall.color: 
                rospy.loginfo('%x', index)
                break
        ## store the position 
        rospy.loginfo('Ball reached. Lets store the position')
        rooms[index].x = position.x 
        rooms[index].y = position.y 
        rospy.loginfo('I am in x = %x, y = %s , in the %s', rooms[index].x, rooms[index].y, rooms[index].name)

        time.sleep(5) 
        return 'done'



## Sleep State
class Sleep(smach.State): 

    ## inizialization
    def __init__(self):
        #outcome to be back in normal state
        smach.State.__init__(self, outcomes=['sleep_to_normal'])

    ##execution 
    def execute(self, userdata): 
        rospy.loginfo('Go to sleep')
        ## call move_base to move to the home position 
        rospy.set_param('StopAction', 0)
        Sleep = move_dog([rospy.get_param("/home_x"), rospy.get_param("/home_y")])
        ## sleep for a while
        time.sleep(rospy.get_param("/timesleeping"))
        rospy.loginfo('Good Morning!')
        return 'sleep_to_normal'


## Play State 
class Play(smach.State): 

    ## inizialization
    def __init__(self):
        ## two otcomes defined 
        smach.State.__init__(self, outcomes=['play_to_normal', 'play_to_find'])


    ##execution 
    def execute(self, userdata): 
        rospy.loginfo('my target is %s', rospy.get_param('Target'))
        rospy.set_param('StopAction', 0)
        ## check the index of the list of the room which corresponds to our target
        for index_, room in enumerate(rooms):     
            if room.name == rospy.get_param('Target'): 
                break

        ## check if the target room has been visited already: if not the default position is true->then switch to find
        if (rooms[index_].x == 0) & (rooms[index_].y == 0): 
            rospy.loginfo('The room %s has not been discovered yet, lets find it', rooms[index_].name)  
            ## set param 'Finding' to 1 to start the search of the ball  
            rospy.set_param('Finding',1) 
            return 'play_to_find'  
        ## otherwise, reach the target 
        else: 
            rospy.loginfo('I know where is %s, lets go there!', rooms[index_].name)
            ## call move_base to reach the target 
            Play = move_dog([rooms[index_].x, rooms[index_].y])
            ## call move_base to reach the human position   
            Gohome = move_dog([rospy.get_param("/human_x"), rospy.get_param("/human_y")]) 
            rospy.loginfo('Hello Friend! How are you?')
            return 'play_to_normal'

## Find State
class Find(smach.State): 

    ## inizialization
    def __init__(self):
        ## two otcomes defined
        smach.State.__init__(self, outcomes=['go_to_play', 'track_place'])


    ##execution 
    def execute(self, userdata): 
        ## if 'finding' param is set to 1 -> start the exploration 
        if rospy.get_param('Finding') == 1: 

            ## launch explore package 
            p = subprocess.Popen(["roslaunch","exp_assignment3","explore.launch"])
            ## in loop 
            while True: 
                ## if a ball has been detected
                if InfoBall.firstdetection == 1: 
                    ## kill the explore process 
                    p.send_signal(signal.SIGINT) 
                    for i in range(0,7): 
                        vel.linear.x = 0 
                        vel_pub.publish(vel)
                    #time.sleep(7)
                    return 'track_place'
        ## if the exploration is done, return in play state 
        if rospy.get_param('Finding') == 0: 
            return 'go_to_play'
        



## Find Track state
class Find_Track(smach.State): 
    ## inizialization
    def __init__(self):
        smach.State.__init__(self, outcomes=['found'])

    ##execution 
    def execute(self, userdata): 
        ## the ball has been found, then set 'finding' = 0 
        rospy.set_param('Finding',0)
        rospy.loginfo('The %s ball has been detected!', InfoBall.color)
        ## follow the ball in case it is faar away 
        while InfoBall.closeball == -1: 
            #rospy.loginfo('ball is far')
            vel.angular.z = -0.002 * (InfoBall.centerx - 400) 
            vel.linear.x = -0.01 * (InfoBall.radius - 100) 
            vel_pub.publish(vel) 

        vel.linear.x = 0 
        vel.angular.z = 0 
        vel_pub.publish(vel)

        ## check the index of our room list according to the color of the detected ball to fill the position as well
        for index, room in enumerate(rooms): 
            if room.color == InfoBall.color: 
                rospy.loginfo('%x', index)
                break
        ## store position 
        rospy.loginfo('Ball reached. Lets store the position')
        rooms[index].x = position.x 
        rooms[index].y = position.y 
        rospy.loginfo('I am in x = %x, y = %s , in the %s', rooms[index].x, rooms[index].y, rooms[index].name)
        time.sleep(4) 

        return 'found'


## main function 
def main(): 
    rospy.init_node('state_machine')
    rospy.Subscriber("/InfoBalls", BallInfoMsg, clbk_ball)
    rospy.Subscriber("/odom", Odometry, clbk_odometry)
    rospy.Subscriber("/UserCommand", String, clbk_command)
    rospy.set_param('Finding',0) 
    rospy.set_param('counter', 0) 
    rospy.set_param('StopAction', 0) 

    

    ## Create a main SMACH state machine
    sm_main = smach.StateMachine(outcomes=[])

    ## main container
    with sm_main:
        ## Add states to the container

        ## Create NORMAL sub-state machine 
        sm_normal = smach.StateMachine(outcomes=['normal_to_play', 'normal_to_sleep'])

        ## Normal container
        with sm_normal: 

            ## Add states to the NORMAL sub container 
            smach.StateMachine.add('NORMAL', Normal(),
                                transitions={'go_to_sleep':'normal_to_sleep',
                                              'go_to_play':'normal_to_play',
                                              'track_ball':'NORMAL_TRACK'})

            smach.StateMachine.add('NORMAL_TRACK', Normal_Track(), 
                                    transitions={'done':'NORMAL'}) 

        smach.StateMachine.add('NORMAL_CONTAINER', sm_normal, 
                                transitions={'normal_to_play':'PLAY',
                                             'normal_to_sleep':'SLEEP'})
        
        smach.StateMachine.add('SLEEP', Sleep(), 
                                transitions={'sleep_to_normal':'NORMAL_CONTAINER'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'play_to_normal':'NORMAL_CONTAINER', 
                                            'play_to_find':'FIND_CONTAINER'})

        ## Create FIND sub-state machine
        sm_find = smach.StateMachine(outcomes=['find_to_play'])
        ## Find container 
        with sm_find: 

            ## Add states to the FIND sub container
            smach.StateMachine.add('FIND', Find(),
                                    transitions={'go_to_play':'find_to_play', 
                                                 'track_place':'FIND_TRACK'})
            smach.StateMachine.add('FIND_TRACK', Find_Track(), 
                                    transitions={'found':'FIND'})

        smach.StateMachine.add('FIND_CONTAINER', sm_find, 
                                transitions={'find_to_play':'PLAY'})

        

    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm_main, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
    TimeToGoToSleep() 


