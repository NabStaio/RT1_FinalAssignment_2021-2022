#!/usr/bin/env python3

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
import math
import rospy

# Getting the parameters for goal's position and the modality
curr_modality = rospy.get_param('active')
desired_x = rospy.get_param("des_pos_x")
desired_y = rospy.get_param("des_pos_y")

flag_goal = 0 # flag to state if we achieve or not the goal


#######################################################
# UPDATE_PARAMETERS():                                #
# This function retrieves the parameters since the    #
# simulation is continously running and then it is    #
# needed an update of the variables                   # 
#######################################################

def update_parameters(): 

	global desired_x, desired_y, curr_modality
	curr_modality = rospy.get_param('active')
	desired_x = rospy.get_param('des_pos_x')
	desired_y = rospy.get_param('des_pos_y')


##################################################
# CLBK_ODOM():                                   #
# Arguments--> msg                               #
# This function is a callback that retrieves     #
# the robot's odometry to keep track of the      #
# position (config variable)                     #
##################################################

def clbk_odom(msg): 

	global config
	config = msg.pose.pose.position
	
####################################################
# DONE():                                          #
# Arguments--> status                              #
#              result                              #
# This function is a callback to get info about    #
# the status of the robot once the goal is reached #
# the change of the flag will set the robot in     #
# WAITING MODE                                     #
####################################################

def done(status,result):
	global flag_goal
	
	if status==3:
		print("YOU'RE REACHED THE GOAL \n")
		flag_goal = 1
	
#######################################################
# ACT_CLIENT_SET_GOAL():                              #
# The function sets new goal through an action        #
# with the use of the callback done(status,result)    #
#######################################################

def act_client_set_goal():


	goal.target_pose.pose.position.x = desired_x
	goal.target_pose.pose.position.y = desired_y
	print("START AUTONOMOUS DRIVE \n")
	client.send_goal(goal,done)

#############################################################################
# ACT_CLIENT_INIT():                                                        #
# This function needs to initialize the action client and the               #
# message to send                                                           #
#############################################################################

def act_client_init():

	global client 
	global goal 
	
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.orientation.w = 1.0
	
####################################################################################
# TIMEOUT():                                                                       #
# Arguments--> event                                                               #
# This function simply check if the robot will reach the target in a               #
# certain amount of time. If the time expire an event is triggered and the robot   #
# will go in WAITING MODE                                                          #
####################################################################################

def timeout(event):
	

	if curr_modality==1:
		print ("GOAL TIME EXPIRED :" + str(event.current_real))
		print("ROBOT DIDN'T REACH THE TARGET POSITION WITHIN A 3min\n")
		rospy.set_param('active', 0)

#####################################################################################
# MAIN():                                                                           #
# The main function will start the node and drives the robot in an autonomous way,  #
# the behaviour is the one of the "go_to_point_service" of the class' exercise.     #
# flags are used to understand in which modality the robot is and if it reaches     #
# the goal.                                                                         #
#####################################################################################	

def main():

	
	global flag_goal #flag to see if the goal is reached
	rospy.init_node('auto') #initialization of the node
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom) #subscribe to odom for knowing position
	rate = rospy.Rate(10)
	#flags for tracing the current modality
	flag = 0 
	flag_ = 0 
	
	act_client_init() #initialize the action through the function
	i = 0
	while(1):
	
		update_parameters()	
		#if i'm in autonomous mode
		if curr_modality==1:
			
			if flag == 1:
				act_client_set_goal() 
				rospy.Timer(rospy.Duration(180),timeout) #timer for the timeout event
				
				flag = 0
				flag_ = 1
		
		else:
			#Initial WAITING MODE 
			if flag == 0 and flag_==0:
				
			#	print("STOP MODALITY 1 \n")
				flag = 1
			
			#WAITING MODE once the user stop the robot
			if flag == 0 and flag_==1:
				
				
				if flag_goal==1:
					# If the goal is reached I will not cancel the goal. 
					print("STOP MODALITY 1")
					flag = 1
					flag_ = 0
					flag_goal = 0
			
				else:
					print("GOAL DELETED, STOP MODALITY 1 ")
					client.cancel_goal()
					flag = 1
					flag_2 = 0
				
		
		# Print of the current position needed for debugging		
		#if(i%10==0):
		
		#	print("CURR_POSITION:  X:"+ str(config.x)+"Y:" + str(config.y), end = '\r')
		#i=i+1
	    		
	rate.sleep()
     

if __name__ == '__main__':
	main()



