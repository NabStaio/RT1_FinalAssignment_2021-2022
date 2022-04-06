#! /usr/bin/env python

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
import math
import rospy

curr_modality = rospy.get_param('active')
desired_x = rospy.get_param("des_pos_x")
desired_y = rospy.get_param("des_pos_y")

flag_goal = 0

def update_variables(): 

	global desired_x, desired_y, curr_modality
	curr_modality = rospy.get_param('active')
	desired_x = rospy.get_param('des_pos_x')
	desired_y = rospy.get_param('des_pos_y')


def clbk_odom(msg): 

	global config
	config = msg.pose.pose.position
	

def done(status,result):
	global flag_goal
	
	if status==3:
		print("YOU'RE REACHED THE GOAL \n")
		flag_goal = 1
	

def act_client_set_goal():


	goal.target_pose.pose.position.x = desired_x
	goal.target_pose.pose.position.y = desired_y
	print("START AUTONOMOUS DRIVE \n")
	client.send_goal(goal,done)


def act_client_init():

	global client 
	global goal 
	
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.orientation.w = 1.0
	
def timeout(event):
	

	if curr_modality==1:
		print ("GOAL TIME EXPIRED :" + str(event.current_real))
		print("ROBOT DIDN'T REACH THE TARGET POSITION WITHIN A 1min\n")
		rospy.set_param('active', 0)
		

def main():

	
	global flag_goal
	rospy.init_node('auto')
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	rate = rospy.Rate(10)
	flag = 0
	flag_ = 0
	
	action_client_init()
	i = 0
	while(1):
	
		update_variables()	
		
		if curr_modality==1:
			
			if flag == 1:
				act_client_set_goal()
				rospy.Timer(rospy.Duration(60),timeout)
				
				flag = 0
				flag_ = 1
		
		else:
			 
			if flag == 0 and flag_==0:
				
				print("STOP MODALITY 1 \n")
				flag = 1
			
			
			if flag == 0 and flag_==1:
				
				
				if flag_goal==1:
					# If the goal is reached I will not cancel the goal because. 
					print("STOP MODALITY 1")
					flag = 1
					flag_ = 0
					flag_goal = 0
			
				else:
					print("GOAL DELETED, STOP MODALITY 1 ")
					client.cancel_goal()
					flag = 1
					flag_2 = 0
				
		
		# Print of the current position		
		if(i%10==0):
		
			print("CURR_POSITION:  X:"+ str(position_.x)+"Y:" + str(position_.y), end = '\r')
		i=i+1
	    		
	rate.sleep()
     

if __name__ == '__main__':
	main()



