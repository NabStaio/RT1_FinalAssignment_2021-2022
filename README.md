Research Track 1 Final Assignment 
================================
## Project Requirements

It is required to develop a software architecture for the control of the robot in the environment given by the professor.
The software will rely on the *move_base* and *gmapping* packages for localizing the robot and plan the motion.
The architecture should be able to get the user request, and let the robot execute one of the following behaviors
(depending on the user's input):
1. autonomously reach a x,y coordinate inserted by the user
2. let the user drive the robot with the keyboard
3. let the user drive the robot assisting them to avoid collisions

Installing and Running
----------------------
To install and run the project you have to create your own ROS workspace and in the src folder you have to:
* Download the *final_assignment* package from professor Recchiuto [repository](https://github.com/CarmineD8/final_assignment) and put it in _src_, then go in the *final_assignment* directory and download the branch that matches your ROS distribution (in the terminal do ```$git checkout -your_ros_distro- ```).
* Download the *slam_gmapping* package from the professor Recchiuto [repository](https://github.com/CarmineD8/slam_gmapping) and follow the same step done for the *final_assignment* package.
* Do ```$catkin_make``` in the root folder of your workspace.

Then open two terminals because the user interface is used separately from the other nodes.

In the first terminal run the User Interface:
```bash
$run final_assignment UI.py
```

in the second terminal launch the simulation and all the nodes:
```bash
$run final_assignment drive_simulator.launch 2>/dev/null
```

2>/dev/null is used to not show the warnings

The *drive_simulator.launch* file simply launch three others launch files in which there is the *nodes.launch*: a file created to run the three nodes corresponding to the three different required modalities; hereafter the syntax:

**drive_simulator.launch**:
```xml
<launch>
	<include file="$(find final_assignment)/launch/simulation_gmapping.launch"/>
	<include file="$(find final_assignment)/launch/nodes.launch"/>
	<include file="$(find final_assignment)/launch/move_base.launch"/>
</launch>
```

**nodes.launch**:
```xml
<launch>

<param name="active" type="int" value="0" />

<param name="des_pos_x" type="double" value="1" />
<param name="des_pos_y" type="double" value="1" />


<node pkg="final_assignment" type="auto.py" name="auto" required="true" output="screen" />
<node pkg="final_assignment" type="teleop.py" name="teleop" required="true" output="screen" /> 
<node pkg="final_assignment" type="assistance.py" name="assistance" required="true" output="screen"/> 

</launch>
```

Simulation Environment
------------
The robot will move in two simulations: a *Gazebo simulation* (first image) to simulate a real environment and a *Rviz one* (second image) to analyze what the robot see of the map:

<p align="center">
<img src="https://github.com/NabStaio/RT1_FinalAssignment_2021-2022/blob/main/images/gazebo.PNG" width="500" height="500">
</p>

<p align="center">
<img src="https://github.com/NabStaio/RT1_FinalAssignment_2021-2022/blob/main/images/rviz.PNG" width="500" height="500">
</p>

The robot is equipped with a laser scan and it doesn't know the all map a priori, it will complete the mapping by moving around, adopting then a SLAM approach. 
  

Nodes
-----
### UI node 
The UI node will provide a user interface that interacts with the user. It pops up a window (image below) which provides instructions about how to use the simulation. The module retrieves user's inputs corresponding to different modalities:
* **0 WAITING MODE**: The robot waits for what kind of modality it has to hire, indeed this mode is used to switch between behaviours
* **1 AUTONOMOUS DRIVE**: The robot drives autonomously to a position given by the user, if the robot is not able to reach the target in a certain amount of time 
it will stop and the goal is deleted, the goal can also be deleted by the user by typing 0 returning in a waiting mode.
* **2 KEYBOARD DRIVE**: The robot is guided by the user with keyboard
* **3 ASSISTANCE DRIVE**: The robot is again guided by the user with keyboard, but when an obstacle is in its sight it will avoid it autonomously.

Instructions' window:
<p align="center">
<img src="https://github.com/NabStaio/RT1_FinalAssignment_2021-2022/blob/main/images/UI.PNG" width="500" height="400">
</p>

The node retrieves input in a `while loop` and `if statements` determines the setting of the parameters corresponding to the correct modality:

```python

while not rospy.is_shutdown():
	 
		command = int(input('CHOOSE MODALITY: \n')) #Take the input from keyboard
		
		# WAITING MODALITY
		if command == 0:
			
			...
				
			rospy.set_param('active', 0)# Setting the parameter to 0.
			...
				
		# AUTONOMOUS DRIVE
		elif command == 1:
		
			...
			
			des_x_input = float(input("INSERT X COORDINATE: "))
			des_y_input = float(input("INSERT Y COORDINATE: "))
			rospy.set_param('des_pos_x', des_x_input)
			rospy.set_param('des_pos_y', des_y_input)
			rospy.set_param('active', 1)
			
            ...
			
		# KEYBOARD DRIVE
		elif command == 2:
		
			...
						
			rospy.set_param('active', 2)
			...
				
		# ASSISTANCE DRIVE
		elif command == 3:
		
			...

			rospy.set_param('active', 3)
			...
					
		else:
			print("WRONG KEY, TRY AGAIN.")



```

Here the flowchart:

<p align="center">
<img src="https://github.com/NabStaio/RT1_FinalAssignment_2021-2022/blob/main/images/UI_flowchart.PNG" width="700" height="500">
</p>


### Auto node
This node implements the _Autonomous Drive_ modality. The script makes use of an _action client_ istance to establish a direct communication with the robot and set and cancel location goals.
The Action Client-Service communicate via a "ROS Action Protocol", which is built on top of ROS messages. The client and server then provide a simple API for users to request goals (on the client side) or to execute goals (on the server side) via function calls and callbacks. In this code only the _Actionclient_ side is implemented using the already existing server of the following action messages:
- ```MoveBaseAction```
- ```MoveBaseGoal```

#### Functions
This paragraph shows the functions used to make the script more flexible.
For handling the _Actionclient_ the ```act_client_init()``` initializes the goal message and when the node is active ```act_client_set_goal() ``` retrieves the new parameters and approprietly fill its fields.

```python
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
```

The following image shows the Rviz graphical interface once the goal is set (-0.05, -0.05):

<p align="center">
<img src="https://github.com/NabStaio/RT1_FinalAssignment_2021-2022/blob/main/images/Set_goal.PNG" width="500" height="500">
</p>

The argument ```done``` of the ```send_goal``` function is a call-back needed for retrieving info on the goal reaching status. This function gets info from the server side. There are many different values associated to the status parameter during execution ending. The only one used in the code is the status 3 related to the goal achievement:

```python
def done(status,result):
	global flag_goal
	
	if status==3:
		print("YOU'RE REACHED THE GOAL \n")
		flag_goal = 1
```

In this case i decide to not use the timeout of the server, but a personal timeout event for deleting the goal if the robot doesn't reach it in time:

```python
def timeout(event):
	if curr_modality==1:
		print ("GOAL TIME EXPIRED :" + str(event.current_real))
		print("ROBOT DIDN'T REACH THE TARGET POSITION WITHIN A 3min\n")
		rospy.set_param('active', 0)
```

The ```cancel_goal``` part is activated if the robot goes in its _Waiting mode_ when the user cancel the goal by typing 0. This process is managed by if statements and flags for tracking the current modality:

```python
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
```

### Teleop node



### assistance node


Nodes' Connection
---------

This is an image that show how nodes are connected to each other (thanks to the command rosrun rqt_graph rqt_graph):

<p align="center">
<img src="https://github.com/NabStaio/RT1_FinalAssignment_2021-2022/blob/main/images/rqt_graph.PNG" width="900" height="500">
</p>


Conclusions
-----------

The robot is able to do a complete lap of the circuit without crushing on walls. Nevertheless, it has problems when its speed is drastically increased or when a continuous race with more than 2 laps is done. Therefore, the code could be improved by having an infinity race withouth stopping, also the UI could be more entertaining by adding more visual features. Hovewer this work in my opinion is sufficient to fulfill the requirements given by the professor. 
