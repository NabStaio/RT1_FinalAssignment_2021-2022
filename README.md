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
* DOwnload the *slam_gmapping* package from the professor Recchiuto [repository](https://github.com/CarmineD8/slam_gmapping) and follow the same step done for the *final_assignment* package.
* Do ```$catkin_make``` in the root folder of your workspace.

Then open two terminals because the user interface is used separately from the other nodes.

In the first terminal run the User Interface:
```bash
$run final_assignment UI.py
```

in the second terminal launch the simulation and all the nodes:
```bash
$run final_assignment drive_simulator.launch
```

The *drive_simulator.launch* file simply launch three others launch files in which there is the *nodes.launch*: a file created to run the three nodes corresponding to the three different required modalities; hereafter the syntax:

**drive_simulator.launch**:
```xml
<launch>
	<include file="$(find final_assignment)/launch/simulation_gmapping.launch"/>
	<include file="$(find final_assignment)/launch/nodes.launch"/>
	<include file="$(find final_assignment)/launch/move_base.launch"/>
</launch>
```

**nodes.launch**
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

Introduction
------------
The robot will move in a Rviz and in a Gazebo simulation :

<p align="center">
<img src="https://github.com/NabStaio/RT1_FinalAssignment_2021-2022/blob/main/images/gazebo.PNG" width="500" height="500">
</p>

<p align="center">
<img src="https://github.com/NabStaio/RT1_FinalAssignment_2021-2022/blob/main/images/rviz.PNG" width="500" height="500">
</p>

  

Nodes
-----
### Stage_ros node 

The stage_ros node subscribes to the /cmd_vel topic from the package `geometry_msgs` which provides a `Twist` type message to express the velocity of the robot in free space, splitted into its linear and angular parts (x,y,z).
The stage_ros node also publishes on the `base_scan` topic, from the package called `sensor_msgs` that provides a `LaserScan`, a laser range-finder.
We had also to call the service `reset_positions` from the `std_srvs` package in order to reset the robot position. In `std_srvs` it is contained the `Empty` type service.

### Robot_controller node

In the robot_controller node there is the main code of the project. This node handles multiple information, moreover it contains the main structure of the code which allows the robot to avoid hitting wall and drive through the circuit without any problem. Furthermore the node permits to increment/decrement the velocity of the robot and reset its position (through the inputs given from keyboard and "passed" by the custom service `UserInterface.srv` (in the srv folder) that handles two elements: char command, that is the request from the client and float32 value, that is the response from the server; indeed this node is the server node that receives the request from the user node (client node). 
In the `base_scan` topic, which provides datas about the laser that scans the surrounding environment, there is the type message `sensor_msgs/LaserScan`. The topic provides an array, which returns the distances between the robot and the obstacles; every distance is given by the array ranges[i] (i = 0,...,720) and it is computed considering each of 721 section in which the vision of the robot is divided, since the vision of the robot is included in a spectrum range of 180 degrees in front of it and expressed in radiant. 3 big subsections are considered (right, left and in front of the robot), inside the 0-720 spectrum, for the vision of the robot and i have computed the minimum distance between the robot and the obstacle for each subsection. 
This is the function:

```cpp
double minimum(int first_index, int last_index, float array[]){

	//as the minimum distance is set a large value that will then be compared
	//with all the values between the first value and the last one in the ranges array
	double min_dist = 35;
	
	for(int i = first_index; i<= last_index; i++){
		if(array[i]<=min_dist){
			min_dist = array[i];	
		}
		
	}
	return min_dist;
}
```
These are the minimun distances for each subsection (done in the Driver function):

```cpp
	double front, right, left; //minimum distances on the three directions
	//right measure from 0° to 30°
	right = minimum(0, 120, info_distance);
	//front measure from 75° to 105°
	front = minimum(300, 420, info_distance);
	//left measure from 150° to 180°
	left = minimum(600, 720, info_distance);
}
```
The node permits to increment/decrement the velocity of the robot and reset its position and it is made handling the request coming from the user node (no response are sent to the user node) : 

```cpp
	// if the request sent from the user is 'i' the velocity is incremented
	if(req.command == 'i'){
		custom_acc += 0.5;
		
		//ROS_INFO("\nIncrement of acceleration :@[%f]", vel.linear.x);
	}
	//if the request sent from the user is 'd' the velocity is decreased
	if(req.command == 'd'){
		custom_acc -= 0.5;
		
		
		//ROS_INFO("\nIncrement of acceleration :@[%f]", vel.linear.x);
			
	}
	//if the request sent from the user is 'r' robot goes back to the initial position
	if(req.command == 'r'){
		ros::service::call("/reset_positions", pose_res);
	}
	
	if(req.command != 'i' && req.command != 'd' && req.command != 'r'){
		std::cout<<"NOT SUPPORTED\n"<<std::endl;
		fflush(stdout);
	}
	 
	res.value = custom_acc;
	return true;

```
### custom_acc is a globar variable that increment/decrement by 0.5 each time a keyboard's button (i/d) is pressed by the user. 

These are the simple statements in order to let the robot drive easily through the circuit:
The following control is implemented in the Driver function that will be called whenever a message is posted on the `base_scan` topic.

```cpp
	//if there is a curve to do
	if(front<d_th){
		//if right wall is near wrt left wall turn right
		if(right<left){
			//turn_right();
			vel.linear.x = 0.5;
			vel.angular.z = 1;
		}
		//if the opposite turn left
		else if(right>left){
			//turn_left();
			vel.linear.x = 0.5;
			vel.angular.z = -1;
		}
	
	}
	//if no curve to do
	else{
		vel.linear.x = 1.0 + custom_acc;
  		vel.angular.z = 0.0;
  		if(vel.linear.x<=0){
  			vel.linear.x = 0.0;
  		}
	}
	
	pub_vel.publish(vel);
```
#### If the velocity is drastically incremented the robot crushes.

The node also behaves like a PUBLISHER since it publish on the topic `/cmd_vel` the type message `cmd_vel geometry_msgs/Twist`, that regards the velocity of the robot, broken in its angular and linear parts (x,y,z). 

### UI node

The UI node represents the interface of the user. Through the user node we can increase/decrease the velocity of the robot and reset its position by simple commands:
* i --> accelerate the robot by 0.5
* d --> decelerate the robot by 0.5 
* r --> reset robot position. 

All the commmands are detected from the keyboards thanks to this function:

```cpp
char Get_command(void){
	char comm;
	
	std::cout<<"Welcome player!\n";
	std::cout<<"If you want to increase the speed press <i>\n";
	std::cout<<"If you want to decrease the speed press <d>\n";
	std::cout<<"If you want to reset the position press <r>\n";
	std::cout<<"Please select: ";
	std::cin>>comm;
	
	return comm;
}
```
The service UserInterface.srv is made like this:

``` xml
     char command
     ---
     float32 value
```
Thanks to this service the request is sent to the robot_controller node (the request is a char, the one pressed on keyboard) and the server node (control node) will manage the request. No response will be sent to the user node (the response should have been a float32 value) since the service operate directly on the control node!

Nodes' Connection
---------

This is an image that show how nodes are connected to each other (thanks to the command rosrun rqt_graph rqt_graph):

<p align="center">
<img src="https://github.com/NabStaio/RT1_SecondAssignment_2021-2022/blob/main/images/rosgraph.png" width="900" height="150">
</p>


Conclusions
-----------

The robot is able to do a complete lap of the circuit without crushing on walls. Nevertheless, it has problems when its speed is drastically increased or when a continuous race with more than 2 laps is done. Therefore, the code could be improved by having an infinity race withouth stopping, also the UI could be more entertaining by adding more visual features. Hovewer this work in my opinion is sufficient to fulfill the requirements given by the professor. 
