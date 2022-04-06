#! /usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import sys, select, termios, tty

#########################################################################################
# ASSISTANCE NODE:                                                                      #
# This node is similar to the teleoperation node, but in addition we have an autonomous #
# obstacle avoidance by introducing the CLBK_LASER() that will guide the robot to take  #
# actions according to distances from obstacles and it will keep away from them         #
#########################################################################################

pub = None #publisher that is in the take_actions() and in the main

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)
    
#############################################################
# CLBK_LASER():                                             #
# Arguments--> msg                                          #
# This function is a callback that defines some regions     #
# related to the presence of an obstacle.                   #
# Then the robot will take the correspondent action through #
# take_action(regions)                                      #
#############################################################
    
def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    take_action(regions)
    
##############################################################
# TAKE_ACTION():                                             #
# Arguments--> regions                                       #
# This function is built to avoid obstacles passed as        #
# arguments. Only three if statements are used for a         #
# generalization purpose: front, front and fright,           #
# front and fleft. If the obstacle is in front of you stop,  #
# if it's front and right turn left, if it's front and left  #
# turn right. The message of type geometry_msgs/Twist is     #
# then publish on /cmd_vel for the control                   #
##############################################################

def take_action(regions):
    msg = Twist()


    #state_description = ''

    if regions['front'] < 1:
    	  
    	#state_description = 'FRONT OBSTACLE! STOP!'
    	vel.linear.x = 0
    	vel.linear.y = 0
    	vel.linear.z = 0

    	vel.angular.x = 0
    	vel.angular.y = 0
    	vel.angular.z = 0
    	


    elif regions['front'] < 1 and regions['fright'] < 1:
        #state_description = 'FRONT&RIGHT OBSTACLES! TURN LEFT!'
        vel.linear.x = x*speed
        vel.linear.y = y*speed
        vel.linear.z = z*speed
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = turn
       
    elif regions['front'] < 1 and regions['fleft'] < 1:
        #state_description = 'FRONT&LEFT OBSTACLES! TURN RIGHT!'
        vel.linear.x = x*speed
        vel.linear.y = y*speed
        vel.linear.z = z*speed
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = -turn
        
    #else:
        #state_description = 'unknonwn'
        

    
    pub.publish(msg)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    

    rospy.init_node('assistance')
    rate = rospy.Rate(5)
    
    curr_modality = rospy.get_param('/active')
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser) #subscriber to scan to retrieve distances
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #publisher to /cmd_vel
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)


        while not rospy.is_shutdown():
            
            curr_modality = rospy.get_param('/active') #get the param of the current modality
            #if i'm in ASSISTANCE MODE
            if curr_modality == 3 :
             
             key = getKey(key_timeout)
             if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
             elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
             else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
 
             pub_thread.update(x, y, z, th, speed, turn)
            
            else:
             rate.sleep()
    except Exception as e:
        print(e)

    finally:
        #pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
