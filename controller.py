#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ HB_2059 ]
# Author List:	[ Atharva Jadhav, Nikhil Isaac, Animesh Wankhede ]
# Filename:		[controller.py]
# Functions: 	[ signal_handler,cleanup,task2_goals_cb,aruco_goals_cb,euclidean_distance, steering_angle, linear_vel_x, linear_vel_y, angular_vel,convert_angle, convert_angle_2pi,PID, inverse_kinematics, move2goal]
# Nodes:		[controller_node,aruco_feedback, gazebo]


################### IMPORT MODULES #######################
import numpy
import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

hola_x = 0
hola_y = 0
hola_theta = 0

PI = 3.14

x_goals = []
y_goals = []
theta_goals = []

right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)
rate = rospy.Rate(100)


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Not mandatory - but it is recommended to do some cleanup over here,
	#	   to make sure that your logic and the robot model behaves predictably in the next run.

	############################################
	rw = Wrench()
	lw = Wrench()
	fw = Wrench()

	rw.force.x = 0	
	lw.force.x = 0
	fw.force.x = 0
	

	right_wheel_pub.publish(rw)
	front_wheel_pub.publish(lw)
	left_wheel_pub.publish(fw)
  
def task2_goals_Cb(msg):
	global x_goals, y_goals, theta_goals
	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def aruco_feedback_Cb(msg):
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Receive & store the feedback / coordinates found by aruco detection logic.
	#	-> This feedback plays the same role as the 'Odometry' did in the previous task.

	############################################
	global hola_x, hola_y, hola_theta
	hola_x = msg.x
	hola_y = msg.y
	hola_theta = msg.theta

def euclidean_distance(x1, y1, x2, y2):
	return math.sqrt((x2-x1)**2 + (y2-y1)**2)


def steering_angle(goal_x, goal_y):
    global hola_x, hola_y, hola_theta
    return math.atan2(goal_y - hola_y, goal_x - hola_x)

def linear_vel_x(goal_x, goal_y, constant=2):
    global hola_x, hola_y, hola_theta
    return constant * math.cos(steering_angle(goal_x, goal_y)-hola_theta)

def linear_vel_y(goal_x, goal_y, constant=2):
    global hola_x, hola_y, hola_theta
    return constant * math.sin(steering_angle(goal_x, goal_y)-hola_theta)

def angular_vel(goal_theta, constant=3):
    return constant * (goal_theta-hola_theta)

def convert_angle(theta):
    theta = theta%(2*math.pi) 
    if theta > math.pi or theta < -math.pi:
        theta = theta - 2*math.pi if theta > 0 else theta + 2*math.pi
    return theta

def convert_angle_2pi(theta):
	if theta < 0:
		theta = theta + 2*math.pi
	return theta

def PID(Kp, Ki, Kd, MV_bar=0):
	e_prev = 0
	t_prev = -100
	I = 0
    
	MV = MV_bar
    
	while True:
		t,PV, SP = yield MV

		e =  PV - SP

		P = Kp*e
		I = I + Ki*e
		D = Kd*(e - e_prev)/(t - t_prev)

		MV = MV_bar + P + I + D

		e_prev = e
		t_prev = t


def inverse_kinematics(vx, vy, w,kp=1):
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use the target velocity you calculated for the robot in previous task, and
	#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
	#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
	############################################
	global hola_x, hola_y, hola_theta
	matrix = numpy.array([[-0.17483,1,0],
					[-0.17483,-.5,-.86602540378],
					[-0.17483,-.5,.86602540378]]
					)
	vel_s = numpy.array([[w],[vx],[vy]])
	forces = matrix@vel_s
	return kp*forces









def move2goal(x_goal, y_goal, theta_goal,pixel_range=False):
	global hola_x, hola_y, hola_theta

	pi_goal = convert_angle(theta_goal)

	distance_tolerance = 0.1
	angle_tolerance = 0.02

	fw = Wrench()
	rw = Wrench()
	lw = Wrench()

	pid = PID(2, 0.00008,0.000008)
	pid.send(None)
	error =euclidean_distance(x_goal, y_goal,hola_x,hola_y) 
	while error>= distance_tolerance or abs(hola_theta-pi_goal) > angle_tolerance:
		# print("x: ", hola_x, "y: ", hola_y, "theta: ", hola_theta)
		forces = inverse_kinematics(linear_vel_x(x_goal, y_goal,error/5), linear_vel_y(x_goal, y_goal,error/5), pid.send([time.time(),convert_angle_2pi(hola_theta) if pixel_range else hola_theta,theta_goal]),15)
		fw.force.x = forces[1][0]
		rw.force.x = forces[2][0]
		lw.force.x = forces[0][0]

		right_wheel_pub.publish(rw)
		front_wheel_pub.publish(lw)
		left_wheel_pub.publish(fw)
		error =euclidean_distance(x_goal, y_goal,hola_x,hola_y)
		rate.sleep()
	pid.close()
	cleanup()
	time.sleep(2)




def main():

	global x_goals, y_goals, theta_goals, hola_x, hola_y, hola_theta
	rospy.init_node('controller_node')
	signal.signal(signal.SIGINT, signal_handler)

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
	
	rate = rospy.Rate(100)

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Make use of the logic you have developed in previous task to go-to-goal.
	#	-> Extend your logic to handle the feedback that is in terms of pixels.
	#	-> Tune your controller accordingly.
	# 	-> In this task you have to further implement (Inverse Kinematics!)
	#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
	#      given velocity of the chassis (Vx, Vy, W)
	#	   

		
	while not rospy.is_shutdown():
		
		# Calculate Error from feedback

		# Change the frame by using Rotation Matrix (If you find it required)

		# Calculate the required velocity of bot for the next iteration(s)
		
		# Find the required force vectors for individual wheels from it.(Inverse Kinematics)

		# Apply appropriate force vectors

		# Modify the condition to Switch to Next goal (given position in pixels instead of meters)
		for x_goal, y_goal, theta_goal in zip(x_goals, y_goals, theta_goals):
			print(x_goal, y_goal, theta_goal)
			theta_goal = convert_angle_2pi(theta_goal)
			pixel_goal = convert_angle_2pi(theta_goal)
			pixel_range = abs(pixel_goal - convert_angle_2pi(hola_theta))<abs(theta_goal - hola_theta)
			move2goal(x_goal, y_goal,pixel_goal if pixel_range else theta_goal, pixel_range)
		x_goals.clear()
		y_goals.clear()
		theta_goals.clear()
		rate.sleep()
		rate.sleep()

    ############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

