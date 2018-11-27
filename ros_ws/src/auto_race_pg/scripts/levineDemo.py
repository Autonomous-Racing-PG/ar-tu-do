#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
#import matplotlib.pyplot as plt

angle_range = 180
# angle_range = np.pi
car_length = 1.5

desired_trajectory = 0

vel = 1
path = [1,0,1,1,1]
turn = 0
preva = 0
flag = 0
errorList = [0,0,0,0,0]
turnStarted = 0
error = 0.0
alpha = 0.0
final_desired_trajectory = -1
final_direction = 1

left_dist = []
objbuffer = [-1]*25

flag_left = 0

pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data,angle):
	if angle > 179.9:
		angle = 179.9
	# if angle > np.pi-1e-4:
		# angle = np.pi-1e-4
	index = len(data.ranges)*angle/angle_range
	dist = data.ranges[int(index)]
	if math.isinf(dist) or math.isnan(dist):
		return 4.0
	return data.ranges[int(index)]


def obs_particles(data, start, end, distance):
	global alpha
	front_values = []
	num_points = 0
	obs = 0
	alpha_degree = int(math.degrees(alpha))
	k = -1
	for i in range(start - alpha_degree, end - alpha_degree):
	# for i in range(start - alpha, end - alpha):
		k = k+1
		front_values.append(getRange(data,i)*math.sin(math.radians(i+alpha_degree)))
		# front_values.append(getRange(data,i)*math.sin(i+alpha))
		if front_values[k] <= distance:
			num_points = num_points + 1
	return front_values,num_points	

def obs_decide(data):
	global alpha
	start = 84
	end = 96
	# start = 84*np.pi/180
	# end = 96*np.pi/180
	distance = 2.0
	values,num_points = obs_particles(data,start,end,distance)
	# print "In range", values
	start_point = 0
	end_point = 0
	#Detects if its a clear path
	print "Num Points", num_points

	if num_points < 3:
		print "Go go go - clear path"
		return -1,-1
	#Detects if there is an obstacle	
	elif num_points <= 15:
		print "normal obstacle"
		k =-1
		for i in values:
			k = k + 1
			if i<= (distance):
				start_point = k + start
				break
		k = -1
		for i in reversed(values):
			k = k+1
			if i<= (distance):
				end_point = end - k
				break
		#if the obstacle covers the previous any one of previous limits, expand the range
		if start_point <= (start+1):
			print "Start point extended"
			start1 = start - 10
			end1 = start
			start_point = start1
			values,num_points = obs_particles(data,start1,end1,distance)
			print "Right extended", values
			k = 0
			for i in reversed(values):
				k = k + 1
				if i > (distance):
					start_point = end1 - k
					break
		
		if end_point >= (end-1):
			start2 = end + 1
			end2 = end + 10
			end_point = end2
			values,num_points = obs_particles(data,start2,end2,distance)			
			print "Right extended", values
			k = len(values)-1
			for i in values:
				k = k-1
				if i > (distance):
					end_point = end2 - k
					break
		
		print "Start Point", start_point
		print "End Point", end_point
		return start_point,end_point
	
	else:
		print "wide obstacle"
		#Looks like a wide obstacle or wall. Start from expanded range
		start1 = start - 10
		end1 = start - 1
		start_point = end1 + 3
		
		values,num_points = obs_particles(data,start1,end1,distance)
		k = len(values) - 1
		for i in reversed(values):
			k = k - 1
			if i > (distance):
				start_point = k + start1
				break

		start2 = end + 1
		end2 = end + 10
		end_point = start2 - 3
		
		values,num_points = obs_particles(data,start2,end2,distance)			
		k = len(values)-1
		for i in values:
			k = k-1
			if i > (distance):
				end_point = end2 - k
				break

		#Looks like a wall if following conditions satisfy
		print "wall"		
		if start_point <= start1+1:
			start_point = -1
		if end_point >= end2-1:
			end_point = -1

		print "Start Point", start_point
		print "End Point", end_point

		return start_point,end_point


def decide_obstacle_direction(data,start_point,end_point):
	global alpha
	left = 0
	right = 1
	centre = 2
	stop = 3
	alpha_degree = int(math.degrees(alpha))
	desired_trajectory = -1

	direction = centre
	
	if start_point!=-1 or end_point!=-1:
		laser_start = getRange(data,start_point-alpha_degree)
		laser_end = getRange(data,end_point-alpha_degree)
		
		start_pointdistance = laser_start*math.cos(math.radians(start_point))
		end_pointdistance = laser_end*math.cos(math.radians(end_point))

		print "Right Width",start_pointdistance
		print "left Width",end_pointdistance
		car_dist_right = getRange(data,0)*math.cos(alpha)
		car_dist_left = getRange(data,179)*math.cos(alpha)
		
		print "Car dist right",car_dist_right
		print "Car dist left",car_dist_left

		obstacle_distance_left = 0.0
		obstacle_distance_right = 0.0

		# if (start_point-alpha_degree) > 90:
		# 	obstacle_distance_right = car_dist_right + start_pointdistance
		# else:
		obstacle_distance_right = car_dist_right - start_pointdistance

		# if (end_point-alpha_degree) > 90:
		# 	obstacle_distance_left = car_dist_left - end_pointdistance
		# else:
		obstacle_distance_left = car_dist_left + end_pointdistance

		print "Right edge",obstacle_distance_right
		print "left edge",obstacle_distance_left

		if (obstacle_distance_left > obstacle_distance_right):# and obstacle_distance_left > 0.4:
			desired_trajectory = obstacle_distance_left/2
			direction =left

			print "left"

		elif (obstacle_distance_left < obstacle_distance_right):# and obstacle_distance_right > 0.4:
			desired_trajectory = obstacle_distance_right/2
			direction =right

			print "right"

		else:
			direction = stop
	else:
		desired_trajectory = -1
		direction = centre

		print "Go centre"

	desired_trajectory = -1
	direction = centre

	print "Go centre"


	print "Desired dist", desired_trajectory
	return desired_trajectory, direction

def followRight(data,desired_trajectory):
	global alpha

	a = getRange(data,60)
	b = getRange(data,0)
	swing = math.radians(60)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "a","b", a, b
	print "Alpha right",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)

	future_dist = curr_dist+car_length*math.sin(alpha)

	print "Right : ",future_dist
	error = desired_trajectory - future_dist
	print "Error : ",error
	return error, curr_dist

def followLeft(data,desired_trajectory):
	global alpha

	a = getRange(data,120)
	b = getRange(data,179.9)
	swing = math.radians(60)
	print "a","b", a, b
	alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha left",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)
	future_dist = curr_dist-car_length*math.sin(alpha)

	print "Left : ",future_dist

	error = future_dist - desired_trajectory
	return error, curr_dist


def followCentre(data,desired_trajectory):
	global alpha

	a = getRange(data,130)
	b = getRange(data,179.9)
	swing = math.radians(50)
	print "center distances: ", a, b
	alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha left",math.degrees(alpha)
	curr_dist1 = b*math.cos(alpha)
	future_dist1 = curr_dist1-car_length*math.sin(alpha)



	a = getRange(data,50)
	b = getRange(data,0)
	swing = math.radians(50)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha right",math.degrees(alpha)
	curr_dist2 = b*math.cos(alpha)

	future_dist2 = curr_dist2+car_length*math.sin(alpha)

	desired_trajectory = (future_dist1 + future_dist2)/2

	print "dist 1 : ",future_dist1
	print "dist 2 : ",future_dist2
	# print "dist : ",future_dist
	error = future_dist1 - future_dist2
	print "Error : ",error
	return error, curr_dist2-curr_dist1



def decideReturn(start_point,end_point):
	global objbuffer
	objbuffer.append(start_point)
	objbuffer.append(end_point)
	del objbuffer[0]
	del objbuffer[0]
	if objbuffer.count(-1)==len(objbuffer):
		return 1
	else:
		return 0

def callback(data):
	global error
	global alpha
	print " "
	global flag_obstacle
	global final_desired_trajectory
	global final_direction
	global prev_direction
	global flag_left
#	a = getRange(data,50)
#	b = getRange(data,0)
#	# c = getRange(data,40)
#	swing = math.radians(50)
#	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
#	if flag_obstacle == 0:
	start_point,end_point = obs_decide(data)
#	else:
#		start_point = -1
#		end_point = -1

	final_desired_trajectory, direction = decide_obstacle_direction(data,start_point,end_point)
#	if desired_trajectory!=1.0:
#		flag_obstacle = 1
	# if direction == 0:
	# 	flag_left = 1
	# elif direction == 2:
	# 	flag_left = 0
	
	# if flag_left == 1:
	# 	direction = 0
	
	# if direction == 0:
	# 	final_desired_trajectory = 0.4
	# 	final_direction = direction
	# elif desired_trajectory != -1:
	# 	final_desired_trajectory = desired_trajectory
	# 	final_direction = direction

	# car_dist_right = getRange(data,0)*math.cos(alpha)
	# car_dist_left = getRange(data,179)*math.cos(alpha)
		
	# if decideReturn(start_point,end_point) == 1:# and (car_dist_left + car_dist_right) > 1.0:
	# 	print "reset done"
	# 	final_desired_trajectory = 0.8
	# 	final_direction = 1
	# 	#flag_obstacle = 0
	# #final_direction= 0
	# #final_desired_trajectory = 1
	# print "Final Desired",final_desired_trajectory
	# print "new code"
	# if final_direction == 0:
	error_right, curr_dist_right = followRight(data,0.05)#followRight(data,2.)
	error_left, curr_dist_left = followRight(data,0.05)#followLeft(data,1.)
	# error_center, curr_dist_center = followCentre(data,1.)
	if curr_dist_right >= curr_dist_left:
		error = error_left
		print 'Following Left'
		print 'Error', error
	else:
		# error = followRight(data,final_desired_trajectory)
		error = error_right
		print 'Following Right'
		print 'Error', error

	# if curr_dist_center:


	print 'Is error same?', error
	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("/racer/laser/scan", LaserScan, callback)
	rospy.spin()
