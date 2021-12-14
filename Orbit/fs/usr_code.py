"""
October 7 2021
Lab 1 - ME409
Ian Kennedy
"""

from logging import error


#Global variables for robot 1 velocity updates
pose_rxed_old = [0,0,0]
dist_old =0
i = 0


def usr(robot):
	import struct
	import math
	time = robot.get_clock()
	last = time
	desired_distance=.25
	
	while True:
		if robot.id == 0:
			last = robot.get_clock()#gets time
			pose_t=robot.get_pose()# gets pose The syscall that returns the robot's global pose (x, y, theta).
			#Note that the onboard sensor has a limited sampling rate of 30HZ. If there is no data from received the sensor since the last time the function get_pose() is called, the syscall will return None. If there is new data received from the sensor, the syscall will return a 3-tuple (x, y, theta), which are robot's x position, y, position, and orientation, respectively.

			#if there is a new postion sensor update, print out and transmit the info
			
			if pose_t: #check pose is valid before using
				pose=pose_t
				robot.send_msg(struct.pack('ffi', pose[0], pose[1],robot.id))# send pose x,y in message

			msgs = robot.recv_msg()
			

			#If we receive a message, calculate the distance and set the robot LED
			if len(msgs) > 0:
				#Load 
				pose_rxed= struct.unpack('ffi', msgs[0][:12])

				if pose_t:
					dist = math.sqrt((pose_t[0]-pose_rxed[0])*(pose_t[0]-pose_rxed[0]) + (pose_t[1]-pose_rxed[1])*(pose_t[1]-pose_rxed[1]))
					
					if dist>desired_distance:
						robot.set_led(100,0,0)
					if dist<desired_distance:
						robot.set_led(0,100,0)
					print('Distance: ', dist) #print distance between robots

				
			#Some arbitrary velocity	
			robot.set_vel(0,0)


		if robot.id == 1:
			last = robot.get_clock()#gets time
			pose_t=robot.get_pose()# gets pose The syscall that returns the robot's global pose (x, y, theta).
			
			#Note that the onboard sensor has a limited sampling rate of 30HZ. If there is no data from received the sensor since the last time the function get_pose() is called, the syscall will return None. If there is new data received from the sensor, the syscall will return a 3-tuple (x, y, theta), which are robot's x position, y, position, and orientation, respectively.
			
			global i # global counter
			global pose_rxed_old #global variable to store X,Y positions
			global dist_old #global variable
	
			i += 1

			#if there is a new postion sensor update, print out and transmit the info
			if pose_t: #check pose is valid before using
				pose=pose_t
				robot.send_msg(struct.pack('ffi', pose[0], pose[1],robot.id))# send pose x,y in message
			
			msgs = robot.recv_msg()


			#Assign "old" distance at regular intervals
			if last > .1 and len(msgs)>0 and (i>500) and pose_t:
				pose_rxed_temp= struct.unpack('ffi', msgs[0][:12])
				pose_rxed_old[0] = pose_rxed_temp[0]
				pose_rxed_old[1] = pose_rxed_temp[1]
				dist_old = math.sqrt((pose_t[0]-pose_rxed_old[0])*(pose_t[0]-pose_rxed_old[0]) + (pose_t[1]-pose_rxed_old[1])*(pose_t[1]-pose_rxed_old[1]))
				i = 0 #reset counter


			#If we receive a message, calculate the distance, update wheel velocities and set the robot LED
			if len(msgs) > 0 and pose_t and last > 0.1:
						
				pose_rxed= struct.unpack('ffi', msgs[0][:12]) #load in the XY data
				
				#If a message is received, update velocity
				if pose_t:

					#Set default velocity	
					vel_l = 30
					vel_r = 30

					#Calculate distance between robots
					dist = math.sqrt((pose_t[0]-pose_rxed[0])*(pose_t[0]-pose_rxed[0]) + (pose_t[1]-pose_rxed[1])*(pose_t[1]-pose_rxed[1]))
					

					#Compare current distance to old distance
					compare  = dist - dist_old
					

					#Update robot LED color
					if dist>desired_distance:
						robot.set_led(100,0,0)
					if dist<desired_distance:
						robot.set_led(0,100,0)	



					#Update wheel velocities based on where the robot is, and if it is getting close or further from other robot
					if dist < desired_distance and compare < 0 :
						vel_r = 50
						vel_l = 5
					if dist < desired_distance and compare > 0 :
						vel_r = 30
						vel_l = 30
					if dist > desired_distance and compare > 0:
						vel_l = 50
						vel_r = 5
					if dist > desired_distance and compare < 0:
						vel_l = 30
						vel_r = 30
					
					#Update the robot wheel velocities
					robot.set_vel(vel_l,vel_r)



