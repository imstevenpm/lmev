#!/usr/bin/env python

# Python libs
import numpy
import tf
import thread

# ROS libraries
import roslib
import rospy

# ROS messages
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# Class that does all the work
class Light_Finder_Algorithm:
	
	
	# Initialize ROS publishers and variables
	def __init__(self):
		
		# Topics where we publish
		self.Move_Tank= rospy.Publisher('/cmd_vel', Twist, queue_size=0)
		self.Move_Arm= rospy.Publisher('/Arm_Motor/vel_cmd', Float64, queue_size=0)
		
		# ROS msg for Move_Tank
		self.msg=Twist()
		self.msg.linear.x=0
		self.msg.linear.y=0
		self.msg.linear.z=0
		self.msg.angular.x=0
		self.msg.angular.y=0
		self.msg.angular.z=0
		
		# Flux variables
		self.Done = 0
		self.Thread = [1,1]
		
		# Flags
		# Current task executing
		# wall, search, exit
		self.Flags = [0,0,0]
		
		# Main threshold variables
		# close_wall (cm), far_wall (cm), exit (cm), min_light (ptc), goal_light (ptc)
		self.MThresholds = [40,60,20,40,80]
		
		# Secondary threshold variables
		# turning_speed, turning_back, cruise_speed, cruise_back
		self.SThresholds = [2.0,2.0,0.15,-0.20]
		
		# Gives time to publishers and subscriber to init
		rospy.sleep(3)
		
		# Guarantees it makes first the follow wall task
		self.enable_flag(0)
	# Algorithm's flux functions
	
	# Defines task priority
	def can_enter(self,mode):
		can=0
		
		# Cruise task enters only if other task are disabled
		if mode==0:
			can=(not self.Flags[0]) and (not self.Flags[1]) and (not self.Flags[2])
			
		# Follow wall task enters only if Searching and Exit are disabled
		elif mode==1:
			can=(not self.Flags[1]) and (not self.Flags[2])
		
		# Searching task enters only if Exit are disabled
		else:
			can= (not self.Flags[2])

		return can
	
	
	# Tells that a new task is working
	def enable_flag(self,mode):
			
		if mode==0:
			self.Flags[0]=1
		elif mode==1:
			self.Flags[1]=1
		else:
			self.Flags[2]=1
		
		
	# Tells that the task ended working	
	def disable_flag(self,mode):
			
		if mode==0:
			self.Flags[0]=0
		elif mode==1:
			self.Flags[1]=0
		else:
			self.Flags[2]=0
	
	
	# Robot's movement functions
	
	# Spin the robot's orientation	
	# The robot's spin can also be done by reading the gyroscope sensor to spin an exact number of degrees or by reading the motors encoders to spin an exact number of wheels rotations 	
	def spin(self,sec,speed):
		
		# Working with time is not the best practices, since it takes ROS always different time to perform the work. It is better to work with number of rotations. If you decide to use time, you will need commands lines like the one below, to guarantee that the motor velocities in simulation, will be the same as the real ones. Since V=d/t
		sec=sec/10.0;
		
		# Set x-linear speed to zero and z-angular speed to speed
		self.msg.linear.x=0
		self.msg.angular.z=speed
		
		# Publish msg in ROS Topic
		self.Move_Tank.publish(self.msg)
		
		# Waits sec seconds and then stops the turn by publishing a z-angular speed to zero
		rospy.sleep(sec)
		self.msg.angular.z=0
		self.Move_Tank.publish(self.msg)
		
		# After a robot turn, usually a sensor readings comes next, so this keeps the robot still while taking the measure
		rospy.sleep(0.5)
	
	
	#	Moves the robot foward or backward
	# This function can also be done by reading the motors encoders to move an exact number of wheels rotations
	def foward(self,sec,speed):
		self.msg.angular.z=0
		self.msg.linear.x=-speed
		self.Move_Tank.publish(self.msg)
		rospy.sleep(sec)
	
	
	# Auxiliar function of Search task
	
	# Finds the direction of more light
	def light_triang(self):
		
		light=self.MThresholds[3]
		
		# Direction of more light. 0=foward, 1=left, 2=right
		direction=0
		
		# Reads the light sensor value
		aux=rospy.wait_for_message('Color_Sensor/Ambient', Int16)
		
		# If it higher of the goal threshold, finish algorithm
		if (aux.data>self.MThresholds[4]):
			self.Thread[0]=0
		
		else:
		
			# Spins toward the left and take a measure
			self.spin(1.5,-self.SThresholds[0])
			aux=rospy.wait_for_message('Color_Sensor/Ambient', Int16)
		
			# If it higher of the minimum value
			if (aux.data>light):
				
				# If it higher from the goal value, finish algorithm
				if (aux.data>self.MThresholds[4]):
					self.Thread[0]=0
					return 0
				
				# Else, this is the new higher light direction
				else:
					light=aux.data
					direction=1
			
			# Spins toward the right and take a measure
			self.spin(3,self.SThresholds[0])
			aux=rospy.wait_for_message('Color_Sensor/Ambient', Int16)
			
			# If it higher from the new higher light direction
			if (aux.data>light):
			
				# If it higher from the goal value, finish algorithm
				if (aux.data>self.MThresholds[4]):
					self.Thread[0]=0
					return 0
				
				# Else, this is the new higher light direction		
				else:
					direction=2
			
			# Go back to the starting position		
			self.spin(1.5,-self.SThresholds[0])
		
		# Send direction
		return direction
		
	
	# Tasks
	
	# Prevents the robot from hit the wall and corrects its orientation
	def exit(self):
		
		# While the algorithm haven't finished
		while self.Thread[0]:
		
			# Reads the sensor values
			US=rospy.wait_for_message('Ultrasonic_Sensor', Float32)
			TS=rospy.wait_for_message('Touch_Sensor', Int16)
			
			# If the ultrasonic measure is less than the exit threshold or if the touch sensor is pressed
			if ((US.data < self.MThresholds[2]) or (TS.data==1)):
			
				# Go backwards and spin, start with follow wall next
				print("Exit Task")
				self.enable_flag(2)
				self.foward(2,self.SThresholds[3])
				self.spin(2,self.SThresholds[1])
				self.disable_flag(2)
				self.enable_flag(0)
				
				# How often the task is executed
				# Important to consider, the tasks need to work in synchrony
				rospy.sleep(0.5)
	
	def search(self):
		
		# While the algorithm haven't finished
		while (self.Thread[0]):
		
			# Reads light sensor value
			CS = rospy.wait_for_message('Color_Sensor/Ambient', Int16)
			
			# If it less than the goal value
			if (CS.data<self.MThresholds[4]):
				
				# Only if there isn't a major task running and if the value is higher of the minimum light threshold value
				if((self.can_enter(2)) and (CS.data>self.MThresholds[3])):
				
					print("Search light task")
					self.enable_flag(1)
					
					# Disable Follow wall and Cruise tasks
					self.Thread[1]=0
					
					# Waits until the Follow wall task finishes
					if (not self.Done):
						while (self.Flags[0]):
							rospy.sleep(1)
						self.Done=1
					
					# Calculate the direction of more light
					direction=self.light_triang()
					
					# Spins toward the direction of more light and advance that way
					if direction ==1:
						print("Go left")
						self.spin(2,-self.SThresholds[0])
						self.foward(0.5,self.SThresholds[2])
						
					elif direction==2:
						print("Go right")
						self.spin(2,self.SThresholds[0])
						self.foward(0.5,self.SThresholds[2])
					
					# If direction==0, the robot have found the light bulb	
					elif direction==0:
					
						# Goes toward it, stops and moves the arm to indicate the algorithm have finished
						print("Go foward")
						self.foward(0.25,self.SThresholds[2])
						self.foward(0.5,0)
						print("Found it! :)")
						self.Move_Arm.publish(-1.5)
						rospy.sleep(2)
						self.Move_Arm.publish(1.0)
						
					self.disable_flag(1)
					
					# How often the task is executed
					rospy.sleep(1)
					
			
			else:
				
				self.Thread[0]=0
				print("Go foward")
				self.foward(0.25,self.SThresholds[2])
				self.foward(0.5,0)
				print("Found it! :)")
				self.Move_Arm.publish(-1.5)
				rospy.sleep(2)
				self.Move_Arm.publish(1.0)
	
	
	# Spins the robot toward the nearest left wall if it is away from  it and spins to the other direction if it is too close from it
	def follow_wall(self):
		
		# While the algorithm haven't finished neither the Search task have started
		while (self.Thread[0] and self.Thread[1]):	
			
			# Only if there isn't a major task running
			if (self.can_enter(1)):
			
				print("Follow wall task")
				self.enable_flag(0)
				
				# Turns, take an ultrasonic sensor measure and go back
				self.spin(2,-self.SThresholds[0])
				US3 =  rospy.wait_for_message('Ultrasonic_Sensor', Float32)
				self.spin(2,self.SThresholds[0])
				
				# This if is for when the Search task takes control
				# It prevents from changing the robot's orientation when a higher light value have been seen
				if (self.can_enter(1)):
					
					# Spin toward the other direction if it is too close
					if (US3.data<self.MThresholds[0]):
						print("Go away from wall")
						self.spin(2,self.SThresholds[0])
						self.disable_flag(0)
					
					# Spin toward the left wall if it is far away
					elif (US3.data>self.MThresholds[1]):
						print("Go close to wall")
						self.spin(2,-self.SThresholds[0])
						self.disable_flag(0)
						
				self.disable_flag(0)
				
				# How often the task is executed
				rospy.sleep(2)
	
	# Moves the robot foward		
	def cruise(self):
	
		# While the algorithm haven't finished neither the Search task have started
		while (self.Thread[0] and self.Thread[1]):
		
			# Only if there isn't a major task running
			if (self.can_enter(0)):
			
				# Go foward
				print("Cruise task")
				self.foward(1,self.SThresholds[2])
				
				# How often the task is executed
				rospy.sleep(1.5)
		
# Main
def main():
    
		# Initializes and cleanup ROS node
    rospy.init_node('Light_Finder_Algorithm', anonymous=True)
    
    # Start an object
    LFA = Light_Finder_Algorithm()
    
    # Keeps the Arm up
    LFA.Move_Arm.publish(0.5)
    
    # Starts tasks in different python threads
    thread.start_new_thread(LFA.exit,())
    thread.start_new_thread(LFA.search,())
    thread.start_new_thread(LFA.follow_wall,())
    thread.start_new_thread(LFA.cruise,())
    
    try:
    		# Keeps the ROS Node running
    		# Listens for new msgs in the subscribed topics
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Light_Finder_Algorithm"

if __name__ == '__main__':
    main()
