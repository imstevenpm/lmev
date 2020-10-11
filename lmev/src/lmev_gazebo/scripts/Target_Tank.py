#!/usr/bin/env python

# Python libs
import numpy
import tf

# ROS libraries
import roslib
import rospy

# ROS messages
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# Class that does all the work
class Target_Tank:

	# Initialize ROS publishers, subscribers and variables
	def __init__(self):
		
		# Topics where we publish
		self.Move_Tank= rospy.Publisher('/cmd_vel', Twist, queue_size=0)
		
		# Subscribed topics
		# Waiting for the number of reoltions to make
		self.Rev_Subscriber = rospy.Subscriber('/Move_Tank_Rev/target_cmd', Int16, self.Rev_Processor)
		
		# Variables
		self.Encoder_Data = [0]
		
		# Twist msg
		self.msg=Twist()
		# 0.2562 m/s = 50 in lmev brick
		self.msg.linear.x= -0.2562
		self.msg.linear.y=0.0
		self.msg.linear.z=0.0
		self.msg.angular.x=0.0
		self.msg.angular.y=0.0
		self.msg.angular.z=0.0
		
		# Gives time to publishers and subscribers to init
		rospy.sleep(3)
	
	# Callback function for a new msg in the subscribed topic
	def Rev_Processor(self,Target_Data):
		
		# Store actual encoder value
		# We will consider only the left encoder
		self.Encoder_Data[0]=rospy.wait_for_message('Encoder_Left_Wheel_Sensor', Int16)
		
		# Declare desired encoder value
		Target_Angle= self.Encoder_Data[0].data+360*Target_Data.data
		
		# Untill the encoder value is equal to the desire angle
		while (self.Encoder_Data[0].data<Target_Angle):
			
			# Move the tank with a speed of 0.2562 m/s (50 in brick)
			self.Move_Tank.publish(self.msg)
			
			# Store new encoder value	
			self.Encoder_Data[0]=rospy.wait_for_message('Encoder_Left_Wheel_Sensor', Int16)
		
		# When bigger, dont move it anymore	
		self.msg.linear.x=0.0
		self.Move_Tank.publish(self.msg)

# Main
def main():
    
		# Initializes and cleanup ROS node
    rospy.init_node('Target_Tank', anonymous=True)
    TT = Target_Tank()
    
    try:
    		# Listens for new msgs in the subscribed topics
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Target Tank"

if __name__ == '__main__':
    main()
