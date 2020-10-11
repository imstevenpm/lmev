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
class Validation:
	
	# Initialize ROS publishers and variables
	def __init__(self):
		
		# Topics where we publish
		self.Move_Tank= rospy.Publisher('/cmd_vel', Twist, queue_size=0)
		self.Move_Arm= rospy.Publisher('/Arm_Motor/vel_cmd', Float64, queue_size=0)
		
		# ROS msg for Move_Tank
		self.msg=Twist()
		self.msg.linear.x=-0.2562
		self.msg.linear.y=0.0
		self.msg.linear.z=0.0
		self.msg.angular.x=0.0
		self.msg.angular.y=0.0
		self.msg.angular.z=0.0
		
		# Flux variables
		self.Thread = [1]
		self.Sensor_Data = [0,0,0]
		
		# Main threshold variables
		self.MThresholds = [60]
		
		# Gives time to publishers and subscriber to init
		rospy.sleep(3)
		
	def Tank_Rev(self,Target_Data):
		
		# Store actual encoder value
		self.Sensor_Data[0]=rospy.wait_for_message('Encoder_Left_Wheel_Sensor', Int16)
		
		# Declare desired encoder value
		Target_Angle= self.Sensor_Data[0].data+360*Target_Data
		
		# Untill the encoder value is equal to the desire angle
		while (self.Sensor_Data[0].data<Target_Angle):
			
			# Move the tank with a speed of 0.2562 m/s (50 in brick)
			self.Move_Tank.publish(self.msg)
			
			# Store new encoder value	
			self.Sensor_Data[0]=rospy.wait_for_message('Encoder_Left_Wheel_Sensor', Int16)
		
		# When bigger, dont move it more	
		self.msg.linear.x=0.0
		self.Move_Tank.publish(self.msg)
		
		return 0
	
	def Turn_Gyro(self,Target_Data):

		# Store actual encoder value
		self.Sensor_Data[1]=rospy.wait_for_message('Gyroscope_Sensor', Int16)
		
		# Untill the robot have moved 90 degress
		while (self.Sensor_Data[1].data>Target_Data):
			self.msg.angular.z=3.85
			self.Move_Tank.publish(self.msg)
			
			# Store new encoder value	
			self.Sensor_Data[1]=rospy.wait_for_message('Gyroscope_Sensor', Int16)
		
		# When bigger, dont move it more	
		self.msg.angular.z=0.0
		self.Move_Tank.publish(self.msg)
		
		return 0
			
	def begin_validation(self):
		
		# Just do it once
		while self.Thread[0]==1:
			
			# Begin only if theres enough light
			Color_Sensor=rospy.wait_for_message('Color_Sensor/Ambient', Int16)
			
			if (Color_Sensor.data>self.MThresholds[0]):
				
				self.Thread[0]=0
				
				self.Tank_Rev(10)
				
				self.Turn_Gyro(-90)
				
				self.Sensor_Data[2]=rospy.wait_for_message('Ultrasonic_Sensor', Float32)
				
				# Move the arm only if theres an object close
				if (self.Sensor_Data[2].data<50.0):
					self.Move_Arm.publish(-2.5)
					print("Ready :)")
			
# Main
def main():
    
		# Initializes and cleanup ROS node
    rospy.init_node('Validation', anonymous=True)
    
    # Start an object
    V = Validation()
    V.begin_validation()
    
    try:
    		# Keeps the ROS Node running
    		# Listens for new msgs in the subscribed topics
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Validation"

if __name__ == '__main__':
    main()
