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

# Class that does all the work
class Target_Motor:

	# Initialize ROS publishers, subscribers and variables
	def __init__(self):
		
		# Topics where we publish
		self.Move_A_Publisher= rospy.Publisher('/Arm_Motor/vel_cmd', Float64, queue_size=1)
		
		# Subscribed topics
		self.Encoder_A_Subscriber = rospy.Subscriber('/Arm_Motor/target_cmd', Int16, self.Encoder_A_Processor)
		
		# Variables
		self.Motor_Vel=[0.5]
		self.Encoder_Data = [0]
		
		# Gives time to publishers and subscribers to init
		rospy.sleep(3)
	
	# Callback function for a new msg in the subscribed topic
	def Encoder_A_Processor(self,Target_Data):
		
		# Store actual encoder value
		self.Encoder_Data[0]=rospy.wait_for_message('Encoder_Arm_Sensor', Int16)
		
		# Declare desired encoder value
		# This supposse relative movemt of the arm
		# Usefull if you want to specify a joint movement by number of rotations
		Target_Angle= self.Encoder_Data[0].data+Target_Data.data
		
		# Untill the encoder value is equal to the desire angle
		while (self.Encoder_Data[0].data!=Target_Angle):
			
			# Move up if it is down
			if self.Encoder_Data[0].data>Target_Angle:
				
				# Publish in /Arm_Motor/vel_cmd topic a msg equal to float -0.5
				self.Move_A_Publisher.publish(-self.Motor_Vel[0])
			
			# Move down if it up
			elif self.Encoder_Data[0].data<Target_Angle:
				
				# Publish in /Arm_Motor/vel_cmd topic a msg equal to float 0.5
				self.Move_A_Publisher.publish(self.Motor_Vel[0])
			
			# Store new encoder value	
			self.Encoder_Data[0]=rospy.wait_for_message('Encoder_Arm_Sensor', Int16)
		
		# When equal, dont move it more	
		self.Move_A_Publisher.publish(0.0)

# Main
def main():
    
		# Initializes and cleanup ROS node
    rospy.init_node('Target_Motor', anonymous=True)
    TM = Target_Motor()
    
    try:
    		# Listens for new msgs in the subscribed topics
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Target Motor"

if __name__ == '__main__':
    main()
