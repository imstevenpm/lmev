#!/usr/bin/env python

# Python libs
import numpy
import tf

# ROS libraries
import roslib
import rospy

from std_msgs.msg import Int16
from std_msgs.msg import Float32

class Main_Controller:
	
	def __init__(self):
		
		# Another way to listen a msg
		
		#self.Ambient_Subscriber = rospy.Subscriber('Color_Sensor/Ambient', Int16, self.Ambient_Processor)
		# Color Mode
		#self.Color_Subscriber = rospy.Subscriber('Color_Sensor/Color', Illuminance, self.Color_Processor)
		# Light Mode
		#self.Light_Subscriber = rospy.Subscriber('Color_Sensor/Light', Illuminance, self.Ambient_Processor)
		#self.Gyroscope_Subscriber = rospy.Subscriber('Gyroscope_Sensor', Int16, self.Gyroscope_Processor)
		#self.Touch_Subscriber = rospy.Subscriber('Touch_Sensor', Int16, self.Touch_Processor)
		#self.Ultrasonic_Subscriber = rospy.Subscriber('Ultrasonic_Sensor', Float32, self.Ultrasonic_Processor)
		#self.RW_Encoder_Subscriber = rospy.Subscriber('Encoder_Right_Wheel_Sensor', Int16, self.RW_Processor)
		#self.LW_Encoder_Subscriber = rospy.Subscriber('Encoder_Left_Wheel_Sensor', Int16, self.LW_Processor)
		#self.A_Encoder_Subscriber = rospy.Subscriber('Encoder_Arm_Wheel_Sensor', Int16, self.A_Processor)
		
		# Variables
		self.Sensor_Data = [0,0,0,0,0,0,0]
		self.Rate = rospy.Rate(2)
	
	#def Ambient_Processor(self,Data):
		#self.Sensor_Data[0]=Data.data
		# Do stuff
	#def Gyroscope_Processor(self,Data):
		#self.Sensor_Data[1]=Data.data
		# Do stuff
	#def Touch_Processor(self,Data):
		#self.Sensor_Data[2]=Data.data
		# Do stuff
	#def Ultrasonic_Processor(self,Data):
		#self.Sensor_Data[3]=Data.data
		# Do stuff
	#def RW_Processor(self,Data):
		#self.Sensor_Data[4]=Data.data
		# Do stuff
	#def LW_Processor(self,Data):
		#self.Sensor_Data[5]=Data.data
		# Do stuff
	#def A_Processor(self,Data):
		# Do stuff
		#self.Sensor_Data[6]=Data.data
	
	def Show_Measures(self):
		
		while not rospy.is_shutdown():
			#self.Sensor_Data[0]=rospy.wait_for_message('Color_Sensor/Color', Int16)
			self.Sensor_Data[0]=rospy.wait_for_message('Color_Sensor/Ambient', Int16)
			#self.Sensor_Data[0]=rospy.wait_for_message('Color_Sensor/Light', Int16)
			self.Sensor_Data[1]=rospy.wait_for_message('Gyroscope_Sensor', Int16)
			self.Sensor_Data[2]=rospy.wait_for_message('Touch_Sensor', Int16)
			self.Sensor_Data[3]=rospy.wait_for_message('Ultrasonic_Sensor', Float32)
			self.Sensor_Data[4]=rospy.wait_for_message('Encoder_Right_Wheel_Sensor', Int16)
			self.Sensor_Data[5]=rospy.wait_for_message('Encoder_Left_Wheel_Sensor', Int16)
			self.Sensor_Data[6]=rospy.wait_for_message('Encoder_Arm_Sensor', Int16)
			
			rospy.loginfo("\n* * * * * * * * * * \nColor Sensor: %s ptc \nGyroscope Sensor: %s degrees \nTouch Sensor: %s state\nUltrasonic Sensor: %s cm \nEncoder Rigth Wheel Sensor: %s degrees \nEncoder Left Wheel Sensor: %s degrees \nEncoder Arm Sensor: %s degrees \n",self.Sensor_Data[0].data,self.Sensor_Data[1].data,self.Sensor_Data[2].data,round(self.Sensor_Data[3].data,1),self.Sensor_Data[4].data,self.Sensor_Data[5].data,self.Sensor_Data[6].data)
			self.Rate.sleep()
        
# Main
def main():

	# Initializes and cleanup ROS node
	rospy.init_node('Main_Controller', anonymous=True)
	MC = Main_Controller()
	MC.Show_Measures()
	
	try:
		# Listens for new msgs in the subscribed topics
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down Main Controller"

if __name__ == '__main__':
	main()
