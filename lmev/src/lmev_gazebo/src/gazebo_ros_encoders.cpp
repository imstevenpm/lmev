/* Modifications and adaptations for use 
*		in the Lego Mindstorms EV3 simulation: 
 *	\author Steven Palma Morera
 *	\date 22th of Nov 2017
 */

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo_msgs/LinkStates.h>
#include <sstream>
#include <gazebo/gazebo.hh>
#include <iostream>
#include <gazebo/physics/World.hh>
#include <std_msgs/Int16.h>
#include <chrono>
#include <random>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <stdlib.h>
#include <tf/transform_datatypes.h>
#include <string>

#include <vector>

namespace gazebo
{
	
	class EncoderPlugin : public ModelPlugin
	{ 
		public: EncoderPlugin() : ModelPlugin(){}
		
		public: void Callback22(const gazebo_msgs::LinkStates::ConstPtr& msg)
		{
		
			// Variables for direction and orientation
			double rollRW, pitchRW, yawRW;
			double rollLW, pitchLW, yawLW;
			double rollA, pitchA, yawA;
    	int RW, LW, Arm;
    	
    	// Finding index for the links which needs the encoders
    	if (this->find==0){
    		for (int i = 0; i < (msg->name.size()); i++) {
    			if (msg->name[i] == "lmev::Right_Wheel") {
    				this->RWi=i;
    				}
    			else if (msg->name[i] == "lmev::Left_Wheel") {
    				this->LWi=i;
    				}
    			else if (msg->name[i] == "lmev::Arm") {
    				this->Ai=i;
    				}
    			}
    		this->find=1;
    		}
    	
    	// Create quaternions
    	tf::Quaternion qRW(msg->pose[this->RWi].orientation.x,msg->pose[this->RWi].orientation.y,msg->pose[this->RWi].orientation.z,msg->pose[this->RWi].orientation.w);
    	
    	tf::Quaternion qLW(msg->pose[this->LWi].orientation.x,msg->pose[this->LWi].orientation.y,msg->pose[this->LWi].orientation.z,msg->pose[this->LWi].orientation.w);
    	
    	tf::Quaternion qA(msg->pose[this->Ai].orientation.x,msg->pose[this->Ai].orientation.y,msg->pose[this->Ai].orientation.z,msg->pose[this->Ai].orientation.w);
    	
    	// Convert quaternions to matrix
    	tf::Matrix3x3 mRW(qRW);
    	
    	tf::Matrix3x3 mLW(qLW);
    	
    	tf::Matrix3x3 mA(qA);
		
			// Convert quaternions to Euler angles
			mRW.getRPY(rollRW,pitchRW,yawRW);
			
			mLW.getRPY(rollLW,pitchLW,yawLW);
			
			mA.getRPY(rollA,pitchA,yawA);
			
			// Extracts only the x angle and convert it in degrees
			rollLW=rollLW*180/3.141593;
			rollRW=rollRW*180/3.141593;
			rollA=rollA*180/3.141593;
		
			// Algorithm to consider rotation direction
			if ((this->Flag0>178) and (rollLW<-178)){
				this->Cont0=this->Cont0+1;
			}
			else if ((this->Flag0<-178) and (rollLW>178)){
				this->Cont0=this->Cont0-1;
			}
		
			this->Flag0=rollLW;
		
			rollLW=rollLW+(this->Cont0)*360;
			
			// Algorithm to consider accumulative angle 
			if ((this->Flag1>719) and (rollLW<361)){
				this->Cont1=this->Cont1+1;
				}
			else if ((this->Flag1<-719) and (rollLW>-361)){
				this->Cont1=this->Cont1-1;
				}
			
			this->Flag1= rollLW;
		
			// Create Gaussian noise
			unsigned seedLW = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine generatorLW (seedLW);
  		// Mean: 0.0 Standard deviation: 0.5
  		std::normal_distribution<double> distributionLW (this->gaussian_noise_mean,this->gaussian_noise_std);
  		
  		// Add accumulative angle, gaussian noise, round and convert to int
  		LW = int(round(rollLW+distributionLW(generatorLW))+(360*this->Cont1));
    
    	// Save encoder data in msg
    	this->msgLW.data=LW;
    	
    	// Same procedure for right encoder
    	
    	if ((this->Flag2>178) and (rollRW<-178)){
				this->Cont2=this->Cont2+1;
			}
			else if ((this->Flag2<-178) and (rollRW>178)){
				this->Cont2=this->Cont2-1;
			}
		
			this->Flag2=rollRW;
		
			rollRW=rollRW+(this->Cont2)*360;
		
			if ((this->Flag3>719) and (rollRW<361)){
				this->Cont3=this->Cont3+1;
				}
			else if ((this->Flag3<-719) and (rollRW>-361)){
				this->Cont3=this->Cont3-1;
				}
			
			this->Flag3= rollRW;
		
			unsigned seedRW = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine generatorRW (seedRW);
  		std::normal_distribution<double> distributionRW (this->gaussian_noise_mean,this->gaussian_noise_std);
  	
  		RW = int(round(rollRW+distributionRW(generatorRW))+(360*this->Cont3));
			
			this->msgRW.data=-RW;
			
			unsigned seedA = std::chrono::system_clock::now().time_since_epoch().count();
  		std::default_random_engine generatorA (seedA);
  		std::normal_distribution<double> distributionA (this->gaussian_noise_mean,this->gaussian_noise_std);
			
			// Add noise, round and convert to int
  		Arm= int(round(rollA+distributionA(generatorA)));
  		
  		this->msgA.data=Arm;
  		
  		// Publish msg in topic
  		this->pubRW.publish(this->msgRW);
  		this->pubLW.publish(this->msgLW);
  		this->pubA.publish(this->msgA);
  		
  		ros::spinOnce();
		}
		
		private: void QueueThread()
		{
  		static const double timeout = 0.01;
  		while (this->rosNode->ok())
  		{
    		this->rosQueue.callAvailable(ros::WallDuration(timeout));
  		}
		}
		
		
		public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// Stores model pointer
			this->model = _model;
      
      // Reads noise custom values
      if (_sdf->HasElement("gaussian_noise_mean_e"))
  		{
    		this->gaussian_noise_mean =  _sdf->Get<double>("gaussian_noise_mean_e");
    		//ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
  		}
  		else
  		{
    		this->gaussian_noise_mean = 0.0;
    		//ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
  		}
  
  		if (_sdf->HasElement("gaussian_noise_std_e"))
  		{
    		this->gaussian_noise_std =  _sdf->Get<double>("gaussian_noise_std_e");
    		//ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
  		}
  		else
  		{
    		this->gaussian_noise_std = 0.0;
    		//ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
  		}
       
      // Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
  			int argc = 0;
  			char **argv = NULL;
  			ros::init(argc, argv, "gazebo_client",
      	ros::init_options::NoSigintHandler);
      }
      
      // Create our ROS node.
			this->rosNode.reset(new ros::NodeHandle());
			
			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so = ros::SubscribeOptions::create<gazebo_msgs::LinkStates>("/gazebo/link_states",1,boost::bind(&EncoderPlugin::Callback22, this, _1),ros::VoidPtr(), &this->rosQueue);
      
      // Subcribe node
      this->sub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread = std::thread(std::bind(&EncoderPlugin::QueueThread, this));
			
			// Create topic to publish
			this->pubRW = this->rosNode->advertise<std_msgs::Int16>("Encoder_Right_Wheel_Sensor", 1);
			
			this->pubLW = this->rosNode->advertise<std_msgs::Int16>("Encoder_Left_Wheel_Sensor", 1);
			
			this->pubA = this->rosNode->advertise<std_msgs::Int16>("Encoder_Arm_Sensor", 1);
			
			// Initializing values
			this->Flag0=0;
			this->Flag1=0;
			this->Flag2=0;
			this->Flag3=0;
			
			this->Cont0=0;
			this->Cont1=0;
			this->Cont2=0;
			this->Cont3=0;
			
			this->RWi=0;
			this->LWi=0;
			this->Ai=0;
			
			this->find=0;
		}
		
		// This could be written in a header file
		
		private: physics::ModelPtr model;
		private: std::unique_ptr<ros::NodeHandle> rosNode;
		private: ros::CallbackQueue rosQueue;
		private: std::thread rosQueueThread;
		private: ros::Subscriber sub;
		private: ros::Publisher pubRW;
		private: ros::Publisher pubLW;
		private: ros::Publisher pubA;
		private: std_msgs::Int16 msgRW;
		private: std_msgs::Int16 msgLW;
		private: std_msgs::Int16 msgA;
		private: double gaussian_noise_mean;
		private: double gaussian_noise_std;
		private: float Flag0;
		private: float Flag1;
		private: float Flag2;
		private: float Flag3;
		private: int Cont0;
		private: int Cont1;
		private: int Cont2;
		private: int Cont3;
		
		private: int RWi;
		private: int LWi;
		private: int Ai;
		
		private: int find;
	};
	
	// Register plugin
	GZ_REGISTER_MODEL_PLUGIN(EncoderPlugin)
}
