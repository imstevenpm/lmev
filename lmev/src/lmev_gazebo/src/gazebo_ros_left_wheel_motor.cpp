/* Modifications and adaptations for use 
*		in the Lego Mindstorms EV3 simulation: 
 *	\author Steven Palma Morera
 *	\date 22th of Nov 2017
 */

#ifndef _MOTOR_PLUGIN_HH_
#define _MOTOR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"

namespace gazebo
{

  class LmotorPlugin : public ModelPlugin
  {

    public: LmotorPlugin() {}
    
    public: physics::JointControllerPtr jointController;
		
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

      this->model = _model;
      
      // Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
  			int argc = 0;
  			char **argv = NULL;
  			ros::init(argc, argv, "gazebo_client",
      	ros::init_options::NoSigintHandler);
      }
      
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
			
			// Create topic to subscribe
			ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64>("/Left_Wheel_Motor/vel_cmd",1,boost::bind(&LmotorPlugin::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);
			
			// Declare the node as a subscriber
			this->rosSub = this->rosNode->subscribe(so);

			this->rosQueueThread = std::thread(std::bind(&LmotorPlugin::QueueThread, this));
    }

    public: void SetVelocity(const double &_vel)
    {
    	
      // Use no more than 0.45 Nm to achive the target vel
      this->model->GetJoint("Left_Wheel_Joint")->SetParam("fmax", 0, 0.45);
      
      // Maximum velocity (rad/s) per motor
      double rads;
      rads=_vel;
      
      if (rads<-18.0){
      	// 175 rpm
      	rads=-18.0;
      }
      else if (rads>18.0){
      	// 175 rpm
      	rads=18.0;
      }
      	
      // Set the joint's target velocity
      this->model->GetJoint("Left_Wheel_Joint")->SetParam("vel", 0, rads);
    }
    
    // Callback function for each msg
		public: void OnRosMsg(const std_msgs::Float64ConstPtr &_msg)
		{
  		this->SetVelocity(_msg->data);
		}

		private: void QueueThread()
		{
  		static const double timeout = 0.01;
  		while (this->rosNode->ok())
  		{
    		this->rosQueue.callAvailable(ros::WallDuration(timeout));
  		}
		}
		
		// This could be written in a header file and use it for the three plugins
		
    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;
    
    /// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;
		
		public: event::ConnectionPtr updateConnection;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(LmotorPlugin)
}
#endif
