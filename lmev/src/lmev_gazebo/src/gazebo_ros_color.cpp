/* Modifications and adaptations for use 
*		in the Lego Mindstorms EV3 simulation: 
 *	\author Steven Palma Morera
 *	\date 22th of Nov 2017
 */ 
 
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "lmev_gazebo/gazebo_ros_color.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/Illuminance.h>

#include <math.h>
#include <stdio.h>

#include <std_msgs/Int16.h>
#include <iostream>
#include <chrono>
#include <random>
#include <math.h>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosColor)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosColor::GazeboRosColor():
  // Declaring fov and range
  _nh("Color_Sensor"),
  _fov(48),
  _range(10)
  {
  	// Create topic to publish
    _sensorPublisher = _nh.advertise<std_msgs::Int16>("Color", 1);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosColor::~GazeboRosColor()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void GazeboRosColor::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(_parent, _sdf);
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;
		
    GazeboRosCameraUtils::Load(_parent, _sdf);

    
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosColor::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
  	if (_sensorPublisher.getNumSubscribers() <= 0)
    return;
  	
    static int seq=0;

    this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
		
		// First try
    if (!this->parentSensor->IsActive())
    {
      if ((*this->image_connect_count_) > 0)
        this->parentSensor->SetActive(true);
    }
    else
    {
      if ((*this->image_connect_count_) > 0)
      {
        common::Time cur_time = this->world_->GetSimTime();
        if (cur_time - this->last_update_time_ >= this->update_period_)
        {
          this->PutCameraData(_image);
          this->PublishCameraInfo();
          this->last_update_time_ = cur_time;
					
					// Find the color
          
          // Create Int16 msg
          std_msgs::Int16 msg;
          
          // RGB Variables
          int r = 0;
    			int g = 0;
    			int b = 0;

					// Store the RGB individual values
    			for(int i=0;i<(_fov*_fov*3);i=i+_fov*3){
        		for(int j=0;j<_fov*3;j=j+3){
            	r += _image[i+j];
            	g += _image[i+j+1];
            	b += _image[i+j+2];
        		}
    			}
    			
    			// RGB values for the color cube point of the image
    			// Mean of each rgb value
    			r = r/(_fov*_fov);
    			g = g/(_fov*_fov);
    			b = b/(_fov*_fov);
         	
         	// Declaring standard HTML color values points in color cube
         	int Black [3] = {0,0,0};
         	int Blue [3] = {0,0,255};
         	int Green [3] = {0,128,0};
         	int Yellow [3] = {255,255,0};
         	int Red [3] = {255,0,0};
         	int White [3] = {255,255,255};
         	int Brown [3] = {165,42,42};
         	
         	int Dist [7]= {0,0,0,0,0,0,0};
         	
         	// Calculate distance from standard points to image color point
         	Dist [0] = pow(Black[0]-r,2)+pow(Black[1]-g,2)+pow(Black[2]-b,2);
         	Dist [1] = pow(Blue[0]-r,2)+pow(Blue[1]-g,2)+pow(Blue[2]-b,2);
         	Dist [2] = pow(Green[0]-r,2)+pow(Green[1]-g,2)+pow(Green[2]-b,2);
         	Dist [3] = pow(Yellow[0]-r,2)+pow(Yellow[1]-g,2)+pow(Yellow[2]-b,2);
         	Dist [4] = pow(Red[0]-r,2)+pow(Red[1]-g,2)+pow(Red[2]-b,2);
         	Dist [5] = pow(White[0]-r,2)+pow(White[1]-g,2)+pow(White[2]-b,2);
         	Dist [6] = pow(Brown[0]-r,2)+pow(Brown[1]-g,2)+pow(Brown[2]-b,2);
         	
         	int index = 0;
         	
         	// Find the nearest
    			for(int i = 1; i < 7; i++){
        		if(Dist[i] < Dist[index])
            	index = i;              
          }
          
          
          // Umbral point
          // If distance is higher than 16256.25 the image color point isn't close to a standard color so the sensor color data should be 'no color'
          // Change this value to adjust the color sensibility
          if(Dist[index]>16256.25){
          	index=0;
          }
          else{
          	index++;
          }
					
					// Convert to int and save color data in msg
          msg.data=index;
					
					// Publish msg in topic
          _sensorPublisher.publish(msg);

          seq++;
        }
      }
    }
  }
}
