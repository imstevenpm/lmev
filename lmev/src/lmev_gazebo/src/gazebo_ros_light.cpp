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
    _sensorPublisher = _nh.advertise<std_msgs::Int16>("Light", 1);
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
    
    // Read noise custom values
    
    if (_sdf->HasElement("gaussian_noise_mean_c"))
  	{
    	gaussian_noise_mean =  _sdf->Get<double>("gaussian_noise_mean_c");
    	//ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
  	}
  	else
  	{
    	gaussian_noise_mean = 0.0;
    	//ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
  	}
  
  	if (_sdf->HasElement("gaussian_noise_std_c"))
  	{
    	gaussian_noise_std =  _sdf->Get<double>("gaussian_noise_std_c");
    	//ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
  	}
  	else
  	{
    	gaussian_noise_std = 0.0;
    	//ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
  	}
    
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

    if (!this->parentSensor->IsActive())
    {
      if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
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


					// Finding light intensity from pixel values
          
          // Create Int16 msg
          std_msgs::Int16 msg;
          
          //// RGB Variables
          int r = 0;
    			int g = 0;
    			int b = 0;

					// Store RGB individual values of the image
    			for(int i=0;i<(_fov*_fov*3);i=i+_fov*3){
        		for(int j=0;j<_fov*3;j=j+3){
            	r += _image[i+j];
            	g += _image[i+j+1];
            	b += _image[i+j+2];
        		}
    			}
         	
         	// Create gaussian noise
         	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  				std::default_random_engine generator (seed);
  				// Mean: 0.0 Standard deviation: 0.5
  				std::normal_distribution<double> distribution (gaussian_noise_mean,gaussian_noise_std);
         	
         	// Relative luminance > Following the luminosity function / Photometric/digital ITU BT.709
         	// Scale to a 0-100 scale
          float illuminance;
          illuminance = 100*(r*0.2126+g*0.7152+b*0.0722)/(255*(_fov*_fov));
          
          // Add noise, round and convert to int
          illuminance = int(round(illuminance+distribution(generator)));
          
          // Saturates noisy data to limit
          if (illuminance>100){
						illuminance=100;
						}
					else if (illuminance<0){
						illuminance=0;
						}
          
          // Save luminance percentaje data in msg
          msg.data=illuminance;
          
          // Publish sensor data
          _sensorPublisher.publish(msg);

          seq++;
        }
      }
    }
  }
}
