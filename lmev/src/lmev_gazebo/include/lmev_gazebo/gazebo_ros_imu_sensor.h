/* Copyright [2015] [Alessandro Settimi]
 * 
 * email: ale.settimi@gmail.com
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/
 
/* Modifications and adaptations for use 
*		in the Lego Mindstorms EV3 simulation: 
 *	\author Steven Palma Morera
 *	\date 22th of Nov 2017
 */ 
 
#ifndef GAZEBO_ROS_IMU_SENSOR_H
#define GAZEBO_ROS_IMU_SENSOR_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Pose.hh>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>

#include <std_msgs/Int16.h>

#include <iostream>
#include <chrono>
#include <random>
#include <math.h>

#include <tf/transform_broadcaster.h>
#include <angles/angles.h>

namespace gazebo
{
  namespace sensors
  {
    class ImuSensor;
  }
  /**
  @anchor GazeboRosImuSensor
  \ref GazeboRosImuSensor is a plugin to simulate an Inertial Motion Unit sensor, the main differences from \ref GazeboRosIMU are:
  - inheritance from SensorPlugin instead of ModelPlugin,
  - measurements are given by gazebo ImuSensor instead of being computed by the ros plugin,
  - gravity is included in inertial measurements.
  */
  /** @brief Gazebo Ros imu sensor plugin. */
  class GazeboRosImuSensor : public SensorPlugin
  {
  public:
    /// \brief Constructor.
    GazeboRosImuSensor();
    /// \brief Destructor.
    virtual ~GazeboRosImuSensor();
    /// \brief Load the sensor.
    /// \param sensor_ pointer to the sensor.
    /// \param sdf_ pointer to the sdf config file.
    virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

  protected:
    /// \brief Update the sensor.
    virtual void UpdateChild(const gazebo::common::UpdateInfo &/*_info*/);

  private:
    /// \brief Load the parameters from the sdf file.
    bool LoadParameters();
    /// \brief Gaussian noise generator.
    /// \param mu offset value.
    /// \param sigma scaling value.
    double GuassianKernel(double mu, double sigma);
    
    /// \brief Ros NodeHandle pointer.
    ros::NodeHandle* node;
    /// \brief Ros Publisher for imu data.
    //ros::Publisher imu_data_publisher;
    
    // MODIFICATION > Creates a new publisher with different msg type
    // ROS publisher for the gyroscope sensor data
    ros::Publisher sensor_data;
    // Create a Int16 msg
    std_msgs::Int16 msg;
    
    /// \brief Ros IMU message.
    sensor_msgs::Imu imu_msg;

    /// \brief last time on which the data was published.
    common::Time last_time;
    /// \brief Pointer to the update event connection.
    gazebo::event::ConnectionPtr connection;
    /// \brief Pointer to the sensor.
    sensors::ImuSensor* sensor;
    /// \brief Pointer to the sdf config file.
    sdf::ElementPtr sdf;
    /// \brief Orientation data from the sensor.
    gazebo::math::Quaternion orientation;
    /// \brief Linear acceleration data from the sensor.
    math::Vector3 accelerometer_data;
    /// \brief Angular velocity data from the sensor.
    math::Vector3 gyroscope_data;
    
    /// \brief Seed for the Gaussian noise generator.
    unsigned int seed;

    //loaded parameters
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
    std::string robot_namespace;
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
    //std::string topic_name;
    /// \brief Name of the link of the IMU.
    std::string body_name;
    /// \brief Sensor update rate.
    double update_rate;
    
    // MODIFICATION > Declaring noise variables for custom values and variables needed for consider accumulative angle and orientation
    /// \brief Gaussian noise.
    double gaussian_noise_mean;
    double gaussian_noise_std;
    /// \brief Offset parameter, position part is unused.
    math::Pose offset;
    
    float Flag0;
    float Flag1;
    int Cont0;
    int Cont1;
    
  };
}

#endif //GAZEBO_ROS_IMU_SENSOR_H
