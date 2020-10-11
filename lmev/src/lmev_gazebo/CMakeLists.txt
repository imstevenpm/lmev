cmake_minimum_required(VERSION 2.8.3)
project(lmev_gazebo)

# Required packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
)

find_package (gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp
)

# Build
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# The sdformat version should match with the carpet in home/usr/include
set(GAZEBOINCLUDEDIRS /usr/include;/usr/include/gazebo-7;/usr/include/sdformat-4.4) 
set(GAZEBOLIBRARYDIRS /usr/lib/x8664-linux-gnu;/usr/lib/x8664-linux-gnu/gazebo-7/plugins)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
)

# Add made libraries
# add_library(name_of_the_library path_to_cpp/cppfile.cpp
# target_link_libraries(name_of_the_library depend_libraries)

# Color Sensor> Color
add_library(Sgazebo_ros_color src/gazebo_ros_color.cpp)
target_link_libraries(Sgazebo_ros_color ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} CameraPlugin )

# Color Sensor> Ambient
add_library(Sgazebo_ros_ambient src/gazebo_ros_ambient.cpp)
target_link_libraries(Sgazebo_ros_ambient ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} CameraPlugin )

# Color Sensor> Light
add_library(Sgazebo_ros_light src/gazebo_ros_light.cpp)
target_link_libraries(Sgazebo_ros_light ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} CameraPlugin )

# Gyroscope Sensor
add_library(Sgazebo_ros_imu_sensor src/gazebo_ros_imu_sensor.cpp)
target_link_libraries(Sgazebo_ros_imu_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Touch Sensor
add_library(Sgazebo_ros_bumper src/gazebo_ros_bumper.cpp)
add_dependencies(Sgazebo_ros_bumper ${catkin_EXPORTED_TARGETS})
target_link_libraries(Sgazebo_ros_bumper ${Boost_LIBRARIES} ContactPlugin ${catkin_LIBRARIES})

# Ultrasonic Sensor
add_library(Sgazebo_ros_range src/gazebo_ros_range.cpp)
target_link_libraries(Sgazebo_ros_range ${catkin_LIBRARIES} ${Boost_LIBRARIES} RayPlugin)

# Encoders Sensors
add_library(Sgazebo_ros_encoders SHARED src/gazebo_ros_encoders.cpp)
target_link_libraries(Sgazebo_ros_encoders ${GAZEBO_libraries})


# Differential Drive for tank movement
add_library(Sgazebo_ros_diff_drive src/gazebo_ros_diff_drive.cpp)
target_link_libraries(Sgazebo_ros_diff_drive ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

# Right Motor for individual movement
add_library(Sgazebo_ros_right_wheel_motor SHARED src/gazebo_ros_right_wheel_motor.cpp)
target_link_libraries(Sgazebo_ros_right_wheel_motor ${GAZEBO_libraries})

# Left Motor for individual movement
add_library(Sgazebo_ros_left_wheel_motor SHARED src/gazebo_ros_left_wheel_motor.cpp)
target_link_libraries(Sgazebo_ros_left_wheel_motor ${GAZEBO_libraries})

# Arm motor for individual movement
add_library(Sgazebo_ros_arm_motor SHARED src/gazebo_ros_arm_motor.cpp)
target_link_libraries(Sgazebo_ros_arm_motor ${GAZEBO_libraries})


# Welcome message
add_library(Swelcome SHARED src/welcome.cpp)
target_link_libraries(Swelcome ${GAZEBO_libraries})
