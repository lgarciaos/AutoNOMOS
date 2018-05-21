cmake_minimum_required(VERSION 2.8.3)
project(autonomos_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  autonomos_description
#  gazebo_plugins
#  gazebo_ros
#  gazebo_ros_control
)

catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
