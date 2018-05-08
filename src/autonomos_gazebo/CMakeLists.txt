cmake_minimum_required(VERSION 2.8.3)
project(autonomos_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  autonomos_description
  gazebo_plugins
  gazebo_ros
#  gazebo_ros_control
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autonomos_gazebo
#  CATKIN_DEPENDS autonomos_description gazebo_plugins gazebo_ros gazebo_ros_control
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
