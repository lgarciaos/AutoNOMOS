cmake_minimum_required(VERSION 2.8.3)
project(lane_states_original)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11) 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
	catkin REQUIRED 
	roscpp 
	std_msgs 
	cv_bridge 
	sensor_msgs)

find_package(OpenCV REQUIRED)

set(ROS_LINK_FLAGS "-lopencv_core -lopencv_highgui -lopencv_legacy -lopencv_imgproc -lopencv_features2d")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_states
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories (
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library (ackerman   
  tools/ackerman.cpp
  ../fu_line_detection/src/tools/NewtonPolynomial.cpp
)

add_executable (lane_states_node lane_states_node.cpp)

target_link_libraries (lane_states_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ackerman)
