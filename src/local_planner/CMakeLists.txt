cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

add_compile_options(-std=c++11) 

# find_package(gazebo REQUIRED )
find_package(catkin REQUIRED 
	roscpp 
	std_msgs
	cv_bridge
	sensor_msgs
) # gazebo_msgs )

find_package(OpenCV REQUIRED)

set(ROS_LINK_FLAGS "-lopencv_core -lopencv_highgui -lopencv_legacy -lopencv_imgproc -lopencv_features2d")

catkin_package(
#  INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS}
#  LIBRARIES lane_states
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include 
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
#	${GAZEBO_INCLUDE_DIRS}
)

#add_library(local_planner   
#  ../fu_line_detection/src/tools/NewtonPolynomial.cpp
#)

# link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(local_planner_node local_planner.cpp)

target_link_libraries(local_planner_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # local_planner) #${GAZEBO_LIBRARIES} local_planner )
