cmake_minimum_required(VERSION 2.8.3)
project(firulais)

## Find catkin macros and libraries
find_package(OpenCV REQUIRED )
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	image_transport
	pcl_conversions
	pcl_msgs
	geometry_msgs
#	cmake_modules REQUIRED
	tf2_ros
	tf
	pcl_ros
	# Eigen REQUIRED
	tf2
	cv_bridge)
	#gazebo_msgs)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	# INCLUDE_DIRS include
	LIBRARIES
    	pcl_ros_filters
    	pcl_ros_io
		pcl_ros_tf
	CATKIN_DEPENDS
    	dynamic_reconfigure
    	# message_filters
    	nodelet
    	pcl_conversions
    	pcl_msgs
    	rosbag
    	roscpp
    	sensor_msgs
    	std_msgs
		tf
	DEPENDS
	    Boost
	    Eigen
		PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
			${OpenCV_INCLUDE_DIRS}
			${catkin_INCLUDE_DIRS} 
			${Eigen_INCLUDE_DIRS} 
			${PCL_INCLUDE_DIRS} )
## Declare a cpp library
# add_library(AutoNOMOS_simulation
#   src/${PROJECT_NAME}/AutoNOMOS_simulation.cpp
# )
link_directories(
	${Boost_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
add_executable(velocity_node velocity_node.cpp)
add_executable(lane_controller_sim_firulais_node PIDcontroller_sim.cpp)
add_executable(TrabajaConlaImagen TrabajaConlaImagen.cpp )
add_executable(perception_node perception_node.cpp )

target_link_libraries(TrabajaConlaImagen       ${catkin_LIBRARIES}  ${OpenCV_LIBS} )
target_link_libraries(velocity_node ${catkin_LIBRARIES})
target_link_libraries(lane_controller_sim_firulais_node ${catkin_LIBRARIES})
target_link_libraries(perception_node ${catkin_LIBRARIES})
