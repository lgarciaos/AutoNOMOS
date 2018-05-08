cmake_minimum_required(VERSION 2.8.3)
project(lane_controller)

find_package(catkin REQUIRED roscpp std_msgs )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_states
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include 
	${catkin_INCLUDE_DIRS}
)

add_executable(lane_controller_node PIDcontroller.cpp)

target_link_libraries(lane_controller_node ${catkin_LIBRARIES})
