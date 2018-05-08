cmake_minimum_required(VERSION 2.8.3)
project(lane_controller_sim)


find_package(gazebo REQUIRED )
find_package(catkin REQUIRED roscpp std_msgs gazebo_msgs )



catkin_package(
#  INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS}
#  LIBRARIES lane_states
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(include 
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(lane_controller_sim_node PIDcontroller_lane_sim.cpp)

target_link_libraries(lane_controller_sim_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
