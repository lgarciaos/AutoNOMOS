cmake_minimum_required(VERSION 2.8.3)
project(lane_planning)


find_package(catkin REQUIRED roscpp std_msgs )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_states
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lane_planning_node lane_planning.cpp)
add_executable(lane_planning_prob_node lane_planning_prob.cpp)

target_link_libraries(lane_planning_node ${catkin_LIBRARIES})
target_link_libraries(lane_planning_prob_node ${catkin_LIBRARIES})
