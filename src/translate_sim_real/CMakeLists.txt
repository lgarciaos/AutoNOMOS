cmake_minimum_required(VERSION 2.8.3)
project(translate_sim_real)

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

add_executable(translate_sim_real_node translate_sim_real.cpp)

target_link_libraries(translate_sim_real_node ${catkin_LIBRARIES})

