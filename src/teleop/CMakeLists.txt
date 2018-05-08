cmake_minimum_required(VERSION 2.8.3)
project(teleop)

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

add_executable(drive_base_node drive_base.cpp)

target_link_libraries(drive_base_node ${catkin_LIBRARIES})
