cmake_minimum_required(VERSION 2.8.3)
project(autonomos_description)
find_package(catkin REQUIRED)
catkin_package()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

#find_package(catkin REQUIRED COMPONENTS
#  geometry_msgs
#  roscpp
#  rviz
#  tf
#  urdf
#  xacro
#)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY urdf/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
	FILES_MATCHING PATTERN "*.dae" PATTERN "*.stl" PATTERN "*.xacro" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)
