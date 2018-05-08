cmake_minimum_required(VERSION 2.8.3)
project(imagen_ackermann)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs cv_bridge sensor_msgs)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(ROS_LINK_FLAGS "-lopencv_core -lopencv_highgui -lopencv_legacy -lopencv_imgproc -lopencv_features2d")


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ekbot_ctrl
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


add_executable(imagenAckerDisplay DisplayImage.cpp)
add_executable(lane_controller_sim_imagenack_node PIDcontroller_sim.cpp)

target_link_libraries(imagenAckerDisplay ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(lane_controller_sim_imagenack_node ${catkin_LIBRARIES})

