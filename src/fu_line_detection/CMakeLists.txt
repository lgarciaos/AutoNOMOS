cmake_minimum_required(VERSION 2.8.3)
project(line_detection_fu_mod)

add_compile_options(-std=c++11) 

find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED) 
find_package(
  catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  sensor_msgs 
  cv_bridge 
#  pcl_ros 
  camera_info_manager 
  image_transport 
  message_generation
  dynamic_reconfigure
)

add_message_files(
    FILES
    pts_array.msg
#   Message2.msg
)

generate_dynamic_reconfigure_options(
  cfg/LaneDetection.cfg
  #...
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME} 
   CATKIN_DEPENDS 
    std_msgs 
    sensor_msgs 
    message_runtime
    dynamic_reconfigure
   DEPENDS 
    OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)


## Declare a cpp library
add_library(LaneDetectorFu   
  src/tools/NewtonPolynomial.cpp
  src/tools/LanePolynomial.cpp
  src/tools/IPMapper.cpp
   #src/LaneDetector.cpp
   #src/StrongClassifier.cpp
   #src/WeakClassifier.cpp
   #src/Feature.cpp
   #src/ContourModel.cpp
   #src/LaneModel.cpp
   #src/PointSet.cpp
   #src/Circle.cpp
   #src/IPMapper.cpp 
)

## Declare a cpp executable
add_executable(line_detection_fu_node src/laneDetection.cpp)

target_link_libraries(line_detection_fu_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  LaneDetectorFu
) 

install(TARGETS line_detection_fu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# make sure configure headers are built before any node using them
add_dependencies(line_detection_fu_node ${PROJECT_NAME}_gencfg)
