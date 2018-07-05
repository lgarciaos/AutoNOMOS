// std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <iterator>

// ros
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ros messages
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>

void get_obstacles_poses(gazebo_msgs::ModelStates msg, std::vector<geometry_msgs::Pose>& obstacles_pos, std::vector<geometry_msgs::Twist>& obstacles_twist)
{
  std::vector<int> obstacles_index;

  // for (auto element : msg) {
  //   ROS_INFO_STREAM(element);
  // }
}
