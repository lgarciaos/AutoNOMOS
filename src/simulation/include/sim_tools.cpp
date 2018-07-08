// std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include <regex>

// ros
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ros messages
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>

#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
// #include <gazebo/common/Time.hh>

#define RADIUS 0.8

const int grid_init_x = -10;
const int grid_end_x = 0;
const int grid_init_y = -10;
const int grid_end_y = 10;

std::string autonomos_static_regex("AutoNOMOS_mini_static_[[:digit:]]+");
std::string lamp_post_regex("lamp_post_autonomos_[[:digit:]]+");
std::vector<geometry_msgs::Pose> autonomos_pose;
std::vector<geometry_msgs::Twist> autonomos_twist;
std::vector<geometry_msgs::Pose> lamp_pose;
std::vector<geometry_msgs::Twist> lamp_twist;

///////////////////////////////////////////////////////////////////////////////
// TODO:
//       det if point is in car ==> trasformation
//////////////////////////////////////////////////////////////////////////////


void get_obstacles_poses(gazebo_msgs::ModelStates msg)
{
  std::vector<int> obstacles_index;
  // ROS_INFO_STREAM("At: " << __PRETTY_FUNCTION__);
  int i = 0;
  autonomos_pose.clear();
  autonomos_twist.clear();
  lamp_pose.clear();
  lamp_twist.clear();
  for (auto element : msg.name) {
    // if (!element.compare(autonomos_static)) {
    if (std::regex_match (element, std::regex(autonomos_static_regex) ))
    {
      autonomos_pose.push_back(msg.pose[i]);
      autonomos_twist.push_back(msg.twist[i]);
    } else if(std::regex_match (element, std::regex(lamp_post_regex)) )
    {
      lamp_pose.push_back(msg.pose[i]);
      lamp_twist.push_back(msg.twist[i]);
    }
    i++;
    // ROS_INFO_STREAM(element);
  }
}

bool point_in_car(geometry_msgs::Point point, geometry_msgs::Pose center_car, bool print = false)
{
  double x_dist = 0.125;
  double y_dist = 0.2;

  bool x_match = false;
  bool y_match = false;

  ignition::math::Quaterniond orient (center_car.orientation.x,
                                      center_car.orientation.y,
                                      center_car.orientation.z,
                                      center_car.orientation.w);
  // std::cout << "Pose: " << orient.Yaw() << ", " <<
  //   orient.Pitch() << ", " <<
  //   orient.Roll() << ", " <<
    // center_car.orientation.w <<
     // '\n';
  // std::cout << "twist: " << center_car.angular.x << ", " << center_car.angular.y << ", " << center_car.angular.z << '\n';
  double x_point_tf = point.x - center_car.position.x;
  double y_point_tf = point.y - center_car.position.y;
  double angle = orient.Roll();

  x_point_tf = x_point_tf * cos(angle) - y_point_tf * sin(angle);
  y_point_tf = y_point_tf * cos(angle) + x_point_tf * sin(angle);

  if (-x_dist <= x_point_tf && x_point_tf <= x_dist)
  {
    x_match = true;
  }
  if (-y_dist <= y_point_tf && y_point_tf <= y_dist )
  {
    y_match = true;
  }

  if (print)
  {
    std::cout <<
      "( " << point.x << ", " << point.y << " )\t\t" <<
      "( " << center_car.position.x << ", " << center_car.position.y << " )\t\t" <<
      "( " << x_point_tf << ", " << y_point_tf << " )" << '\n';
  }

  return x_match & y_match;
}


std::vector<geometry_msgs::Point> remove_obst_points(std::vector<geometry_msgs::Point> points_in)
{
  std::vector<geometry_msgs::Point> points_out;
  std::vector<int> points_in_obstacles;
  bool obst_found = false;
  int car_num = 0;
  int point_num = 0;
  for(auto point : points_in)
  {
    car_num = 0;
    for(auto pose : autonomos_pose)
    {
      if (point_in_car(point, pose))
      {
        points_in_obstacles.push_back(point_num);
      }
      if( car_num == 67)
      {
        std::cout << "is 67 in car? " << point_in_car(point, pose, true) << std::endl;
      }
      car_num++;
    }
    point_num++;
  }

  std::unique(points_in_obstacles.begin(), points_in_obstacles.end());

  for ( auto p : points_in_obstacles ) {
    std::cout << "index: " << p <<
      "\t( " << points_in[p].x << ", " << points_in[p].y << " )" << std::endl;
  }

  int index_point;
  if(points_in_obstacles.size() > 0)
  {
    index_point = points_in_obstacles.front();
    points_in_obstacles.erase(points_in_obstacles.begin());
  }
  for (size_t i = 0; i < points_in.size(); i++) {
    std::cout << "i: " << i << "\tindex_point: " << index_point << '\n';
    if (i == index_point)
    {
      if(points_in_obstacles.size() > 0)
      {
        index_point = points_in_obstacles.front();
        points_in_obstacles.erase(points_in_obstacles.begin());
      } else {
        index_point = -1;
      }
    } else {
      points_out.push_back(points_in[i]);
    }
  }

  for ( auto p : points_out ) {
    std::cout << "( " << p.x << ", " << p.y << " )" << '\n';
  }

  return points_out;
}


std::vector<geometry_msgs::Point> generate_grid(double x_inc, double y_inc)
{
  std::vector<geometry_msgs::Point> v;
  geometry_msgs::Point aux_pt;
  for (double x = grid_init_x; x < grid_end_x; x+=x_inc) {
    for (double y = grid_init_y; y < grid_end_y; y+=y_inc) {
      aux_pt.x = x;
      aux_pt.y = y;
      aux_pt.z = 0;

      v.push_back(aux_pt);
    }
  }
  return v;
}

void paint_points(std::vector<geometry_msgs::Point> points)
{
  std::cout << "At paint_points" << '\n';
  ignition::msgs::Marker markerMsg;
  ignition::transport::Node node;
  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/Green");
  markerMsg.set_ns("default");
  // markerMsg.set_id(0);

  // std::cout << "Adding 100 points inside the square\n";
  // gazebo::common::Time::Sleep(4);
  markerMsg.set_id(0);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::POINTS);
  markerMsg.clear_point();
  for (auto element : points)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(element.x, element.y, element.z));
  }
  node.Request("/marker", markerMsg);
}

double distance(geometry_msgs::Point ini, geometry_msgs::Point p)
{
  return sqrt( (p.x - ini.x) * (p.x - ini.x) + (p.y - ini.y) * (p.y - ini.y) );
}

void h()
{

}

void cost()
{

}



std::vector<geometry_msgs::Point> get_adj_points(geometry_msgs::Point ini, std::vector<geometry_msgs::Point> points)
{
  std::vector<geometry_msgs::Point> out;
  for(auto pt : points)
  {
    if (distance(ini, pt) < RADIUS)
    {
      out.push_back(pt);
    }
  }
  // TODO: determine if the point can be reach by the car.
}

void a_star(std::vector<geometry_msgs::Point> points, int init_point)
{
  std::vector<geometry_msgs::Point> next_pts = get_adj_points(p);

}
