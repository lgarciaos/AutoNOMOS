// std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iterator>
#include <regex>
#include <cassert>

// ros
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ros messages
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int64MultiArray.h>

// gazebo
#include <gazebo_msgs/ModelStates.h>


#define RECTANGLE 0
#define CIRCLE 1

gazebo_msgs::ModelStates model_states;
std::string points_creation;
geometry_msgs::Point initial_pt, end_pt;
int path_counter;
std::string autonomos_static_regex("AutoNOMOS_mini_static[_]*\\d*");
std::string lamp_post_regex("lamp_post_autonomos[_]*\\d*");
geometry_msgs::PoseArray  msg_obstacles_poses;
std_msgs::Int64MultiArray msg_obstacles_types;

void get_model_states(const gazebo_msgs::ModelStates& msg)
{

    model_states = msg;
}

void get_obstacles_poses()
{
  std::vector<int> obstacles_index;
  int i = 0;
  msg_obstacles_poses.poses.clear();
  msg_obstacles_types.data.clear();
  for (auto element : model_states.name)
  {
    if (std::regex_match (element, std::regex(autonomos_static_regex) ))
    {
      msg_obstacles_poses.poses.push_back(model_states.pose[i]);
      msg_obstacles_types.data.push_back(RECTANGLE);
      // autonomos_twist.push_back(msg.twist[i]);
    } else if(std::regex_match (element, std::regex(lamp_post_regex)) )
    {
      msg_obstacles_poses.poses.push_back(model_states.pose[i]);
      msg_obstacles_types.data.push_back(CIRCLE);
      // lamp_twist.push_back(msg.twist[i]);
    }
    i++;
  }
}

int main(int argc, char **argv)
{
    int rate_hz = 10;
    ros::init(argc, argv, "get_obstacles_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Rate loop_rate(rate_hz);
    path_counter = 0;

    // nh_priv.param<std::string>("points_creation", points_creation, "GRID");
    // nh_priv.param<double>("initial_point_x", initial_pt.x, -10);
    ros::Subscriber sub_model_states  = nh.subscribe("/gazebo/model_states", 1, &get_model_states);

    ros::Publisher pub_obs_poses = nh.advertise<geometry_msgs::PoseArray>(
      "/obstacles/poses", 1);
    ros::Publisher pub_obs_types = nh.advertise<std_msgs::Int64MultiArray>(
      "/obstacles/types", 1);

    ROS_INFO_STREAM("get_obstacles_node initiated");
    ros::spinOnce();
    msg_obstacles_poses.header.seq = 0;
    msg_obstacles_poses.header.frame_id = 1;
    while(ros::ok())
    {
        ros::spinOnce();
        if (model_states.name.size() > 0)
        {
          get_obstacles_poses(); 
          msg_obstacles_poses.header.seq++;
          msg_obstacles_poses.header.stamp = ros::Time::now();
          pub_obs_poses.publish(msg_obstacles_poses);
          pub_obs_types.publish(msg_obstacles_types);
        }
        loop_rate.sleep();
    }
    return 0;
}
