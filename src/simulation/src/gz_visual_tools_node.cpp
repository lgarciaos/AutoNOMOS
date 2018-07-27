// std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iterator>
#include <cassert>

// ros
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ros messages
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose2D.h>

// Boost
#include "boost/bind.hpp"

// ignition
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>

bool simulation;
int gz_total_lines;
std::vector<std::vector<double>> lines_vec;
std::vector<std::string> colors = {"Gazebo/Blue", "Gazebo/Purple", "Gazebo/White"};
int markers_id;
ignition::transport::Node ign_node;
// ignition::msgs::Marker markerMsg;


void get_lines_callback(const std_msgs::Float64MultiArrayConstPtr& msg, const int& lines_num)
{
  // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';

  float pt_x, pt_y, pt_z = 0.01;
  pt_z = pt_z - lines_num * pt_z / gz_total_lines;
  // std::cout << "lines_num: " << lines_num << "\tsize: " << msg -> data.size()
  //   << "\tz: " << pt_z <<'\n';
  ignition::msgs::Marker markerMsg;
  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name(colors[lines_num]);
  markerMsg.set_id(lines_num);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
  markerMsg.clear_point();

  for (int j = 0; j < msg -> data.size(); j+=2)
  {
    pt_x = msg -> data[j];
    pt_y = msg -> data[j+1];
    ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(pt_x, pt_y, pt_z));
  }
  ign_node.Request("/marker", markerMsg);


}

void get_goal_callback(const geometry_msgs::Pose2D& msg)
{
  // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';

  ignition::msgs::Marker markerMsg;
  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/Green");
  markerMsg.set_id(gz_total_lines);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  markerMsg.clear_point();

  double x_offset = .4;
  double y_offset = .2;
  double z_offset = 0.001;
  ignition::msgs::Set(markerMsg.mutable_pose(),
    ignition::math::Pose3d(0, 0, 0, 0, 0, msg.theta));
  ignition::msgs::Set(markerMsg.add_point(),
    ignition::math::Vector3d(msg.x -x_offset,msg.y - y_offset, z_offset));
  ignition::msgs::Set(markerMsg.add_point(),
    ignition::math::Vector3d(msg.x + x_offset, msg.y + 0, z_offset));
  ignition::msgs::Set(markerMsg.add_point(),
    ignition::math::Vector3d(msg.x -x_offset, msg.y + y_offset, z_offset));

  ign_node.Request("/marker", markerMsg);

}

void get_start_callback(const geometry_msgs::Pose2D& msg)
{
  // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';

  ignition::msgs::Marker markerMsg;
  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/Yellow");
  markerMsg.set_id(gz_total_lines + 1);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  markerMsg.clear_point();

  double x_offset = .4;
  double y_offset = .2;
  double z_offset = 0.001;
  ignition::msgs::Set(markerMsg.mutable_pose(),
    ignition::math::Pose3d(0, 0, 0, 0, 0, msg.theta));
  ignition::msgs::Set(markerMsg.add_point(),
    ignition::math::Vector3d(msg.x -x_offset,msg.y - y_offset, z_offset));
  ignition::msgs::Set(markerMsg.add_point(),
    ignition::math::Vector3d(msg.x + x_offset, msg.y + 0, z_offset));
  ignition::msgs::Set(markerMsg.add_point(),
    ignition::math::Vector3d(msg.x -x_offset, msg.y + y_offset, z_offset));

  ign_node.Request("/marker", markerMsg);

}


int main(int argc, char **argv)
{
    int rate_hz = 10;
    ros::init(argc, argv, "gz_visual_tools_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Rate loop_rate(rate_hz);

    nh.param<bool>("simulation",        simulation,  true);
    nh.param<int> ("gz_total_lines",   gz_total_lines,  0);
    // nh_priv.param<std::string>("points_creation", points_creation, "GRID");
    // nh_priv.param<double>("initial_point_x", initial_pt.x, -10);

    // ros::Subscriber sub_model_states  = nh.subscribe("/gz_visual/lines", 1, &get_model_states);
    ros::Subscriber sub_goal  = nh.subscribe("/goal_pose", 1, &get_goal_callback);
    ros::Subscriber sub_start  = nh.subscribe("/start_pose", 1, &get_start_callback);
    std::cout << "gz_total_lines: " << gz_total_lines << '\n';
    std::vector<ros::Subscriber> sub_gazebo_lines_visualizer;

    lines_vec.reserve(gz_total_lines);
    for(int i = 0; i < gz_total_lines; i++)
    {
      std::stringstream i_ss;
      i_ss << "/gz_visual/lines_" << i;
	     sub_gazebo_lines_visualizer.push_back(
         nh.subscribe<std_msgs::Float64MultiArray>(i_ss.str(), 1,
         boost::bind(get_lines_callback, _1, i)));
    }

    // ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("/target_pose", 1);

    ROS_INFO_STREAM("sim_tools_testing_node initiated");
    ros::spinOnce();

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
