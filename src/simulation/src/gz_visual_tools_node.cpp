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
  std::cout << "plotting lines: " << colors[lines_num] << std::endl;
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
    // std::cout << "";
    // std::cout << "pt_x: " << pt_x << "\tpt_y: " << pt_y << std::endl;
    ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(pt_x, pt_y, pt_z));
  }
  // std::cout << std::endl;
  ign_node.Request("/marker", markerMsg);

  std::cout << "ended plotting lines: " << colors[lines_num] << std::endl;

}

void get_goal_callback(const geometry_msgs::Pose2D& msg)
{
  // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';

  ignition::msgs::Marker markerMsg;
  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/Green");
  markerMsg.set_id(gz_total_lines);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  // markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
  markerMsg.clear_point();

  double x_offset = .4;
  double y_offset = .2;
  // x_offset = x_offset * cos(msg.theta) - y_offset * sin(msg.theta);
  // y_offset = x_offset * cos(msg.theta) + y_offset * sin(msg.theta);
  double z_offset = 0.001;


  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(msg.x, msg.y, z_offset, 0, 0, 0));
  // ignition::msgs::Set(markerMsg.add_point(),
  //       ignition::math::Vector3d(0, 0, 0.05));
  double radius = 0.125;
  for (double t = 0; t <= 2 * M_PI; t+= 0.01)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(radius * cos(t), radius * sin(t), z_offset));
  }
  // ign_node.Request("/marker", markerMsg);

  // ignition::msgs::Set(markerMsg.mutable_pose(),
  //   ignition::math::Pose3d(0, 0, 0, 0, 0, msg.theta));

  // double* pt_x = new double[3];
  // double* pt_y = new double[3];
  // double* pt_z = new double[3];
  //
  // if (0 <= msg.theta && msg.theta < 3.14159 / 2 )
  // {
  //   std::cout << "lines:" << __LINE__ << '\n';
  //   pt_x[0] = msg.x - x_offset; pt_y[0] = msg.y - y_offset; pt_z[0] = z_offset;
  //   pt_x[1] = msg.x + x_offset; pt_y[1] = msg.y + 0;        pt_z[1] = z_offset;
  //   pt_x[2] = msg.x - x_offset; pt_y[2] = msg.y + y_offset; pt_z[2] = z_offset;
  //
  //   // pt_x[0] = pt_x[0] * cos(msg.theta) - pt_y[0] * sin(msg.theta);
  //   // pt_y[0] = pt_x[0] * cos(msg.theta) + pt_y[0] * sin(msg.theta);
  //   // pt_x[1] = pt_x[1] * cos(msg.theta) - pt_y[1] * sin(msg.theta);
  //   // pt_y[1] = pt_x[1] * cos(msg.theta) + pt_y[1] * sin(msg.theta);
  //   // pt_x[2] = pt_x[2] * cos(msg.theta) - pt_y[2] * sin(msg.theta);
  //   // pt_y[2] = pt_x[2] * cos(msg.theta) + pt_y[2] * sin(msg.theta);
  // }
  // else if (3.14159 / 2 <= msg.theta && msg.theta < 3.14159 )
  // {
  //   std::cout << "lines:" << __LINE__ << '\n';
  //   pt_x[0] = msg.x + x_offset; pt_y[0] = msg.y - y_offset; pt_z[0] = z_offset;
  //   pt_x[1] = msg.x - x_offset; pt_y[1] = msg.y - y_offset; pt_z[1] = z_offset;
  //   pt_x[2] = msg.x + 0       ; pt_y[2] = msg.y + y_offset; pt_z[2] = z_offset;
  // }
  // else
  // {
  //   pt_x[0] = msg.x - x_offset; pt_y[0] = msg.y - y_offset; pt_z[0] = z_offset;
  //   pt_x[1] = msg.x + x_offset; pt_y[1] = msg.y + 0;        pt_z[1] = z_offset;
  //   pt_x[2] = msg.x - x_offset; pt_y[2] = msg.y + y_offset; pt_z[2] = z_offset;
  // }

  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(pt_x[0], pt_y[0], pt_z[0]));
  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(pt_x[1], pt_y[1], pt_z[1]));
  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(pt_x[2], pt_y[2], pt_z[2]));

  // printf("Goal: (%.2f, %.2f, %.2f)\n", msg.x, msg.y, msg.theta );
  // printf("offset: (%.2f, %.2f)\n", x_offset, y_offset );
  // printf("pt_1: (%.2f, %.2f)\n", pt_x[0], pt_y[0] );
  // printf("pt_2: (%.2f, %.2f)\n", pt_x[1], pt_y[1]);
  // printf("pt_3: (%.2f, %.2f)\n", pt_x[2], pt_y[2] );

  ign_node.Request("/marker", markerMsg);

}

void get_start_callback(const geometry_msgs::Pose2D& msg)
{
  // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';

  // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';

  ignition::msgs::Marker markerMsg;
  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/Yellow");
  markerMsg.set_id(gz_total_lines + 1);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
  // markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  markerMsg.clear_point();

  double x_offset = .4;
  double y_offset = .2;
  // x_offset = x_offset * cos(msg.theta) - y_offset * sin(msg.theta);
  // y_offset = x_offset * cos(msg.theta) + y_offset * sin(msg.theta);
  double z_offset = 0.001;
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(msg.x, msg.y, z_offset, 0, 0, 0));
  // ignition::msgs::Set(markerMsg.add_point(),
  //       ignition::math::Vector3d(0, 0, 0.05));
  double radius = 0.125;
  for (double t = 0; t <= 2 * M_PI; t+= 0.01)
  {
    ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(radius * cos(t), radius * sin(t), z_offset));
  }
  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(msg.x -x_offset,msg.y - y_offset, z_offset));
  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(msg.x + x_offset, msg.y + 0, z_offset));
  // ignition::msgs::Set(markerMsg.add_point(),
  //   ignition::math::Vector3d(msg.x -x_offset, msg.y + y_offset, z_offset));


  ign_node.Request("/marker", markerMsg);

}


int main(int argc, char **argv)
{
    int rate_hz = 10;
    ros::init(argc, argv, "gz_visual_tools_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Rate loop_rate(rate_hz);

    nh.param<bool>("simulation/simulation",       simulation,  true);
    nh.param<int> ("simulation/gz_total_lines",   gz_total_lines,  0);
    // nh_priv.param<std::string>("points_creation", points_creation, "GRID");
    // nh_priv.param<double>("initial_point_x", initial_pt.x, -10);

    // ros::Subscriber sub_model_states  = nh.subscribe("/gz_visual/lines", 1, &get_model_states);
    ros::Subscriber sub_goal  = nh.subscribe("/goal_pose", 1, &get_goal_callback);
    ros::Subscriber sub_start  = nh.subscribe("/start_pose", 1, &get_start_callback);
    std::cout << "NODE: gz_visual_tools_node" << '\n';
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
