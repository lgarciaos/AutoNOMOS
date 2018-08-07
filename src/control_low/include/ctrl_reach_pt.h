// std
#include <iostream>

// ros
#include <ros/ros.h>
#include <signal.h>

// ros msgs
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

// Boost
#include <boost/format.hpp>

#define RATE_HZ 30

static const uint32_t MY_ROS_QUEUE_SIZE = 1;

int autonomos_number;
bool simulation;

ros::Subscriber sub_vel;
ros::Subscriber sub_ste;

void get_ctrl_action_steer(const geometry_msgs::Pose2D& val);

void get_ctrl_action_vel(const std_msgs::Int16& val);

void get_vel_vec(const geometry_msgs::Pose2D& msg);

void mySigintHandler(int sig);
