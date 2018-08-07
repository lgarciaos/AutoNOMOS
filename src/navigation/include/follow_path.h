#ifndef _FOLLOW_PATH_HEADER_
#define _FOLLOW_PATH_HEADER_

#include <iostream>

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
// #include <nav_msgs/GridCells.h>
// #include "msg/pts_array.msg"

// custom msgs
#include "motion_planning/ctrl_path.h"




#define RATE_HZ 30

double x_now, y_now, theta_now;
double x_next, y_next, theta_next;
double kp, ka, kb;

// int N = 25;

int current_point;
int current_ctrl;

std_msgs::Int16 ste;
std_msgs::Int16 vel;
geometry_msgs::Pose2D next_pose;
motion_planning::ctrl_path path;

// ros::Time segment_start;
// ros::Time now;

double duration;

double segment_duration;


void get_next_pose(); //De al edo deseado, calcular el siguiente punto obj de acuerdo al ctrl P del exámen :p



#endif
