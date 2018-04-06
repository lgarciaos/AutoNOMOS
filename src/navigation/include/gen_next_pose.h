#ifndef _GEN_NEXT_POSE_HEADER_
#define _GEN_NEXT_POSE_HEADER_

#include <iostream>

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>

#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
// #include "msg/pts_array.msg"

using namespace std;


#define RATE_HZ 30

#define STATE_DONT_KNOW_LEFT 0
#define STATE_OUTSIDE_LEFT 1
#define STATE_LEFT_LEFT 2
#define STATE_LEFT_CENTER 3
#define STATE_CENTER_CENTER 4
#define STATE_RIGHT_CENTER 5
#define STATE_RIGHT_RIGHT 6
#define STATE_OUTSIDE_RIGHT 7
#define STATE_DONT_KNOW_RIGHT 8

int N = 25;

geometry_msgs::Point point_N_left;
geometry_msgs::Point point_N_center;
geometry_msgs::Point point_N_right;

float x_car = 78;
float y_car = 158;

int state = STATE_RIGHT_CENTER;

int rate_hz = 10;

geometry_msgs::Pose2D next_pose;

void get_des_state(const std_msgs::Int16& val);

void get_rensac_left(const nav_msgs::GridCells& val);

void get_rensac_center(const nav_msgs::GridCells& val);

void get_rensac_right(const nav_msgs::GridCells& val);


geometry_msgs::Pose2D get_next_pose(); //De al edo deseado, calcular el siguiente punto obj de acuerdo al ctrl P del exámen :p



#endif 
