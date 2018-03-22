#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>
#include <string>
#include <sstream>
//#include <random>
//@AUTHOR: GARY
#define NUM_STATES 9 
#define HEIGHT 90
geometry_msgs::Twist destiny_position;
double rate_hz = 5;
ros::Publisher pub_loc;
ros::Publisher pub_lidar;
std::string nombre;

nav_msgs::GridCells arr_left; 
nav_msgs::GridCells arr_center;
nav_msgs::GridCells arr_right;

std_msgs::Float32MultiArray p;

// estados: 	 NSI,   FI,   CI,   CD,   FD, NSD

// float c0 [6] = {0.40, 0.05, 0.05, 0.05, 0.05, 0.40};
// float c1 [6] = {1/30, 0.40, 0.25, 0.25, 1/30, 1/30};
// float c2 [6] = {1/30, 1/30, 0.35, 0.20, 0.35, 1/30};
// float c3 [6] = {1/40, 1/40, 0.60, 0.30, 1/40, 1/40};
// float c4 [6] = {1/40, 1/40, 1/40, 0.30, 0.60, 1/40};
// float c5 [6] = {0.02, 0.02, 0.02, 0.90, 0.02, 0.02};
// float c6 [6] = {1/40, 1/40, 1/40, 0.30, 0.60, 1/40};
// float c7 [6] = {0.02, 0.02, 0.02, 0.90, 0.02, 0.02};
					// dist  0    1       2     3     4     5     6    7     8 
					// 1..9 = n*(n+1)/2  ==> 9*10/2 = 9*5 = 45    
// float prob_dist_edo [9] = {9/45.0, 8/45.0, 7/45.0, 6/45.0, 5/45.0, 4/45.0, 3/45.0, 2/45.0, 1/45.0};

// std::string nombre_estado [NUM_STATES] = {"NS Izquierda", "Fuera Izquierda", "Carril Izquierdo", "Carril Derecho", "Fuera Derecha", "NS Derecha"};

float p_exact = .35;
float p_undershoot = .45;
float p_overshoot = .2;

float p_hit = 0.2;
float p_miss = 0.1; 

float alpha = 12; //TODO

int movement = 45;
// PERCEPCION DE LIDAR

int L = 0;
int C = 0;
int R = 0;
int des_state = 5;

int ctrl_action = 45;
//gets the left points
void get_pts_left(const nav_msgs::GridCells& array);
//gets the center points
void get_pts_center(const nav_msgs::GridCells& array);

//gets the right points
void get_pts_right(const nav_msgs::GridCells& array);

//transforms the motion into values for shift >> used before but maybe not useful anymore (290317)
void get_ctrl_action(const std_msgs::Int16& val);

//gets and stores the desired state
void get_des_state(const std_msgs::Int16& val);

//calculates the distance. NOTE: only using th x component because Y is asumed constante, maybe isnt the best way to have it.
//If y is asumed constant ==> using abs() instead of sqrt might be more efficient
float dist(geometry_msgs::Point p1, geometry_msgs::Point p2);

int det_hit (int state);

std_msgs::Float32MultiArray conv(std_msgs::Float32MultiArray p);


std_msgs::Float32MultiArray sense(std_msgs::Float32MultiArray prob);

int combinaciones( int n, int r );

float det_prob(int edo_ini, int ctrl_action, int edo_fin);

std_msgs::Float32MultiArray move(std_msgs::Float32MultiArray prob);

void print_state_order();


