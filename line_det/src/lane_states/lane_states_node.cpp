#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <math.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>

#define NUM_STATES 160

// geometry_msgs::Twist destiny_position;

double rate_hz = 10;
ros::Publisher pub_loc;
ros::Publisher pub_lidar;
std::string nombre;

nav_msgs::GridCells arr_left; 
nav_msgs::GridCells arr_center;
nav_msgs::GridCells arr_right;

// char lines_sensed = 0 ;

std_msgs::Float32MultiArray p;
std_msgs::Float32MultiArray senseArray;

// float c0 [6] = {0.40, 0.05, 0.05, 0.05, 0.05, 0.40};
// float c1 [6] = {1/30, 0.40, 0.25, 0.25, 1/30, 1/30};
// float c2 [6] = {1/30, 1/30, 0.35, 0.20, 0.35, 1/30};
// float c3 [6] = {1/40, 1/40, 0.60, 0.30, 1/40, 1/40};
// float c4 [6] = {1/40, 1/40, 1/40, 0.30, 0.60, 1/40};
// float c5 [6] = {0.02, 0.02, 0.02, 0.90, 0.02, 0.02};
// float c6 [6] = {1/40, 1/40, 1/40, 0.30, 0.60, 1/40};
// float c7 [6] = {0.02, 0.02, 0.02, 0.90, 0.02, 0.02};

// float gauss[5]={.05,.25,.4,.25,.05};

// std::string nombre_estado [NUM_STATES] = {"NS Izquierda", "Fuera Izquierda", "Carril Izquierdo", "Carril Derecho", "Fuera Derecha", "NS Derecha"};

float p_exact = .7;
float p_undershoot = .15;
float p_overshoot = .15;

int movement = 0;
int orientation = 0;
float forward_noise = 2.0;
// PERCEPCION DE LIDAR



void get_pts_left(const nav_msgs::GridCells& array)
{
	//arr_left.cells = array.cells;
	//lines_sensed = array.cell_width > 0 ?  lines_sensed | 4 : lines_sensed | 0;
	for(int i=0;i<array.cell_width;i++){
		senseArray.data[array.cells[i].x]++;
	}
	// ROS_INFO_STREAM("Left array: " << array);
}

void get_pts_center(const nav_msgs::GridCells& array)
{
	//arr_center.cells = array.cells;
	//lines_sensed = array.cell_width > 0 ?  lines_sensed | 2 : lines_sensed | 0;
	for(int i=0;i<array.cell_width;i++){
		senseArray.data[array.cells[i].x]++;
	}
}

void get_pts_right(const nav_msgs::GridCells& array)
{
	//arr_right.cells = array.cells;
	// ROS_INFO_STREAM("Array: " << array);
	// ROS_INFO_STREAM("len: " << sizeof(array.data) / sizeof(array.data[0]) << " 1:" << sizeof(array.data) << " 2: " << sizeof(array.data[0]));
	// lines_sensed = array.cell_width > 0 ?  lines_sensed | 1 : lines_sensed | 0;
	for(int i=0;i<array.cell_width;i++){
		senseArray.data[array.cells[i].x]++;
	}
}

void get_motion(const std_msgs::Int16& val)
{
	orientation=abs(val.data-90)-45;
	ROS_INFO_STREAM("Steering: " << val.data << ", orientation: " << orientation);
}


void sense()
{
	ROS_INFO_STREAM("sense");
	
	for (int i = 0; i < NUM_STATES; i++)
	{
		p.data[i]=p.data[i]*senseArray.data[i];
		senseArray.data[i] = 1.0;
	}

	// Normalize
	float sum = 0.0;
	for (int i = 0; i < NUM_STATES; i++){
		sum+=p.data[i];
	}
	if(sum>0){
		for (int i = 0; i < NUM_STATES; i++){
			p.data[i]=p.data[i]/sum;
		}
	}
	
}


std_msgs::Float32MultiArray move()
{
	ROS_INFO_STREAM("move");
	std_msgs::Float32MultiArray dist;
	/*
	float velocidad = 60;
	float tiempo = 1/60;
	for (int i = 0; i < 20; ++i)
	{
		dist.data.push_back(velocidad*tiempo + random.gauss(0.0, forward_noise));
	}
	*/

	if(orientation>0)
	{
		int U = (cos(orientation) * 1);
		
		for (int i = 0; i < NUM_STATES; ++i)
		{
			float s = p_exact * p.data[(i-U) % NUM_STATES];
	        s = s + p_overshoot * p.data[(i-U-1) % NUM_STATES];
	        s = s + p_undershoot * p.data[(i-U+1) % NUM_STATES];

			dist.data.push_back(s);
		}
		return dist;
	}
	else
		return p;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "lane_states_node");
	ROS_INFO_STREAM("lane_states_node initialized");
	ros::NodeHandle nh;
	ros::Rate loop_rate(rate_hz);

	//ROS_INFO_STREAM("Array initialization: \n" << p);
	for (int i = 0; i < NUM_STATES; ++i)
	{
		p.data.push_back((float) (1/(float)NUM_STATES));
		senseArray.data.push_back(1.0);
	}

	ROS_INFO_STREAM("inicializado");
	pub_loc = nh.advertise<std_msgs::Float32MultiArray>("/localization_array", rate_hz);

	ros::Subscriber sub_pts_left = nh.subscribe("/points/left",1, get_pts_left);
	ros::Subscriber sub_pts_center = nh.subscribe("/points/center",1, get_pts_center);
	ros::Subscriber sub_pts_right = nh.subscribe("/points/right",1, get_pts_right);
	ros::Subscriber sub_mov = nh.subscribe("/manual_control/steering",1,get_motion);

	pub_loc.publish(p);

	while(nh.ok())
	{
	    ros::spinOnce();
	    sense();
	    // p=move();
	    pub_loc.publish(p);
   		ROS_INFO_STREAM(std::endl << "Histograma: " << p);
	    loop_rate.sleep();
	}
	return 0;
};

