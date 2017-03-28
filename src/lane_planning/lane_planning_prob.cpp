#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>
#include <vector>

#define NUM_STATES 6
#define LEFT 0
#define RIGHT 1

#define NSI 0
#define FI 1
#define CI 2
#define CD 3
#define FD 4
#define NSD 5

static const uint32_t MY_ROS_QUEUE_SIZE = 1;

#define MHEIGHT 3

double rate_hz = 1;
ros::Publisher pub_path;
// ros::Publisher pub_lidar;
int corte = 0;

std::string nombre;

nav_msgs::GridCells arr_left; 
nav_msgs::GridCells arr_center;
nav_msgs::GridCells arr_right;

std_msgs::Float32MultiArray localizationArray;

nav_msgs::GridCells path_planned;

float p_exact = .5;
float p_undershoot = .25;
float p_overshoot = .25;
double matrizMovimiento[MHEIGHT][NUM_STATES];
double vectorMovimiento[MHEIGHT] = {-1,0,1};

int estado = -1;
std::string nombre_estado [NUM_STATES] = {"NS Izquierda", "Fuera Izquierda", "Carril Izquierdo", "Carril Derecho", "Fuera Derecha", "NS Derecha"};

// estados: 	 NSI,   FI,   CI,   CD,   FD, NSD
void get_pts_left(const nav_msgs::GridCells& array)
{
	arr_left.cells = array.cells;
	arr_left.cell_width = array.cell_width;
	// lines_sensed = array.cell_width > 0 ?  lines_sensed | 4 : lines_sensed | 0;
}

void get_pts_center(const nav_msgs::GridCells& array)
{
	arr_center.cells = array.cells;
	arr_center.cell_width = array.cell_width;
	// lines_sensed = array.cell_width > 0 ?  lines_sensed | 2 : lines_sensed | 0;
}

void get_pts_right(const nav_msgs::GridCells& array)
{
	arr_right.cells = array.cells;
	arr_right.cell_width = array.cell_width;
	// ROS_INFO_STREAM("Array: " << array);
	// lines_sensed = array.cell_width > 0 ?  lines_sensed | 1 : lines_sensed | 0;
}

void planning(){
	// TODO
}

void move(std_msgs::Float32MultiArray& prob)
{
	int pos, pos_exact, pos_undershoot, pos_overshoot;
	float s;
	
	
	for (int m=0;m<MHEIGHT;m++){
		double movement = vectorMovimiento[m];
		ROS_INFO_STREAM("Moving: " << movement);
		for (int i = 0; i < NUM_STATES; ++i)
		{
			pos = i - movement;
			pos_exact      = (pos) % NUM_STATES;
			pos_undershoot = (pos - 1) % NUM_STATES;
			pos_overshoot  = (pos + 1) % NUM_STATES;
			
			s = p_exact * localizationArray.data[pos_exact];
			s += p_undershoot * localizationArray.data[pos_undershoot];
			s += p_overshoot * localizationArray.data[pos_overshoot];

			matrizMovimiento[m][i] = s;
		}
	}
}

void get_localization(const std_msgs::Float32MultiArray& locArray) {
	// detectar estado de mayor probabilidad para imprimirlo

	localizationArray.data = locArray.data;

	float max=0;
	for(int i=0;i<NUM_STATES;i++){
	 	if(locArray.data[i]>max){
	 		max=locArray.data[i];
	 	}
	}

	for(int i=0;i<NUM_STATES;i++){
	 	if(locArray.data[i]==max){
	 		ROS_INFO_STREAM("Estas en:" << nombre_estado[i]);
	 		estado = i;
	 	}
	}
}



int main(int argc, char** argv){
	ros::init(argc, argv, "lane planning node");
	ROS_INFO_STREAM("lane_planning_node initialized");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");
	ros::Rate loop_rate(rate_hz);

	std::string node_name = ros::this_node::getName();

	priv_nh_.param<int>(node_name+"/corte", corte, 140);

	pub_path = nh.advertise<nav_msgs::GridCells>("/planning", rate_hz);
	
	ros::Subscriber sub_pts_left = nh.subscribe("/points/ransac_left",1, &get_pts_left);
	ros::Subscriber sub_pts_center = nh.subscribe("/points/ransac_center",1, &get_pts_center);
	ros::Subscriber sub_pts_right = nh.subscribe("/points/ransac_right",1, &get_pts_right);
	ros::Subscriber sub_localization = nh.subscribe("/localization_array",1, &get_localization);

	
	while(nh.ok())
	{
	    ros::spinOnce();
	    planning();
	    // p = move(p);
	    //pub_path.publish(p);
	    loop_rate.sleep();
	}
	return 0;
};

