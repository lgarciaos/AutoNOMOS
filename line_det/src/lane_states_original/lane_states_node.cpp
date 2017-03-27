#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>

#define NUM_STATES 6

geometry_msgs::Twist destiny_position;
double rate_hz = 1;
ros::Publisher pub_loc;
ros::Publisher pub_lidar;
std::string nombre;

nav_msgs::GridCells arr_left; 
nav_msgs::GridCells arr_center;
nav_msgs::GridCells arr_right;

std_msgs::Float32MultiArray p;

// estados: 	 NSI,   FI,   CI,   CD,   FD, NSD

float c0 [6] = {0.40, 0.05, 0.05, 0.05, 0.05, 0.40};
float c1 [6] = {1/30, 0.40, 0.25, 0.25, 1/30, 1/30};
float c2 [6] = {1/30, 1/30, 0.35, 0.20, 0.35, 1/30};
float c3 [6] = {1/40, 1/40, 0.60, 0.30, 1/40, 1/40};
float c4 [6] = {1/40, 1/40, 1/40, 0.30, 0.60, 1/40};
float c5 [6] = {0.02, 0.02, 0.02, 0.90, 0.02, 0.02};
float c6 [6] = {1/40, 1/40, 1/40, 0.30, 0.60, 1/40};
float c7 [6] = {0.02, 0.02, 0.02, 0.90, 0.02, 0.02};

std::string nombre_estado [NUM_STATES] = {"NS Izquierda", "Fuera Izquierda", "Carril Izquierdo", "Carril Derecho", "Fuera Derecha", "NS Derecha"};


float p_exact = .5;
float p_undershoot = .25;
float p_overshoot = .25;

float p_hit = 0.6;
float p_miss = 0.2; 

int movement = 0;
// PERCEPCION DE LIDAR

int L = 0;
int C = 0;
int R = 0;
int des_state = 3;

void get_pts_left(const nav_msgs::GridCells& array)
{
	arr_left.cells = array.cells;
	L = array.cell_width;
	// lines_sensed = array.cell_width > 0 ?  lines_sensed | 4 : lines_sensed | 0;
}

void get_pts_center(const nav_msgs::GridCells& array)
{
	arr_center.cells = array.cells;
	C = array.cell_width;
	// lines_sensed = array.cell_width > 0 ?  lines_sensed | 2 : lines_sensed | 0;
}

void get_pts_right(const nav_msgs::GridCells& array)
{
	arr_right.cells = array.cells;
	R = array.cell_width;
	// ROS_INFO_STREAM("Array: " << array);
	// ROS_INFO_STREAM("len: " << sizeof(array.data) / sizeof(array.data[0]) << " 1:" << sizeof(array.data) << " 2: " << sizeof(array.data[0]));
	// lines_sensed = array.cell_width > 0 ?  lines_sensed | 1 : lines_sensed | 0;
}

void get_motion(const std_msgs::Int16& val)
{
	ROS_INFO_STREAM("Steering: " << val.data);
	if(val.data >= 0 && val.data < 30)
		movement = 1;
	else if(val.data >= 30 && val.data < 60)
		movement = 0;
	else if(val.data >= 60 && val.data < 90)
		movement = -1;
}

void get_des_state(const std_msgs::Int16& val)
{
	des_state = val.data;
}

std_msgs::Float32MultiArray conv(bool hit, std_msgs::Float32MultiArray p)
{
	std_msgs::Float32MultiArray q;

	for (int i = 0; i < NUM_STATES; ++i)
	{
		if (p.data[i] < 0.001)
		{
			q.data.push_back(0.001);
		} else {
			q.data.push_back(p.data[i] * (hit * p_hit + (1-hit) * p_miss));
		}
	}

	// normalizacion
	float sum = 0;
	for (int i = 0; i < NUM_STATES; ++i)
	{
		sum += q.data[i];
	}
	for (int i = 0; i < NUM_STATES; ++i)
	{
		q.data[i] /= sum;
	}

	return q;
}

int det_actual_state()
{
	int lanes_detected = (L > 0);
	lanes_detected = actual_state << 1;
	lanes_detected = C > 0;
	lanes_detected = actual_state << 1;
	lanes_detected = R > 0;
	// bool hit = (des_state == actual_state);
        // q.append(p[i] * (hit * pHit + (1-hit) * pMiss))

	ROS_INFO_STREAM("L: " << L << "C: " << C << "R: " << R);

	int actual_state;
	switch(lanes_detected)
	{
		case 0: 
			//estado actual depende si estoy más cerca de NI o ND
			actual_state = desire_state < 3 ? 0 : 5;  
			break;
		case 1: 
			actual_state = 1;
			break;
		case 2: 
			actual_state = 4;
			break;
		case 3: 
			actual_state = 2;
			break;
		case 4:
			actual_state = 4; 
			break;
		case 5: 
			actual_state = 3;
			break;
		case 6: 
			actual_state = 4;
			break;
		case 7: 
			actual_state = 3;
			break;		
	}
	ROS_INFO_STREAM("Actual state: " << actual_state);
	return actual_state
}

std_msgs::Float32MultiArray sense(std_msgs::Float32MultiArray prob)
{
	std_msgs::Float32MultiArray q;

	bool hit = desire_state == det_actual_state();

	q = conv(hit, prob);
	
	// if(L > 0 && C > 0 && R > 0) {
	// 	q = conv(c7, prob); ROS_INFO_STREAM("sense: " << 7);
	// }
	// else if(L > 0 && C > 0 && R == 0)	{
	// 	q = conv(c6, prob); ROS_INFO_STREAM("sense: " << 6);
	// }
	// else if(L > 0 && C == 0 && R > 0)	{
	// 	q = conv(c5, prob); ROS_INFO_STREAM("sense: " << 5);
	// }
	// else if(L > 0 && C == 0 && R == 0) {	
	// 	q = conv(c4, prob); ROS_INFO_STREAM("sense: " << 4);
	// }
	// else if(L==0 && C > 0 && R > 0)	{
	// 	q = conv(c3, prob); ROS_INFO_STREAM("sense: " << 3);
	// }
	// else if(L==0 && C > 0 && R == 0)	{
	// 	q = conv(c2, prob); ROS_INFO_STREAM("sense: " << 2);
	// }
	// else if(L==0 && C == 0 && R > 0) {
	// 	q = conv(c1, prob); ROS_INFO_STREAM("sense: " << 1);
	// }
	// else if(L==0 && C == 0 && R ==0) {
	// 	q = conv(c0, prob); ROS_INFO_STREAM("sense: " << 0);
	// }
	
	return q;
}

std_msgs::Float32MultiArray move(std_msgs::Float32MultiArray prob)
{
	std_msgs::Float32MultiArray q;
	int pos, pos_exact, pos_undershoot, pos_overshoot;
	float s;
	ROS_INFO_STREAM("Moving: " << movement);
	for (int i = 0; i < NUM_STATES; ++i)
	{
		pos_exact      = (i - movement) % NUM_STATES;
		pos_undershoot = (i - movement - 1) % NUM_STATES;
		pos_overshoot  = (i - movement + 1) % NUM_STATES;
		
		s = p_exact * prob.data[pos_exact];
		s += p_undershoot * prob.data[pos_undershoot];
		s += p_overshoot * prob.data[pos_overshoot];

		q.data.push_back(s);
	}
	return q;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "lane_states_node");
	ROS_INFO_STREAM("lane_states_node initialized");
	ros::NodeHandle nh;
	ros::Rate loop_rate(rate_hz);

	ROS_INFO_STREAM("First prob: " << 1.0/(float)NUM_STATES);
	for (int i = 0; i < NUM_STATES; ++i)
	{
		p.data.push_back((float) (1/(float)NUM_STATES));
	}
	ROS_INFO_STREAM("Array initialization: \n" << p);

	pub_loc = nh.advertise<std_msgs::Float32MultiArray>("/localization_array", rate_hz);

	ros::Subscriber sub_pts_left = nh.subscribe("/points/left",1, get_pts_left);
	ros::Subscriber sub_pts_center = nh.subscribe("/points/center",1, get_pts_center);
	ros::Subscriber sub_pts_right = nh.subscribe("/points/right",1, get_pts_right);
	ros::Subscriber sub_mov = nh.subscribe("/manual_control/steering",1,get_motion);
	ros::Subscriber sub_des_state = nh.subscribe("/desire_state",1, get_des_state);
	
	while(nh.ok())
	{
		L=0;
		R=0;
		C=0;

	    ros::spinOnce();
	    p = sense(p);
	    // p = move(p);
	    pub_loc.publish(p);

	    ROS_INFO_STREAM("Histogram: [" << p.data[0] << "," << p.data[1] << "," << p.data[2] << ","<< p.data[3] << ","<< p.data[4] << ","<< p.data[5]);

	    // detectar estado de mayor probabilidad para imprimirlo
	    float max=0;
	    for(int i=0;i<NUM_STATES;i++){
	    	if(p.data[i]>max){
	    		max=p.data[i];
	    	}
	    }

	    for(int i=0;i<NUM_STATES;i++){
	    	if(p.data[i]==max){
	    		ROS_INFO_STREAM("Estas en:" << nombre_estado[i]);
	    	}
	    }

	    movement = 0;

	    loop_rate.sleep();
	}
	return 0;
};

