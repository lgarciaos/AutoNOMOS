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
int future_points_x[MHEIGHT] = {50,80,110};
geometry_msgs::Point pt_to_send;
std_msgs::Float32MultiArray p[MHEIGHT];

int estado = -1;
std::string nombre_estado [NUM_STATES] = {"NS Izquierda", "Fuera Izquierda", "Carril Izquierdo", "Carril Derecho", "Fuera Derecha", "NS Derecha"};
int des_state = 3;
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

void get_des_state(const std_msgs::Int16& val)
{
	des_state = val.data;
}

void planning(){
	// TODO
}

std_msgs::Float32MultiArray move(std_msgs::Float32MultiArray prob, int movement)
{
	std_msgs::Float32MultiArray q;
	int pos, pos_exact, pos_undershoot, pos_overshoot;
	float s;
	ROS_INFO_STREAM("Moving: " << movement);

	for (int i = 0; i < NUM_STATES; ++i)
	{
		pos_exact      = (i - movement) % NUM_STATES;
		pos_undershoot = (i - movement - 1 + NUM_STATES) % NUM_STATES;
		pos_overshoot  = (i - movement + 1) % NUM_STATES;
		
		s = p_exact * prob.data[pos_exact];
		s += p_undershoot * prob.data[pos_undershoot];
		s += p_overshoot * prob.data[pos_overshoot];
		// ROS_INFO_STREAM("i: " << i <<"\ts: " << s << "\tpe: " << pos_exact << "\tpu: " << pos_undershoot << "\tpo: " << pos_overshoot);
		q.data.push_back(s);
	}
	// if(movement == 0){
	// ROS_INFO_STREAM("prob: " << prob);
	// ROS_INFO_STREAM("q: " << q);
	// ROS_BREAK();
	// }
	return q;
}

// void move(std_msgs::Float32MultiArray& prob)
// {
// 	int pos, pos_exact, pos_undershoot, pos_overshoot;
// 	float s;
	
	
// 	for (int m=0;m<MHEIGHT;m++){
// 		double movement = vectorMovimiento[m];
// 		ROS_INFO_STREAM("Moving: " << movement);
// 		for (int i = 0; i < NUM_STATES; ++i)
// 		{
// 			pos = i - movement;
// 			pos_exact      = (pos) % NUM_STATES;
// 			pos_undershoot = (pos - 1) % NUM_STATES;
// 			pos_overshoot  = (pos + 1) % NUM_STATES;
			
// 			s = p_exact * localizationArray.data[pos_exact];
// 			s += p_undershoot * localizationArray.data[pos_undershoot];
// 			s += p_overshoot * localizationArray.data[pos_overshoot];

// 			matrizMovimiento[m][i] = s;
// 		}
// 	}
// }


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
int det_next_state()
{
	// if (pt_x_l < pt_x_des) //ransac izq esta a la izq del pt izq
	// 		{
	// 			next_state = -1; //estaria a un edo a la izq
	// 		} else if ( pt_)
}
int det_hit(int state, int pt_x_des)
{
	// int hit;
	// int pt_x_r = 0, pt_x_c = 0, pt_x_l = 0;
	// int next_state;
	// pt_x_r = det_point(arr_right);
	// pt_x_c = det_point(arr_center);
	// pt_x_l = det_point(arr_left);
			
	// switch(pt_x_des)
	// {
	// 	case future_points[0]:
	// 		//determinar pto carril izq
	// 		det_next_state();
	// 		break;
	// 	case future_points[1]:
	// 		break;
	// 	case future_points[2]:
	// 		break;

	// }







	// int lanes_detected = (L > 0);
	// lanes_detected = lanes_detected << 1;
	// lanes_detected = lanes_detected | C > 0;
	// lanes_detected = lanes_detected << 1;
	// lanes_detected = lanes_detected | R > 0;

	// int hit;
	// switch(lanes_detected)
	// {
	// 	case 0: 
	// 		//estado actual depende si estoy más cerca de NI o ND
	// 		hit = state == 0 || state == 5;  
	// 		break;
	// 	case 1: 
	// 		hit = state == 1 ;//|| state == 2 || state == 3;
	// 		break;
	// 	case 2: 
	// 		hit = state == 2 || state == 4;
	// 		break;
	// 	case 3: 
	// 		hit = state == 2 ;//|| state == 3 || state == 4;
	// 		break;
	// 	case 4:
	// 		hit = state == 4; // || state == 3; 
	// 		break;
	// 	case 5: 
	// 		hit = state == 3;
	// 		break;
	// 	case 6: 
	// 		hit = state == 4 ;// || state == 3;
	// 		break;
	// 	case 7: 
	// 		hit = state == 3;
	// 		break;		
	// }
	// ROS_INFO_STREAM("lanes: " << lanes_detected << "\tstate:" << state << "\thit" << hit);
	// return hit;
}

std_msgs::Float32MultiArray conv(std_msgs::Float32MultiArray p, int state_to_sense)
{
	std_msgs::Float32MultiArray q;

	// for (int i = 0; i < NUM_STATES; ++i)
	// {
	// 	if (p.data[i] < 0.001)
	// 	{
	// 		q.data.push_back(0.001);
	// 	} else {
	// 		bool hit = det_hit(i, state_to_sense);
	// 		// ROS_INFO_STREAM(p.data[i] << " ==> " << p.data[i] * (hit * p_hit + (1-hit) * p_miss));
	// 		q.data.push_back(p.data[i] * (hit * p_hit + (1-hit) * p_miss));
	// 	}
	// }

	// // normalizacion
	// float sum = 0;
	// for (int i = 0; i < NUM_STATES; ++i)
	// {
	// 	sum += q.data[i];
	// }
	// for (int i = 0; i < NUM_STATES; ++i)
	// {
	// 	q.data[i] /= sum;
	// }

	return q;
}


std_msgs::Float32MultiArray sense(std_msgs::Float32MultiArray prob, int state_to_sense)
{
	std_msgs::Float32MultiArray q;

	q = conv(prob, state_to_sense);
	
	return q;
}

void estimate_move()
{
	for (int i = 0; i < MHEIGHT; ++i)
	{
		// ROS_INFO_STREAM("estimate_move: " <<  i);
		// ROS_INFO_STREAM("printing..." << p[i]);
		// ROS_INFO_STREAM("done?");
		p[i] = move(localizationArray, vectorMovimiento[i]);
	}
}

void estimate_next_state()
{
	for (int i = 0; i < MHEIGHT; ++i)
	{
		p[i] = sense(p[i], vectorMovimiento[i]);
	}
}

int det_next_move()
{	
	float max = 0;
	int  next_ctrl = -1;
	for (int i = 0; i < MHEIGHT; ++i)
	{
		if (max < p[i].data[des_state])
		{
			ROS_INFO_STREAM("max: "<< max << "\tnext_ctrl: " << next_ctrl) ;
			max = p[i].data[des_state];
			next_ctrl = i;
		}
	}

	ROS_INFO_STREAM("max: "<< max << "\tnext_ctrl: " << next_ctrl) ;
	// ROS_ASSERT(res != -1);
	return future_points_x[next_ctrl];

}

void init_p()
{	
	for (int i = 0; i < MHEIGHT; ++i)
	{
		for (int j = 0; j < NUM_STATES; ++j)
		{
			p[i].data.push_back((float) (1/(float)NUM_STATES));
		}
		ROS_INFO_STREAM("p[" << i << "]: " << p[i]);
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
	ros::Subscriber sub_des_state = nh.subscribe("/desire_state",1, get_des_state);
	
	init_p();
	ROS_INFO_STREAM("p[0];" << p[0]);
	loop_rate.sleep();
	loop_rate.sleep();
	loop_rate.sleep();
	while(nh.ok())
	{
		// ROS_INFO_STREAM("at 1");
	    ros::spinOnce();
	    // ROS_INFO_STREAM("at 2");
	    // planning();
	    // ROS_INFO_STREAM("at 3");
	    // estimate_next_state();
	    // ROS_INFO_STREAM("at 4");
	    estimate_move();
	    // ROS_INFO_STREAM("at 5");
	    det_next_move();
	    ROS_INFO_STREAM("locArr" << localizationArray);
	    ROS_INFO_STREAM("next p" << p[0]);
	    ROS_INFO_STREAM("next p" << p[1]);
	    ROS_INFO_STREAM("next p" << p[2]);
	    // ROS_INFO_STREAM("at 6");
	    pt_to_send.x = 100;
	    pt_to_send.y = det_next_move();
	    ROS_INFO_STREAM("Moving to: (" << pt_to_send.x << " , " << pt_to_send.y << " )" ) ; 
	    pt_to_send.z = 0;
	    path_planned.cells.push_back(pt_to_send);
	    pub_path.publish(path_planned);
	    loop_rate.sleep();
	}
	return 0;
};

