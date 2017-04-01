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

#define NUM_STATES 9 
#define HEIGHT 90
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
					// dist  0    1       2     3     4     5     6    7     8 
					// 1..9 = n*(n+1)/2  ==> 9*10/2 = 9*5 = 45    
float prob_dist_edo [9] = {9/45.0, 8/45.0, 7/45.0, 6/45.0, 5/45.0, 4/45.0, 3/45.0, 2/45.0, 1/45.0};

std::string nombre_estado [NUM_STATES] = {"NS Izquierda", "Fuera Izquierda", "Carril Izquierdo", "Carril Derecho", "Fuera Derecha", "NS Derecha"};


float p_exact = .35;
float p_undershoot = .45;
float p_overshoot = .2;

float p_hit = 0.6;
float p_miss = 0.2; 

float alpha = 12; //TODO

int movement = 0;
// PERCEPCION DE LIDAR

int L = 0;
int C = 0;
int R = 0;
int des_state = 3;

int ctrl_action = 0;
//gets the left points
void get_pts_left(const nav_msgs::GridCells& array)
{
	arr_left.cells = array.cells;
	L = array.cell_width;
}
//gets the center points
void get_pts_center(const nav_msgs::GridCells& array)
{
	arr_center.cells = array.cells;
	C = array.cell_width;
}

//gets the right points
void get_pts_right(const nav_msgs::GridCells& array)
{
	arr_right.cells = array.cells;
	R = array.cell_width;
}

//transforms the motion into values for shift >> used before but maybe not useful anymore (290317)
void get_ctrl_action(const std_msgs::Int16& val)
{
	ctrl_action = val.data;
}
//gets and stores the desired state
void get_des_state(const std_msgs::Int16& val)
{
	des_state = val.data;
}
//calculates the distance. NOTE: only using th x component because Y is asumed constante, maybe isnt the best way to have it.
//If y is asumed constant ==> using abs() instead of sqrt might be more efficient
float dist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
//	ROS_INFO_STREAM("x1: " << p1.x << "\ty1: "<< p1.y << "\tx2: " << p2.x << "\ty2: " << p2.y);
	float dif_x = p1.x - p2.x;
//	float dif_y = p1.y - p2.y;
	return sqrt(dif_x * dif_x);
}

//determines a hit based on the number of lines detected and the distance from the car center to the lines
//to determine lines detected, the following table is used:
//   | 	L  |  C  |  R
//===================== 
// 0 |  0  |  0  |  0
// 1 |  0  |  0  |  1
// 2 |  0  |  1  |  0
// 3 |  0  |  1  |  1
// 4 |  1  |  0  |  0
// 5 |  1  |  0  |  1
// 6 |  1  |  1  |  0
// 7 |  1  |  1  |  1
//
//The states are:
//		|	|	|			NI -> No se  Izq
//		|		|			AI -> Afuera Izq
//		|	|	|			LL -> Left Left
//		|		|			LC -> Left Center
//		|	|	|			CC -> Center Center
//		|		|			RC -> Right Center
//		|	|	|			RR -> Right Right
//   NI | AI  |LL| LC |CC |RC |RR | AD | ND		AD -> Afuera Derecha
//   0	| 1   |2 | 3  |4  |5  |6  | 7  | 8		ND -> No se Derecha
//	
//
int det_hit (int state)
{
	//Determine the number of lanes seen
	int lanes_detected = (L > 0);
	lanes_detected = lanes_detected << 1;
	lanes_detected = lanes_detected | C > 0;
	lanes_detected = lanes_detected << 1;
	lanes_detected = lanes_detected | R > 0;
	
	geometry_msgs::Point pt_r ;
	geometry_msgs::Point pt_c ;
	geometry_msgs::Point pt_l ;
	geometry_msgs::Point pt_car ;
	
	//define the static point (center) of the car
	pt_car.x = 80;
	pt_car.y = 160;
	pt_car.z = 0;
	// if there are points in the line ==> get the last point (the last point is the closer to the car)
	if ( R > 0 ) pt_r = arr_right.cells[R - 1] ;
	if ( C > 0 ) pt_c = arr_center.cells[C - 1];
	if ( L > 0 ) pt_l = arr_left.cells[L - 1]  ;

	// if there are points in the lines, get the distance between the car and the closest point if not, assign a BIG number
	float dist_rr = R > 0 ?  dist(pt_r, pt_car) : 1000;
	float dist_cc = C > 0 ?  dist(pt_c, pt_car) : 1000;
	float dist_ll = L > 0 ?  dist(pt_l, pt_car) : 1000;
	
	// define if each distance is smaller than alpha
	bool rr = dist_rr < alpha;
	bool cc = dist_cc < alpha;
	bool ll = dist_ll < alpha;
	//printing for debugging
	//ROS_INFO_STREAM("rr: " << rr << "\tcc: " << cc << "\tll: " << ll << "\tlanes: " << lanes_detected);
	//ROS_INFO_STREAM("dist_rr: " << dist_rr << "\tdist_cc: " << dist_cc << "\tdist_ll: " << dist_ll << "\talpha: " << alpha);
	
	//var to return
	int hit;
	//switch depending on the state to eval
	switch (state)
	{
		case 0:	//is hit if there are no lines
			hit = lanes_detected == 0;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 1: //is hit 
			hit = lanes_detected == 1;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 2: 
			hit = cc && lanes_detected > 1;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 3: 
			hit = !(rr || cc || ll) && lanes_detected < 7; 
		//     if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 4:
			hit = cc && lanes_detected > 1  || rr && lanes_detected > 1 && lanes_detected < 7;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 5:
			hit = !(cc || rr || ll) || lanes_detected > 2 ;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 6:
			hit = rr && lanes_detected > 1;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 7:
			hit = lanes_detected == 4 || lanes_detected == 2 || lanes_detected == 6;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 8:
			hit = lanes_detected == 0;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
			
	}

	return hit;
}

std_msgs::Float32MultiArray conv(std_msgs::Float32MultiArray p)
{
	std_msgs::Float32MultiArray q;

	for (int i = 0; i < NUM_STATES; ++i)
	{	
		//ROS_INFO_STREAM("-------------------------" << i << "------------------------"); 
		if (p.data[i] < 0.001)
		{
			q.data.push_back(0.001);
		} else {
			bool hit = det_hit(i);
			 //ROS_INFO_STREAM(p.data[i] << " ==> " << p.data[i] * (hit * p_hit + (1-hit) * p_miss));
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



std_msgs::Float32MultiArray sense(std_msgs::Float32MultiArray prob)
{
	std_msgs::Float32MultiArray q;

	// bool hit = des_state == det_actual_state();

	q = conv(prob);
	
	return q;
}

// int comb(int n, int r)
// {
// 	int i, j;
// 	int row[101];
// 	row[0] = 1;
// 	for (int i = 1; i <= n; ++i)
// 	{
// 		for (int j = i; j > 0; j--)
// 		{
// 			row[j] += row[ j - 1];
// 		}
// 		/* code */
// 	}

// 	return row[r];
// }

int combinaciones( int n, int r )
{
//     unsigned long long
// choose(unsigned long long n, unsigned long long k) {
	//ROS_INFO_STREAM("At Comnbs");
 	double cnm = 1;
 	if (n < r) return 0;
    long long f[n + 1];
    f[0] = 1;
    for (int i = 1; i <= n; i++)
        f[i] = i * f[i - 1];
    // return f[n] / f[r] / f[n - r];
    long long res = f[n] / f[r] / f[n - r];
    // ROS_INFO_STREAM(n <<"C" << r << " = " << res );
	
	//ROS_INFO_STREAM("v: " <<v);
    return (int)res;
// }
}

float det_prob(int edo_ini, int ctrl_action, int edo_fin)
{
	double prob = -1;
	
	int n = NUM_STATES, k = edo_ini;
	float p_bin = -1;

	// ctrl_action = (90 - ctrl_action); //Might be neccesary to fix steering direction

	int sig_edo = 90 / tan(ctrl_action * HEIGHT * M_PI / 180);
	sig_edo = edo_fin + sig_edo / 10;

	p_bin = (sig_edo ) / (float)(NUM_STATES) ;


	if(edo_ini == 6)
	{
		ROS_INFO_STREAM("n: " << n << " k: " << k << " p_bin: " << p_bin << " p_bin**k: " << pow(p_bin, k) << " 1-p_bin**(n-k): " << pow(1-p_bin, (n-k)) << " nCk: " << combinaciones(n,k));
		ROS_INFO_STREAM("prob: " << combinaciones(n,k) * pow(p_bin, k) * pow(1-p_bin, (n-k)) );
	}
	prob = combinaciones(n,k) * pow(p_bin, k) * pow(1-p_bin, (n-k));
	

	return prob;
}

std_msgs::Float32MultiArray move(std_msgs::Float32MultiArray prob)
{
	std_msgs::Float32MultiArray q;
	for (int i = 0; i < NUM_STATES; ++i)
	{
		q.data.push_back(0);
	}
	for (int edo_fin = 0; edo_fin < NUM_STATES; ++edo_fin)
	{
		for (int edo_ini = 0; edo_ini < NUM_STATES; ++edo_ini)
		{
			//if(edo_ini == 0) ROS_INFO_STREAM("det_prob: " <<  det_prob(edo_ini, ctrl_action, edo_fin));
			// combinaciones(edo_ini, edo_fin);
			//if(edo_ini == 6) ROS_INFO_STREAM("edo_ini " << edo_ini << " probs: " << det_prob(edo_ini, ctrl_action, edo_fin));
			q.data[edo_fin] += prob.data[edo_ini] * det_prob(edo_ini, ctrl_action, edo_fin);
		}
		// ROS_INFO_STREAM("[" << q.data[0] << "," << q.data[1] << "," << q.data[2] << ","<< q.data[3] << ","<< q.data[4] << ","<< q.data[5] << "," << q.data[6] << "," << q.data[7] << ","<< q.data[8] << "]");
		
	}

	 return q;
}

void print_state_order()
{
	
	std_msgs::Float32MultiArray order;
	float max = 0, max_ant = 2;
	int i_max = -1;
	std::string str;
	std::stringstream ss;
//ss << a;
//string str = ss.str();
	for (int i = 0; i < NUM_STATES; ++i)
	{
		order.data.push_back(p.data[i]);
	}

	
	for (int i = 0; i < NUM_STATES; ++i)
	{
	    //ROS_INFO_STREAM("[" << p.data[0] << "," << p.data[1] << "," << p.data[2] << ","<< p.data[3] << ","<< p.data[4] << ","<< p.data[5] << "," << p.data[6] << "," << p.data[7] << ","<< p.data[8] << "]");
		for (int j = 0; j < NUM_STATES; ++j)
		{
			if (order.data[j] <=1 && max <= order.data[j])
			{
				max = order.data[j];
				i_max = j;
			}	
		}
		order.data[i_max] = 2;
		//max_ant = max;
		ss << i_max;
		ss << "\t";
		max = 0;
	}
	ROS_INFO_STREAM("Order: " << ss.str() ) ; 

}

int main(int argc, char** argv){
	ros::init(argc, argv, "lane_states_node");
	ROS_INFO_STREAM("lane_states_node initialized");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");
	ros::Rate loop_rate(rate_hz);
	
	std::string node_name = ros::this_node::getName();
        ROS_INFO_STREAM("Getting parameters");
        priv_nh_.param<float>(node_name+"/alpha", alpha,12);

	// ROS_INFO_STREAM("First prob: " << 1.0/(float)NUM_STATES);
	for (int i = 0; i < NUM_STATES; ++i)
	{
		p.data.push_back((float) (1/(float)NUM_STATES));
		std::cout << prob_dist_edo[i];
	}
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("Array initialization: \n" << p);

	pub_loc = nh.advertise<std_msgs::Float32MultiArray>("/localization_array", rate_hz);

	ros::Subscriber sub_pts_left = nh.subscribe("/points/left",1, get_pts_left);
	ros::Subscriber sub_pts_center = nh.subscribe("/points/center",1, get_pts_center);
	ros::Subscriber sub_pts_right = nh.subscribe("/points/right",1, get_pts_right);
	ros::Subscriber sub_mov = nh.subscribe("/manual_control/steering",1,get_ctrl_action);
	ros::Subscriber sub_des_state = nh.subscribe("/desire_state",1, get_des_state);
	
	while(nh.ok())
	{
		L=0;
		R=0;
		C=0;

	    ros::spinOnce();
	    p = sense(p);
	    ROS_INFO_STREAM("Arr after sensed");
		ROS_INFO_STREAM("[" << p.data[0] << "," << p.data[1] << "," << p.data[2] << ","<< p.data[3] << ","<< p.data[4] << ","<< p.data[5] << "," << p.data[6] << "," << p.data[7] << ","<< p.data[8] << "]");
		ROS_INFO_STREAM("Applying movement: ");
	    p = move(p);
	    ROS_INFO_STREAM("[" << p.data[0] << "," << p.data[1] << "," << p.data[2] << ","<< p.data[3] << ","<< p.data[4] << ","<< p.data[5] << "," << p.data[6] << "," << p.data[7] << ","<< p.data[8] << "]");
		
	    pub_loc.publish(p);

	    print_state_order();
	    

	    movement = 0;

	    loop_rate.sleep();
	}
	return 0;
};

