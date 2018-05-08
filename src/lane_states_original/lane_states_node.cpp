#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <string>
#include <sstream>
#include <cmath>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "gazebo_msgs/LinkStates.h"
#include "tf/tf.h"

#include <sensor_msgs/Imu.h>

static const uint32_t MY_ROS_QUEUE_SIZE = 1;
#define NUM_STATES 7
#define STATE_WIDTH 22
#define RADIO 15
#define PI 3.14159265

#define PRINT_OUTPUT 
#define PUBLISH_DEBUG_OUTPUT 

geometry_msgs::Twist destiny_position;
double rate_hz = 5;
ros::Publisher pub_loc;
ros::Publisher pub_lidar;
ros::Publisher pub_image;
std::string nombre;

nav_msgs::GridCells arr_left; 
nav_msgs::GridCells arr_center;
nav_msgs::GridCells arr_right;

geometry_msgs::Twist car_global_pose;

// std::string nombre_estado [9] = {"DNL",   "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR", "DNR"};
std::string nombre_estado [NUM_STATES] = { "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR"};

// movement
float p_exact = 0.7;
float p_undershoot = 0.15;
float p_undershoot_2 = 0.05;
float p_overshoot = 0.15;
float p_overshoot_2 = 0.05;

// sensor
float p_hit = 0.99;
float p_miss = 0.01;

int car_center = 80; //TODO
float actual_speed = 0;
float car_orientation = 0;

//int movement = 45;
// PERCEPCION DE LIDAR

int L = 0;
int C = 0;
int R = 0;
int des_state = 5;
float actual_steering = 0;
float ctrl_estado = 0;
double dist_x_steering = 0;
double pix_x_prob = 0;
int pixeles_cambio_estado=0;

double odom_x = 0;
double odom_y = 0;
double odom_theta = 0;

int lanes_detected = 0;

float dist_sensado_rr;
float dist_sensado_cc;
float dist_sensado_ll;

float linear_acc_x;
float linear_acc_y;
float linear_acc_z;

bool rr;
bool cc;
bool ll;

int inc_color = 0;

// imu vars
signed long accx[2], accy[2];
signed long velocityx[2], velocityy[2];
signed long positionx[2], positiony[2];
// unsigned char near direction;

unsigned int contador = 0;

//gets the left points
void get_pts_left(const nav_msgs::GridCells& array) {
	arr_left.cells = array.cells;
	if (array.cell_width > 5 && array.cells[0].x > 0) {
		L = array.cell_width;
	}
	else {
		L=0;
	}
}

//gets the center points
void get_pts_center(const nav_msgs::GridCells& array) {
	arr_center.cells = array.cells;
	if (array.cell_width > 5 && array.cells[0].x > 0) {
		C = array.cell_width;
	}
	else {
		C=0;
	}
}

//gets the right points
void get_pts_right(const nav_msgs::GridCells& array) {
	arr_right.cells = array.cells;
	if (array.cell_width > 5 && array.cells[0].x > 0) {
		R = array.cell_width;
	}
	else {
		R=0;
	}
}

/* reads actual_speed and steering from standarized topic
void get_ctrl_action(const geometry_msgs::Twist& val) {
	// negative actual_speed is forward
	// it is not required to know if the car is moving forward or backward, positive for simplification
	actual_steering = val.angular.z;
	actual_speed = sqrt(val.linear.x * val.linear.x);
}
*/

void get_ctrl_desired_state(const std_msgs::Float64& val) {
    ctrl_estado = val.data;
}

// get global orientation
void get_car_orientation(const std_msgs::Float32& val) {
	car_orientation = val.data;
}

// get global pose of car
void poseCallback(const gazebo_msgs::LinkStates& msg){
	// msg.name 
	// TODO search by car name
	car_global_pose.linear.x = msg.pose[2].position.x;
	car_global_pose.linear.y = msg.pose[2].position.y;
	double yaw = tf::getYaw(msg.pose[2].orientation);
	car_global_pose.angular.z = yaw;
}

void get_imu(const sensor_msgs::Imu& val){

	/*
	std_msgs/Header header
	geometry_msgs/Quaternion orientation
	float64[9] orientation_covariance
	geometry_msgs/Vector3 angular_velocity
	float64[9] angular_velocity_covariance
	geometry_msgs/Vector3 linear_acceleration
	float64[9] linear_acceleration_covariance
	*/

	// linear_acc_x = val.linear_acceleration.x;
	// linear_acc_y = val.linear_acceleration.y;
	// linear_acc_z = val.linear_acceleration.z;

	accy[0] = accy[1];
	accy[1] = val.linear_acceleration.y;

	printf("\n contador imu %d \n", contador);
}

// Calculates the distance in pixels. NOTE: only using th x component because Y is asumed constant, 
// maybe isnt the best way to have it.
// If y is asumed constant ==> using abs() instead of sqrt might be more efficient
float horizontal_dist(geometry_msgs::Point p1, geometry_msgs::Point p2) {
	float dif_x = p1.x - p2.x;
	return sqrt(dif_x * dif_x); // absolute value
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
//		 |     |     |			
//		 |           |			OL -> Out left
//		 |     |     |			LL -> Left Left
//		 |           |			LC -> Left Center
//		 |	   |     |			CC -> Center Center
//		 |           |			RC -> Right Center
//		 |     |     |			RR -> Right Right
//   OL LL LC CC RC RR OR		OR -> Out Right
//   0	1  2  3  4  5  6 
//	
//
//Determine the probable position of the car according to what is seen
int det_hit (int position) {
	int state = (int)floor(position/STATE_WIDTH);
	int hit = 0;
	switch (state)
	{
		// con el cambio a dbscan aumenta el sensado y disminuyen el numero de 
		// combinaciones de deteccion a: 
		// con el problema de que R no siempre estará a la derecha del carro

		//   | 	L  |  C  |  R
		//===================== 
		// 0 |  0  |  0  |  0
		// 1 |  0  |  0  |  1
		// 3 |  0  |  1  |  1
		// 7 |  1  |  1  |  1


		case 0: //OL
			// posibles combinaciones:
			// no esta cerca de ninguna linea
			// 1L y el carro está a la izquierda de R
			// 2L y el carro está a la izquierda de C
			// 3L y el carro está a la izquierda de L


			if (R > 0 || C > 0 || R > 0)
				hit = !(ll || cc || rr ) && ((lanes_detected == 1 && car_center < arr_right.cells[0].x) ||
										 (lanes_detected == 3 && car_center < arr_center.cells[0].x) ||
										 (lanes_detected == 7 && car_center < arr_left.cells[0].x) );
			break;
		case 1: // LL
			// posibles combinaciones:
			// cerca de L C ó R
			// 1L carro sobre R
			// 2L carro sobre C
			// 3L carro sobre L

			hit = (ll || cc || rr ) && ((lanes_detected == 1 && rr) ||
										(lanes_detected == 3 && cc) ||
										(lanes_detected == 7 && ll));

			if (hit) {
				double state_center = (state * STATE_WIDTH + (state+1) * STATE_WIDTH) / 2;
				// el supuesto es que solo ve una linea: right or center
				double dist_car_in_state = dist_sensado_rr < dist_sensado_cc ? dist_sensado_rr : dist_sensado_cc;
				dist_car_in_state = dist_car_in_state < dist_sensado_ll ? dist_car_in_state : dist_sensado_ll;

	        	if ((position < state_center - dist_car_in_state - RADIO / 2 || position >  state_center - dist_car_in_state + RADIO / 2) && (position < state_center + dist_car_in_state - RADIO / 2 || position >  state_center + dist_car_in_state + RADIO / 2))
					hit = !hit;
			}
			
			break;

			// FALTA APROVECHAR INFORMACION DE SENSADO 

		case 2: //LC
			// posibles combinaciones:
			// no esta cerca de ninguna linea
			// 2L carro entre C y R
			// 3L carro entre L y C

			// hit = !(rr || cc ) && (lanes_detected == 3); 
			if (R > 0 || C > 0 || R > 0) {
				hit = !(ll || cc || rr) && ((lanes_detected == 3 && car_center > arr_center.cells[0].x) ||
			 								(lanes_detected == 7 && car_center < arr_center.cells[0].x));

				
				if (hit) {
					double min_coord = state * STATE_WIDTH;
					double max_coord = (state + 1) * STATE_WIDTH;
					double temp_min, temp_max;

					if ( car_center > arr_center.cells[0].x ) {
						// pt_c debe estar a la izquierda, en este caso se cumple
						if (dist_sensado_cc > 0 && dist_sensado_cc < 1000)
							temp_min = min_coord + dist_sensado_cc - STATE_WIDTH / 2 - RADIO;
						// pt_r debe estar a la derecha, en este caso se cumple
						if (dist_sensado_rr > 0 && dist_sensado_rr < 1000)
							temp_max = max_coord + -dist_sensado_rr + STATE_WIDTH / 2 + RADIO;
					} else {
						// pt_c debe estar a la izquierda, en este caso se cumple
						if (dist_sensado_cc > 0 && dist_sensado_cc < 1000)
							temp_min = min_coord + dist_sensado_ll - STATE_WIDTH / 2 - RADIO;
						// pt_r debe estar a la derecha, en este caso se cumple
						if (dist_sensado_rr > 0 && dist_sensado_rr < 1000)
							temp_max = max_coord + -dist_sensado_cc + STATE_WIDTH / 2 + RADIO;
					}
					if (temp_min > min_coord) {
						if (temp_min > max_coord - RADIO)
							min_coord =  max_coord - RADIO;
						else
							min_coord = temp_min;
					}
					if (temp_max < max_coord) {
						if (temp_max < min_coord + RADIO)
							max_coord = min_coord + RADIO;
						else
							max_coord = temp_max;
					}


					if (position < min_coord || position > max_coord)
						hit = !hit;
				}
				
			}

			break;
		case 3: //CC
			// posibles combinaciones:
			// cerca de C R
			// 2L carro sobre C o R
			// 3L carro sobre C

			hit = ( cc || rr ) && ((lanes_detected == 3) ||
			 					   (lanes_detected == 7 && cc));

			if (hit) {
				double state_center = (state*STATE_WIDTH + (state+1)*STATE_WIDTH) / 2;
				// el supuesto es que solo ve una linea: right or center
				double dist_car_in_state = dist_sensado_rr < dist_sensado_cc ? dist_sensado_rr : dist_sensado_cc;

	        	if ((position < state_center - dist_car_in_state - RADIO / 2 || 
	        		position >  state_center - dist_car_in_state + RADIO / 2) && 
	        		(position < state_center + dist_car_in_state - RADIO / 2 || 
	        		position >  state_center + dist_car_in_state + RADIO / 2))
					hit = !hit;
			}

			break;
		case 4: //RC
			// posibles combinaciones:
			// no esta cerca de ninguna linea
			// 2L carro entre C y R
			// 3L carro entre C y R

			if ( C > 0 ) {
				hit = !(ll || cc || rr) && 
					car_center > arr_center.cells[0].x && 
					( lanes_detected == 3 || lanes_detected == 7 );

				
				if (hit) {
					double min_coord = state * STATE_WIDTH;
					double max_coord = (state + 1) * STATE_WIDTH;
					double temp;

					// pt_c debe estar a la izquierda, en este caso se cumple
					if (dist_sensado_cc > 0 && dist_sensado_cc < 1000) {
						temp = min_coord + dist_sensado_cc - STATE_WIDTH / 2 - RADIO;
						if (temp > min_coord) {
							if (temp > max_coord - RADIO)
								min_coord =  max_coord - RADIO;
							else
								min_coord = temp;
						}
					}

					// pt_r debe estar a la derecha, en este caso se cumple
					if (dist_sensado_rr > 0 && dist_sensado_rr < 1000) {
						temp = max_coord + -dist_sensado_rr + STATE_WIDTH / 2 + RADIO;
						if (temp < max_coord) {
							if (temp < min_coord + RADIO)
								max_coord = min_coord + RADIO;
							else
								max_coord = temp;
						}
					}

					if (position < min_coord || position > max_coord)
						hit = !hit;
				}
				
			}

			break;
		case 5: //RR
			// posibles combinaciones:
			// cerca de R
			// 1L carro sobre R
			// 2L carro sobre R
			// 3L carro sobre R

			hit = rr && (lanes_detected == 1 || 
						 lanes_detected == 3 || 
						 lanes_detected == 7);

			if (hit) {
				if (hit) {
					double state_center = (state * STATE_WIDTH + (state + 1) * STATE_WIDTH) / 2;
					// el supuesto es que solo ve una linea: right or center
					double dist_car_in_state = dist_sensado_rr;

		        	if ((position < state_center - dist_car_in_state - RADIO / 2 || position >  state_center - dist_car_in_state + RADIO / 2) && (position < state_center + dist_car_in_state - RADIO / 2 || position >  state_center + dist_car_in_state + RADIO / 2))
						hit = !hit;
				}
			}
			
			break;
		case 6: //OR
			// posibles combinaciones:
			// no esta cerca de ninguna linea
			// 1L carro a la derecha de R
			// 2L carro a la derecha de R
			// 3L carro a la derecha de R

			if ( R > 0 )
				hit = !(ll || cc || rr ) &&
				   car_center > arr_right.cells[0].x && (lanes_detected == 1 ||
										 				  lanes_detected == 3 ||
										 				  lanes_detected == 7 );

			break;
	}

	return hit;
}

float* det_hits() {
	// TODO: Hay una idea pendiente, la cual es que el sensado nos puede dar información de los estados 
	// aunque no estemos en ese estado, si podemos saber que nos estamos acercado
	// p.ej. estamos en RC y nos movemos a la izquierda sabemos que estamos en los limites de CC

	lanes_detected = (L > 0);
	lanes_detected = lanes_detected << 1;
	lanes_detected = lanes_detected | C > 0;
	lanes_detected = lanes_detected << 1;
	lanes_detected = lanes_detected | R > 0;
	
	geometry_msgs::Point pt_r;
	geometry_msgs::Point pt_c;
	geometry_msgs::Point pt_l;
	geometry_msgs::Point pt_car;
	
	//define the static point (center) of the car
	pt_car.x = car_center;
	pt_car.y = 0;
	pt_car.z = 0;
	
	// first points are closer to the car using ransac
	if ( R > 0 ) pt_r = arr_right.cells[0];
	if ( C > 0 ) pt_c = arr_center.cells[0];
	if ( L > 0 ) pt_l = arr_left.cells[0];
	
	// if there are points in the lines, get the distance between the car and the closest point if not, assign a BIG number
	dist_sensado_rr = R > 0 ? horizontal_dist(pt_r, pt_car) : 1000;
	dist_sensado_cc = C > 0 ? horizontal_dist(pt_c, pt_car) : 1000;
	dist_sensado_ll = L > 0 ? horizontal_dist(pt_l, pt_car) : 1000;
	
	// define if each distance is smaller than dist_lines
	rr = dist_sensado_rr <= STATE_WIDTH / 2;
	cc = dist_sensado_cc <= STATE_WIDTH / 2;
	ll = dist_sensado_ll <= STATE_WIDTH / 2;
	
	// based on sensing update probabilities
	float* hits = new float[NUM_STATES*STATE_WIDTH];
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {
		unsigned int hit = det_hit(i);
		hits[i] = (hit * p_hit + (1-hit) * p_miss);
	}

	return hits;
}

std_msgs::Float32MultiArray sense(std_msgs::Float32MultiArray p, float* hits) {
	std_msgs::Float32MultiArray q;
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {	
		q.data.push_back(p.data[i] * hits[i]);
	}

	// normalizacion
	float sum = 0;
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {
		sum += q.data[i];
	}
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {
		q.data[i] /= sum;
	}
	return q;
}

std_msgs::Float32MultiArray move(std_msgs::Float32MultiArray prob) {
	//actual_steering: radians in simulation
	// positive is left

	// a que estado podria llegar basado en velocidad y steering
	// double actual_speed = 0.15; // m/s
	// multiplied by 100 to convert meters to cm (last 60) 
	// adjusted to 40 for better reflection of real motion
	// TODO conversion from pixels to cm

	/* EQUATIONS with respect to car frame
	R = ℓ / sin(α) (when steering angle is not 0)
	φ = V * dt / R
	dx = R * sin(φ)
	dy = (R - R * cos(φ)) (for left turn)
	dy = -(R - R * cos(φ)) (for right turn)
	dθ = V / ℓ * sin(α) * dt
	x = x + dx * cos(θ) - dy * sin(θ)
	y = y + dx * sin(θ) + dy * cos(θ)
	θ = θ + dθ
	vx = dx / dt
	vy = dy / dt
	vθ = dθ / dt
	*/

	/*
	double l = 0.3;
	double alpha = actual_steering;
	double dt = 1 / rate_hz;
	double V = actual_speed;

	if (alpha != 0) {
		double R = l / sin(alpha);
		double phi = V * dt / R; // m/s 
		double dx = (R * sin(phi));
		double dy = alpha < 0 ? (R - R * cos(phi)) : -(R - R * cos(phi));
		// double dy = alpha >= 0 ? (R - R * cos(phi)) : -(R - R * cos(phi));
		// double dy = (R - R * cos(phi));
		double dtheta = V / l * sin(alpha) * dt;
		odom_x = odom_x + dx * cos(odom_theta) - dy * sin(odom_theta);
		double odom_y_previo = odom_y;
		odom_y = odom_y + dx * sin(odom_theta) + dy * cos(odom_theta);
		odom_theta = odom_theta + dtheta;
		double vx = dx / dt;
		double vy = dy / dt;
		double vtheta = dtheta / dt;
		printf("\n odomx: %.2f, odomy: %.2f, odomtheta: %.2f", odom_x, odom_y, odom_theta);

		// x con respecto al marco de referencia global, y con respecto al marco del carro
		dist_x_steering = odom_y - odom_y_previo; //actual_speed * sin(actual_steering); // antes cos(car_orientation - actual_steering)
	} else {
		dist_x_steering = 0;
	}

	pix_x_prob = dist_x_steering / 0.006879221;
	*/

	printf("\n contador move %d \n", ++contador);

	float T = 1 / 10; // rate of imu topic
    // first integration
    // vel_x.append(vel_x[i - 1] + ( acc_x[i] + (acc_x[i] - acc_x[i - 1]) / 2 ) * T)
    velocityy[0] = velocityy[1];
	velocityy[1] = velocityy[0] + (accy[1] + (accy[1] - accy[0]) / 2) * T; 
    // second integration
	// pos_x.append(pos_x[i - 1] + ( vel_x[i] + (vel_x[i] - vel_x[i - 1]) / 2 ) * T) 
    positiony[0] = positiony[1];
    positiony[1] = positiony[0] + (velocityy[1] + (velocityy[1] - velocityy[0]) / 2) * T;
    

	// tamaño de celda en pixeles: 0.006879221
    // negativo por la rotacion de cuadro de referencia con respecto al carro
	float rate_pos = -(positiony[1] - positiony[0]) / 0.006879221;
	int U = round( rate_pos );

	std_msgs::Float32MultiArray q;
	// ROS_INFO_STREAM("Orientation: " << car_orientation << ", Control: " << actual_steering << ", U: " << U << ", dist_x_steering: " << dist_x_steering);
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; i++) {

		double s = 0.0;
		
		//HISTOGRAMA CICLICO
		//EXACT
		int mov = i + U;
		int mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
		if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
		s = p_exact * prob.data[mod2];
		
		//HISTOGRAMA CICLICLO
		//UNDERSHOOT
		mov = i + U - 1;
		mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
		if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
		s += p_undershoot * prob.data[mod2];

		// UNDERSHOOT 2
		/*
		mov = i + U - 2;
		mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
		if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
		s += p_undershoot_2 * prob.data[mod2];
		*/

		//HISTOGRAMA CICLICLO
		//OVERSHOOT
		mov = i + U + 1;
		mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
		if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
		s += p_overshoot * prob.data[mod2];

		//OVERSHOOT 2
		/*
		mov = i + U + 1;
		mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
		if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
		s += p_overshoot_2 * prob.data[mod2];
		*/

		q.data.push_back(s);

	}
	return q;
}

int actual_state(std_msgs::Float32MultiArray locArray) {
	float max=0;
	for (int i=0;i<NUM_STATES*STATE_WIDTH;i++) {
	 	if(locArray.data[i]>max){
	 		max=locArray.data[i];
	 	}
	}

	int countStates=0;
	int state = -1;
	for (int i=NUM_STATES*STATE_WIDTH-1;i>=0;i--) {
        if(locArray.data[i]==max){
        	int temp_state = (int)floor(i/STATE_WIDTH);
            if (temp_state != state){
	            state = temp_state;
	            countStates++;
        	}
        }
    }

    if (countStates==1)
    	return state;
    else
    	return -1; // no se pudo determinar el estado, ya que hay mas de uno posible
}

void write_to_file(const char* type, std_msgs::Float32MultiArray p) {
	// debug behavior of probabilities using file
	FILE *f = fopen("~/git/AutoNOMOS/src/histogramfilter.txt", "a");

	fprintf(f, "%s", type);
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {
		fprintf(f, "\t%.2f ", p.data[i]);
	}
	fprintf(f, "\n");

	fclose(f);
}

void write_to_file(const char* type, float* p, int values) {
	// debug behavior of probabilities using file
	FILE *f = fopen("~/git/AutoNOMOS/src/histogramfilter.txt", "a");

	fprintf(f, "%s", type);
	for (int i = 0; i < values; ++i)
	{
		fprintf(f, "\t%.2f ", p[i]);
	}
	fprintf(f, "\n");

	fclose(f);
}

void write_to_file_headers() {
	// debug behavior of probabilities using file
	FILE *f = fopen("~/git/AutoNOMOS/src/histogramfilter.txt", "w");

	for (int i = 0; i < NUM_STATES * STATE_WIDTH; ++i) {
		fprintf(f, "\tEst_%d ", i);
	}


	fprintf(f, "\t%s", "L"); // 0
	fprintf(f, "\t%s", "C");
	fprintf(f, "\t%s", "R");
	fprintf(f, "\t%s", "d_ll");
	fprintf(f, "\t%s", "d_cc");
	fprintf(f, "\t%s", "d_rr");
	fprintf(f, "\t%s", "actual_speed");
	fprintf(f, "\t%s", "ctrl"); // 7

	fprintf(f, "\t%s", "odom_x");
	fprintf(f, "\t%s", "odom_y"); // 9
	fprintf(f, "\t%s", "odom_theta");

	fprintf(f, "\t%s", "dist_x_prob");
	fprintf(f, "\t%s", "pix_x_prob"); // 12
	fprintf(f, "\t%s", "state");
	fprintf(f, "\t%s", "ctrl_st");
	fprintf(f, "\t%s", "car_or");
	fprintf(f, "\t%s", "pose_x");
	fprintf(f, "\t%s", "pose_y");
	fprintf(f, "\t%s", "pose_yaw");
	fprintf(f, "\t%s", "diff_x_real");
	fprintf(f, "\t%s", "pix_x_real"); // 20


	fprintf(f, "\n");
	fclose(f);
}

void write_to_file(std_msgs::Float32MultiArray m_array, float* p, int values) {
	// debug behavior of probabilities using file
	FILE *f = fopen("~/git/AutoNOMOS/src/histogramfilter.txt", "a");

	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {
		fprintf(f, "\t%.2f ", m_array.data[i]);
	}
	for (int i = 0; i < values; ++i) {
		fprintf(f, "\t%.5f ", p[i]);
	}
	
	fprintf(f, "\n");

	fclose(f);
}

void write_to_image( cv::Mat& imagen, float* hits, std_msgs::Float32MultiArray sense, std_msgs::Float32MultiArray move, int values, int borrarSenseImagen ) {
	// debug behavior of probabilities using rqt
	int hist_height = 100;
	
	for (int i = 0; i < values; ++i) {
		// SENSE
		int y_height = 130;
		if (borrarSenseImagen == 0 && i < values - 1)
			cv::line(imagen, cv::Point(i, y_height), cv::Point(i + 1, y_height - hist_height), cv::Scalar(0, 0, 0), 1, CV_AA);
		cv::line(imagen, cv::Point(i, y_height), cv::Point(i, y_height - sense.data[i] * hist_height), cv::Scalar(0, inc_color, 255 - inc_color), 1, CV_AA);
		
		// HITS
		y_height = 150;
		cv::line(imagen, cv::Point(i, y_height), cv::Point(i, y_height - hits[i] * 10), cv::Scalar(0, 100, 255), 1, CV_AA);
		// borrar hit posicion i + 1 en imagen
		if (i < values - 1)
			cv::line(imagen, cv::Point(i + 1, y_height), cv::Point(i + 1, y_height - 10), cv::Scalar(0, 0, 0), 1, CV_AA);
		
		// MOVE
		y_height = 250;
		cv::line(imagen, cv::Point(i, y_height - sense.data[i] * hist_height), cv::Point(i, y_height - move.data[i] * hist_height), cv::Scalar(0, 200, 255), 1, CV_AA);
		// borrar movimiento posicion i + 1 en imagen
		if (i < values - 1)
			cv::line(imagen, cv::Point(i + 1, y_height), cv::Point(i + 1, y_height - hist_height + 15), cv::Scalar(0, 0, 0), 1, CV_AA);
	}

	inc_color = (inc_color + 7) % 255;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "lane_states_node");
	// ROS_INFO_STREAM("lane_states_node initialized");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");
	ros::Rate loop_rate(rate_hz);
	
	std::string node_name = ros::this_node::getName();
    ROS_INFO_STREAM("Getting parameters");
    priv_nh_.param<int>(node_name+"/car_center", car_center, 80);
	
	const char * format = "P(x)=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n";
	
	std_msgs::Float32MultiArray m;
	std_msgs::Float32MultiArray s;

	// Iniciar con distribucion uniforme
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i)
	{
		m.data.push_back((float)(1/(float)(NUM_STATES*STATE_WIDTH)));
	}

	// write_to_file_headers();
	// write_to_file("init", m);

	pub_loc = nh.advertise<std_msgs::Float32MultiArray>("/localization_array", MY_ROS_QUEUE_SIZE);
	pub_image = nh.advertise<sensor_msgs::Image>("/histogram_states", MY_ROS_QUEUE_SIZE);

	ros::Subscriber sub_pts_left = nh.subscribe("/points/ransac_left", MY_ROS_QUEUE_SIZE, &get_pts_left);
	ros::Subscriber sub_pts_center = nh.subscribe("/points/ransac_center", MY_ROS_QUEUE_SIZE, &get_pts_center);
	ros::Subscriber sub_pts_right = nh.subscribe("/points/ransac_right", MY_ROS_QUEUE_SIZE, &get_pts_right);
	ros::Subscriber sub_des_state = nh.subscribe("/desired_state", MY_ROS_QUEUE_SIZE, &get_ctrl_desired_state);
	
	// no es confiable velocidad y steering
	// ros::Subscriber sub_mov = nh.subscribe("/standarized_vel_ste", MY_ROS_QUEUE_SIZE, &get_ctrl_action);
	// probar con imu
	ros::Subscriber sub_imu = nh.subscribe("/AutoNOMOS_mini/imu", MY_ROS_QUEUE_SIZE, &get_imu);

	ros::Subscriber sub_orientation = nh.subscribe("/car_orientation", MY_ROS_QUEUE_SIZE, &get_car_orientation);

	ros::Subscriber sub_robot_pos = nh.subscribe("/gazebo/model_states", MY_ROS_QUEUE_SIZE, &poseCallback); 
	
	
	int estadoEstimado;
	float* hits;
	int num_datos = 21;
	float* datos = new float[num_datos];

	// publicar imagen con la distribucion de sense, hits y move para debug
	#ifdef PUBLISH_DEBUG_OUTPUT
	
		cv::Mat img_hist(260, 160, CV_8UC3, cv::Scalar(0, 0, 0));
		sensor_msgs::ImagePtr imgmsg;
		
		for (int i = 0; i < NUM_STATES; i++) {
			cv::line(img_hist, cv::Point(i*STATE_WIDTH, 0), cv::Point(i*STATE_WIDTH, 260), cv::Scalar(255, 0, 0), 1, CV_AA);
			cv::putText(img_hist, nombre_estado[i], cv::Point(i*STATE_WIDTH + STATE_WIDTH/4, 10), 0, .35, cv::Scalar(0,255,237));
		}

		cv::putText(img_hist, "hits", cv::Point(0, 160), 0, .35, cv::Scalar(0,221,237));
		cv::putText(img_hist, "sense", cv::Point(0, 138), 0, .35, cv::Scalar(0,221,237));
		cv::putText(img_hist, "move", cv::Point(0, 260), 0, .35, cv::Scalar(0,221,237));
	
	#endif

	int borrarSenseImagen = 0;	
	bool bandera_primer = true;
	while(ros::ok()) {
	    // ros::spinOnce();
	    hits = det_hits();
	    if(lanes_detected > 0) {
	    	// write_to_file("hits", hits, NUM_STATES*STATE_WIDTH);
	    	s = sense(m, hits);
		} else {
			// mantain previous probabilities if no lines detected
			s = m; 
		}
	    
	    pub_loc.publish(s);

	    /*
	    	fprintf(f, "\t%s", "L"); // 0
			fprintf(f, "\t%s", "C");
			fprintf(f, "\t%s", "R");
			fprintf(f, "\t%s", "d_ll");
			fprintf(f, "\t%s", "d_cc");
			fprintf(f, "\t%s", "d_rr");
			fprintf(f, "\t%s", "actual_speed");
			fprintf(f, "\t%s", "ctrl"); // 7
		
			fprintf(f, "\t%s" "odom_x");
			fprintf(f, "\t%s" "odom_y"); // 9
			fprintf(f, "\t%s" "odom_theta");
		
			fprintf(f, "\t%s", "dist_x_prob");
			fprintf(f, "\t%s", "pix_x_prob"); // 12
			fprintf(f, "\t%s", "state");
			fprintf(f, "\t%s", "ctrl_st");
			fprintf(f, "\t%s", "car_or");
			fprintf(f, "\t%s", "pose_x");
			fprintf(f, "\t%s", "pose_y");
			fprintf(f, "\t%s", "pose_yaw");
			fprintf(f, "\t%s", "diff_x_real");
			fprintf(f, "\t%s", "pix_x_real"); // 20
	    */
	    
	    int estadoEstimado = actual_state(s);
	    datos[0] = (float) L;
	    datos[1] = (float) C;
	    datos[2] = (float) R;
	    datos[3] = dist_sensado_ll;
	    datos[4] = dist_sensado_cc;
	    datos[5] = dist_sensado_rr;
	    datos[6] = actual_speed;
	    datos[7] = actual_steering;
	    
	    datos[8] = odom_x;
	    datos[9] = odom_y;
	    datos[10] = odom_theta;
	    
	    datos[11] = dist_x_steering;
	    datos[12] = pix_x_prob;
	    if(estadoEstimado>=0) {
	    	datos[13] = (float)estadoEstimado;
			printf("%d, %d, %d, %d, %.2f, %.2f, %.2f, %.2f, %.2f, %s\n", 0, L, C, R, dist_sensado_ll, dist_sensado_cc, dist_sensado_rr, actual_speed, actual_steering, nombre_estado[estadoEstimado].c_str());
	    } else {
	    	datos[13] = -0.0f;
			printf("%d, %d, %d, %d, %.2f, %.2f, %.2f, %.2f, %.2f, %s\n", 0, L, C, R, dist_sensado_ll, dist_sensado_cc, dist_sensado_rr, actual_speed, actual_steering, "?");
	    }
	    datos[14] = ctrl_estado;
	    datos[15] = odom_theta; // direccion
	    datos[18] = car_global_pose.angular.z;
	    // magnitud de vector
	    if ( !bandera_primer ) {
	    	// magnitud
	    	datos[19] = car_global_pose.linear.x - datos[16];
	    	datos[20] = datos[19] / 0.006879221;
	    } else {
	    	datos[19] = 0;
	    	datos[20] = 0;
	    	bandera_primer = false;
	    }
	    
	    datos[16] = car_global_pose.linear.x;
	    datos[17] = car_global_pose.linear.y;
	    
	    // write_to_file(s, datos, num_datos);
	    
	    // write_to_file("L C R d_ll d_cc d_rr actual_speed ctrl state ctrl_estado orientation", datos, 11);
	    m = move(s);
	    #ifdef PUBLISH_DEBUG_OUTPUT
	    	write_to_image(img_hist, hits, s, m, STATE_WIDTH * NUM_STATES, borrarSenseImagen);
	    	borrarSenseImagen = ++borrarSenseImagen % STATE_WIDTH;
	    #endif
	    printf (format, m.data[0*STATE_WIDTH],m.data[1*STATE_WIDTH],m.data[2*STATE_WIDTH],m.data[3*STATE_WIDTH],m.data[4*STATE_WIDTH],m.data[5*STATE_WIDTH],m.data[6*STATE_WIDTH]);
	    loop_rate.sleep();
	    #ifdef PAINT_OUTPUT
	    	cv::imshow("Localization results", img_hist);
        	cv::waitKey(1);
		#endif
        #ifdef PUBLISH_DEBUG_OUTPUT
	    	imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_hist).toImageMsg();
	 		pub_image.publish(imgmsg);  
	 	#endif 

	 	loop_rate.sleep();
	}

	free(hits);
	free(datos);

	return 0;
}
