#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <string>
#include <sstream>
#include <cmath>

static const uint32_t MY_ROS_QUEUE_SIZE = 1;
#define NUM_STATES 7
#define STATE_WIDTH 22
#define RADIO 7
#define PI 3.14159265

geometry_msgs::Twist destiny_position;
double rate_hz = 5;
ros::Publisher pub_loc;
ros::Publisher pub_lidar;
std::string nombre;

nav_msgs::GridCells arr_left; 
nav_msgs::GridCells arr_center;
nav_msgs::GridCells arr_right;

// std::string nombre_estado [9] = {"DNL",   "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR", "DNR"};
std::string nombre_estado [NUM_STATES] = { "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR"};


// movement
float p_exact = 0.8;
float p_undershoot = 0.1;
float p_overshoot = 0.1;
//float p_overshoot_2 = .05;

// sensor
float p_hit = 0.95;
float p_miss = 0.05;

// distance to lines
float dist_lines = 12; //TODO

int width_center = 80; //TODO
float speed = 0;

//int movement = 45;
// PERCEPCION DE LIDAR

int L = 0;
int C = 0;
int R = 0;
int des_state = 5;
float ctrl_action = 0;
float ctrl_estado = 0;
int pixeles_cambio_estado=0;

int lanes_detected = 0;

float dist_sensado_rr;
float dist_sensado_cc;
float dist_sensado_ll;

// define if each distance is smaller than dist_lines
bool rr;
bool cc;
bool ll;

// int U = 0;
// double angulo_real;

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

// reads speed and steering from standarized topic
void get_ctrl_action(const geometry_msgs::Twist& val) {
	// negative is forward
	ctrl_action = val.angular.z;
	speed = sqrt(val.linear.x * val.linear.x);
	// printf("\n vel: %.2f ", speed);
}

void get_ctrl_desired_state(const std_msgs::Float64& val) {
    ctrl_estado = val.data;
}

//calculates the distance in pixels. NOTE: only using th x component because Y is asumed constant, maybe isnt the best way to have it.
//If y is asumed constant ==> using abs() instead of sqrt might be more efficient
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
int det_hit (int position) {
	int state = (int)floor(position/STATE_WIDTH);
	//Determine the number of lanes seen
	int hit;
	//switch depending on the state to eval
	switch (state)
	{
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
		case 0: //OL
			hit = !(cc || rr ) && (lanes_detected == 1);
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 1: // LL
			hit = (cc || rr ) && (lanes_detected >= 1 && lanes_detected <= 3);
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			
			if (hit) {
				double state_center = (state*STATE_WIDTH + (state+1)*STATE_WIDTH) / 2;
				// el supuesto es que solo ve una linea: right or center
				double dist_car_in_state = dist_sensado_rr < dist_sensado_cc ? dist_sensado_rr : dist_sensado_cc;

        		if ((position < state_center - dist_car_in_state - RADIO / 2 || position >  state_center - dist_car_in_state + RADIO / 2) && (position < state_center + dist_car_in_state - RADIO / 2 || position >  state_center + dist_car_in_state + RADIO / 2))
					hit = !hit;
        	}
			
			break;
		case 2: //LC
			hit = !(rr || cc ) && (lanes_detected == 3); 

			// discriminar la posicion con base en la distancia a las lineas
			if (hit) {
				double min_coord = state*STATE_WIDTH;
				double max_coord = (state+1)*STATE_WIDTH;
				// pt_c debe estar a la izquierda, en este caso se cumple
				if (dist_sensado_cc > 0 && dist_sensado_cc < 1000) 
					min_coord += dist_sensado_cc - dist_lines - RADIO;
				// pt_r debe estar a la derecha, en este caso se cumple
				if (dist_sensado_rr > 0 && dist_sensado_rr < 1000) 
					max_coord += -dist_sensado_rr + dist_lines + RADIO;
				// el supuesto es que cc esta a la izquierda y rr a la derecha
				if (position < min_coord || position > max_coord)
					hit = !hit;
			}

			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 3: //CC, no puede estar en el centro viendo solo una linea
			hit = (cc || rr ) && ((lanes_detected == 3) || (lanes_detected >= 5 && lanes_detected <= 7));
			
			if (hit) {
				double state_center = (state*STATE_WIDTH + (state+1)*STATE_WIDTH) / 2;
				// el supuesto es que solo ve una linea: right or center
				double dist_car_in_state = dist_sensado_rr < dist_sensado_cc ? dist_sensado_rr : dist_sensado_cc;

        		if ((position < state_center - dist_car_in_state - RADIO / 2 || position >  state_center - dist_car_in_state + RADIO / 2) && (position < state_center + dist_car_in_state - RADIO / 2 || position >  state_center + dist_car_in_state + RADIO / 2))
					hit = !hit;
        	}
			

			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 4: //RC
			hit = !(cc || rr || ll) && ((lanes_detected == 3) ||  (lanes_detected >= 5 && lanes_detected <= 7)) ;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);

			// discriminar la posicion con base en la distancia a las lineas
			if (hit) {
				double min_coord = state*STATE_WIDTH;
				double max_coord = (state+1)*STATE_WIDTH;
				// pt_c debe estar a la izquierda, en este caso se cumple
				if (dist_sensado_cc > 0 && dist_sensado_cc < 1000) 
					min_coord += dist_sensado_cc - dist_lines - RADIO;
				// pt_r debe estar a la derecha, en este caso se cumple
				if (dist_sensado_rr > 0 && dist_sensado_rr < 1000) 
					max_coord += -dist_sensado_rr + dist_lines + RADIO;

				if (position < min_coord || position > max_coord)
					hit = !hit;
			}
			break;
		case 5: //RR
			hit = (cc || rr ) && ((lanes_detected >= 1 && lanes_detected <= 3) || (lanes_detected >= 5 && lanes_detected <= 7));
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			
			if (hit) {
				double state_center = (state*STATE_WIDTH + (state+1)*STATE_WIDTH) / 2;
				// el supuesto es que solo ve una linea: right or center
				double dist_car_in_state = dist_sensado_rr < dist_sensado_cc ? dist_sensado_rr : dist_sensado_cc;

        		if ((position < state_center - dist_car_in_state - RADIO / 2 || position >  state_center - dist_car_in_state + RADIO / 2) && (position < state_center + dist_car_in_state - RADIO / 2 || position >  state_center + dist_car_in_state + RADIO / 2))
					hit = !hit;
        	}
			
			break;
		case 6: //OR
			hit = !(rr || ll || cc) && (lanes_detected == 2 || lanes_detected == 4 || lanes_detected == 6);
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
	}

	return hit;
}

float* det_hits() {
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
	pt_car.x = width_center;
	pt_car.y = 0;
	pt_car.z = 0;

	// if there are points in the line ==> get the last point (the last point is the closer to the car)
	if ( R > 0 ) pt_r = arr_right.cells[R - 1];
	if ( C > 0 ) pt_c = arr_center.cells[C - 1];
	if ( L > 0 ) pt_l = arr_left.cells[L - 1];
	
	// if there are points in the lines, get the distance between the car and the closest point if not, assign a BIG number
	dist_sensado_rr = R > 0 ?  horizontal_dist(pt_r, pt_car) : 1000;
	dist_sensado_cc = C > 0 ?  horizontal_dist(pt_c, pt_car) : 1000;
	dist_sensado_ll = L > 0 ?  horizontal_dist(pt_l, pt_car) : 1000;
	
	// define if each distance is smaller than dist_lines
	rr = dist_sensado_rr <= dist_lines;
	cc = dist_sensado_cc <= dist_lines;
	ll = dist_sensado_ll <= dist_lines;
	
	
	// based on sensing update probabilities
	
	float* hits = new float[NUM_STATES*STATE_WIDTH];

	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {
		
		int hit = det_hit(i);
		hits[i]=(hit * p_hit + (1-hit) * p_miss);
	}

	return hits;
}

std_msgs::Float32MultiArray sense(std_msgs::Float32MultiArray p, float* hits) {
	std_msgs::Float32MultiArray q;
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {	
		q.data.push_back(p.data[i] * hits[i]);
	}

	const char * format = "z_k =[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n";
  	printf (format, hits[0*STATE_WIDTH],hits[1*STATE_WIDTH],hits[2*STATE_WIDTH],hits[3*STATE_WIDTH],hits[4*STATE_WIDTH],hits[5*STATE_WIDTH],hits[6*STATE_WIDTH]);

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
	//ctrl_action: radians in simulation
	// positive is left

	// a que estado podria llegar basado en velocidad y steering
	// double speed = 0.15; // m/s
	// plus 100 to convert meters to cm
	// adjusted to 40 for better reflection of real motion
	double dist_x = speed * 40 * cos(PI/2 - ctrl_action); //*;
	
	int U=dist_x;
	/* NO ES NECESARIO ESTE CÓDIGO
	if(ctrl_action > 0){
		U=dist_x; //mov izquierda
	} else if (ctrl_action < 0) {
		U=-dist_x; // mov derecha
	}
	*/

	std_msgs::Float32MultiArray q;
	ROS_INFO_STREAM("Control: " << ctrl_action << ", U: " << U << ", dist_x: " << dist_x);
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; i++) {
		double s = 0.0;
		
		//HISTOGRAMA CICLICLO
		//EXACT
		int mov = i+U;		
		int mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
		if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
		s = p_exact * prob.data[mod2];
		
		//HISTOGRAMA CICLICLO
		//UNDERSHOOT
		mov = i+U-1;
		mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
		if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
		s += p_undershoot * prob.data[mod2];
		
		//mov = i+U+mov_over;
		//HISTOGRAMA CICLICLO
		//OVERSHOOT
		mov = i+U+1;
		mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
		if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
		s += p_overshoot * prob.data[mod2];
		
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

int main(int argc, char** argv){
	ros::init(argc, argv, "lane_states_node");
	// ROS_INFO_STREAM("lane_states_node initialized");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");
	ros::Rate loop_rate(rate_hz);
	
	std::string node_name = ros::this_node::getName();
	ROS_INFO_STREAM("Getting parameters");
	priv_nh_.param<float>(node_name+"/dist_lines", dist_lines, 8);
	priv_nh_.param<int>(node_name+"/width_center", width_center, 80);
	// priv_nh_.param<float>(node_name+"/speed", speed,0.1);

	const char * format = "P(x)=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n";
	
	std_msgs::Float32MultiArray m;
	std_msgs::Float32MultiArray s;

	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i)
	{
		// Iniciar con distribucion uniforme
		m.data.push_back((float)(1/(float)(NUM_STATES*STATE_WIDTH)));
		// Iniciar con carril derecho
		//p.data.push_back(bel_RC[i]);
	}


	//const char * format = "P(x)=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n";
	// ROS_INFO_STREAM("Array initialization: ");
  	// printf (format, p.data[1],p.data[4],p.data[7],p.data[10],p.data[13],p.data[16],p.data[19],p.data[22],p.data[25]);
		

	ROS_INFO_STREAM("dist_lines: " << dist_lines);
	ROS_INFO_STREAM("Array initialization: \n" << m);

	pub_loc = nh.advertise<std_msgs::Float32MultiArray>("/localization_array", MY_ROS_QUEUE_SIZE);

	ros::Subscriber sub_pts_left = nh.subscribe("/points/left", MY_ROS_QUEUE_SIZE, &get_pts_left);
	ros::Subscriber sub_pts_center = nh.subscribe("/points/center", MY_ROS_QUEUE_SIZE, &get_pts_center);
	ros::Subscriber sub_pts_right = nh.subscribe("/points/right", MY_ROS_QUEUE_SIZE, &get_pts_right);
	ros::Subscriber sub_des_state = nh.subscribe("/desired_state", MY_ROS_QUEUE_SIZE, &get_ctrl_desired_state);

	// actualizar topico de control
	// /manual_control/steering // real car
	// /autonomos/steer/steer_position_controller/command // simulation
	ros::Subscriber sub_mov = nh.subscribe("/standarized_vel_ste", MY_ROS_QUEUE_SIZE, &get_ctrl_action);
	// ros::Subscriber sub_vel = nh.subscribe("/cmd_vel", 1, get_velocity);

	
	// ros::Subscriber sub_des_state = nh.subscribe("/desire_state",1, get_des_state);
	
	loop_rate.sleep();
	int estadoEstimado;
	float* hits;
	float* datos = new float[10];

	while(nh.ok())
	{

	    ros::spinOnce();
	    ROS_INFO_STREAM("Sensing update");

	    hits = det_hits();
	    if(lanes_detected > 0) {
	    	s = sense(m, hits);
		} else {
			s = m; // mantain previous probabilities
		}

	    printf (format, s.data[0*STATE_WIDTH],s.data[1*STATE_WIDTH],s.data[2*STATE_WIDTH],s.data[3*STATE_WIDTH],s.data[4*STATE_WIDTH],s.data[5*STATE_WIDTH],s.data[6*STATE_WIDTH]);
	    pub_loc.publish(s);

	    int estadoEstimado = actual_state(s);
	    
	    datos[0] = (float)L;
	    datos[1] = (float)C;
	    datos[2] = (float)R;

	    datos[3] = dist_sensado_ll;
	    datos[4] = dist_sensado_cc;
	    datos[5] = dist_sensado_rr;

	    datos[6] = speed;
	    datos[7] = ctrl_action;

	    if(estadoEstimado>=0) {
	    	datos[8] = (float)estadoEstimado;
			printf("%d, %d, %d, %d, %.2f, %.2f, %.2f, %.2f, %.2f, %s\n", 0, L, C, R, dist_sensado_ll, dist_sensado_cc, dist_sensado_rr, speed, ctrl_action, nombre_estado[estadoEstimado].c_str());
	    } else {
	    	datos[8] = -0.0f;
			printf("%d, %d, %d, %d, %.2f, %.2f, %.2f, %.2f, %.2f, %s\n", 0, L, C, R, dist_sensado_ll, dist_sensado_cc, dist_sensado_rr, speed, ctrl_action, "?");
	    }
	    datos[9] = ctrl_estado;
	    
	    ROS_INFO_STREAM("Motion update: ");

	    m = move(s);

	    printf (format, m.data[0*STATE_WIDTH],m.data[1*STATE_WIDTH],m.data[2*STATE_WIDTH],m.data[3*STATE_WIDTH],m.data[4*STATE_WIDTH],m.data[5*STATE_WIDTH],m.data[6*STATE_WIDTH]);
	    loop_rate.sleep();

	    
	}

	delete []hits;
	return 0;
}