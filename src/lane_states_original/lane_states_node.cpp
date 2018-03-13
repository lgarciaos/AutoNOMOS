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

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

static const uint32_t MY_ROS_QUEUE_SIZE = 1;
#define NUM_STATES 7
#define STATE_WIDTH 22
#define RADIO 7
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

// std::string nombre_estado [9] = {"DNL",   "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR", "DNR"};
std::string nombre_estado [NUM_STATES] = { "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR"};

// movement
float p_exact = 0.7;
float p_undershoot = 0.1;
float p_undershoot_2 = 0.05;
float p_overshoot = 0.1;
float p_overshoot_2 = 0.05;

// sensor
float p_hit = 0.95;
float p_miss = 0.05;

int car_center = 80; //TODO
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

bool rr;
bool cc;
bool ll;

int inc_color = 0;

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
	// negative speed is forward
	// it is not required to know if the car is moving forward or backward, positive for simplification
	ctrl_action = val.angular.z;
	speed = sqrt(val.linear.x * val.linear.x);
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
			
			break;
		case 2: //LC
			// posibles combinaciones:
			// no esta cerca de ninguna linea
			// 2L carro entre C y R
			// 3L carro entre L y C


			// hit = !(rr || cc ) && (lanes_detected == 3); 
			if (R > 0 || C > 0 || R > 0)
				hit = !(ll || cc || rr) && ((lanes_detected == 3 && car_center > arr_center.cells[0].x) ||
			 							(lanes_detected == 7 && car_center < arr_center.cells[0].x)) ;

			break;
		case 3: //CC
			// posibles combinaciones:
			// cerca de C R
			// 2L carro sobre C o R
			// 3L carro sobre C

			hit = ( cc || rr ) && ((lanes_detected == 3) ||
			 					   (lanes_detected == 7 && cc));

			break;
		case 4: //RC
			// posibles combinaciones:
			// no esta cerca de ninguna linea
			// 2L carro entre C y R
			// 3L carro entre C y R

			if ( C > 0 )
				hit = !(ll || cc || rr) && 
					car_center > arr_center.cells[0].x && 
					( lanes_detected == 3 || lanes_detected == 7 ) ;

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
	// multiplied by 100 to convert meters to cm (last 60) 
	// adjusted to 40 for better reflection of real motion
	// TODO conversion from pixels to cm
	double dist_x = speed * 60 * cos(PI/2 - ctrl_action); //*;
	
	int U=dist_x;

	std_msgs::Float32MultiArray q;
	ROS_INFO_STREAM("Control: " << ctrl_action << ", U: " << U << ", dist_x: " << dist_x);
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; i++) {

		double s = 0.0;
		
		//HISTOGRAMA CICLICO
		//EXACT
		int mov = i+U;
		int mod2 = mov;
		
		if (mov < 0)
			mod2 = abs(mov);
		if (mov > (NUM_STATES*STATE_WIDTH - 1))
			mod2 = (NUM_STATES*STATE_WIDTH - 1) - (mov - (NUM_STATES*STATE_WIDTH - 1));
		s = p_exact * prob.data[mod2];
		
		//HISTOGRAMA CICLICLO
		//UNDERSHOOT
		mov = i+U-1;
		mod2 = mov;
		
		if (mov < 0)
			mod2 = abs(mov);
		if (mov > (NUM_STATES*STATE_WIDTH - 1))
			mod2 = (NUM_STATES*STATE_WIDTH - 1) - (mov - (NUM_STATES*STATE_WIDTH - 1));
		s += p_undershoot * prob.data[mod2];

		// UNDERSHOOT 2
		mov = i+U-2;
		mod2 = mov;
		
		if (mov < 0)
			mod2 = abs(mov);
		if (mov > (NUM_STATES*STATE_WIDTH - 1))
			mod2 = (NUM_STATES*STATE_WIDTH - 1) - (mov - (NUM_STATES*STATE_WIDTH - 1));
		s += p_undershoot_2 * prob.data[mod2];
		
		//HISTOGRAMA CICLICLO
		//OVERSHOOT
		mov = i+U+1;
		mod2 = mov;
		
		if (mov < 0)
			mod2 = abs(mov);
		if (mov > (NUM_STATES*STATE_WIDTH - 1))
			mod2 = (NUM_STATES*STATE_WIDTH - 1) - (mov - (NUM_STATES*STATE_WIDTH - 1));
		s += p_overshoot * prob.data[mod2];

		//OVERSHOOT 2
		mov = i+U+1;
		mod2 = mov;
		
		if (mov < 0)
			mod2 = abs(mov);
		if (mov > (NUM_STATES*STATE_WIDTH - 1))
			mod2 = (NUM_STATES*STATE_WIDTH - 1) - (mov - (NUM_STATES*STATE_WIDTH - 1));
		s += p_overshoot_2 * prob.data[mod2];
		
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
	FILE *f = fopen("/home/eduardo/TESIS/git/AutoNOMOS/src/histogramfilter.txt", "a");

	fprintf(f, "%s", type);
	for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i)
	{
		fprintf(f, "\t%.2f ", p.data[i]);
	}
	fprintf(f, "\n");

	fclose(f);
}

void write_to_file(const char* type, float* p, int values) {
	// debug behavior of probabilities using file
	FILE *f = fopen("/home/eduardo/TESIS/git/AutoNOMOS/src/histogramfilter.txt", "a");

	fprintf(f, "%s", type);
	for (int i = 0; i < values; ++i)
	{
		fprintf(f, "\t%.2f ", p[i]);
	}
	fprintf(f, "\n");

	fclose(f);
}

void write_to_image( cv::Mat& imagen, float* hits, std_msgs::Float32MultiArray sense, std_msgs::Float32MultiArray move, int values, int borrarSenseImagen ) {
	// debug behavior of probabilities using rqt
	int hist_height = 100;
	
	for (int i = 0; i < values; ++i)
	{
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
			cv::line(imagen, cv::Point(i + 1, y_height), cv::Point(i + 1, y_height - hist_height), cv::Scalar(0, 0, 0), 1, CV_AA);
	}

	inc_color = (inc_color + 3) % 255;
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

	// write_to_file("init", m);

	pub_loc = nh.advertise<std_msgs::Float32MultiArray>("/localization_array", MY_ROS_QUEUE_SIZE);
	pub_image = nh.advertise<sensor_msgs::Image>("/histogram_states", rate_hz);
	ros::Subscriber sub_pts_left = nh.subscribe("/points/ransac_left", MY_ROS_QUEUE_SIZE, &get_pts_left);
	ros::Subscriber sub_pts_center = nh.subscribe("/points/ransac_center", MY_ROS_QUEUE_SIZE, &get_pts_center);
	ros::Subscriber sub_pts_right = nh.subscribe("/points/ransac_right", MY_ROS_QUEUE_SIZE, &get_pts_right);
	ros::Subscriber sub_des_state = nh.subscribe("/desired_state", MY_ROS_QUEUE_SIZE, &get_ctrl_desired_state);
	ros::Subscriber sub_mov = nh.subscribe("/standarized_vel_ste", MY_ROS_QUEUE_SIZE, &get_ctrl_action);
	
	loop_rate.sleep();
	int estadoEstimado;
	float* hits;
	float* datos = new float[10];

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
	while(nh.ok())
	{
	    ros::spinOnce();
	    hits = det_hits();
	    if(lanes_detected > 0) {
	    	// write_to_file("hits", hits, NUM_STATES*STATE_WIDTH);
	    	s = sense(m, hits);
		} else {
			// mantain previous probabilities if no lines detected
			s = m; 
		}
	    // write_to_file("sense", s);
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
	    // write_to_file("L C R d_ll d_cc d_rr speed ctrl state ctrl_estado", datos, 10);
	    m = move(s);
	    #ifdef PUBLISH_DEBUG_OUTPUT
	    	write_to_image(img_hist, hits, s, m, STATE_WIDTH * NUM_STATES, borrarSenseImagen);
	    	borrarSenseImagen = borrarSenseImagen++ % STATE_WIDTH;
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
	}

	free(hits);
	free(datos);

	return 0;
}