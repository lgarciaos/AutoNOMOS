#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>

#define NUM_STATES 7
#define STATE_WIDTH 22
#define RADIO 7

static const uint32_t MY_ROS_QUEUE_SIZE = 1;

double rate_hz = 1;
ros::Publisher pub_path;
ros::Publisher pub_pathxy;
// ros::Publisher pub_lidar;

std::string nombre;
nav_msgs::GridCells arr_lane_model;
std_msgs::Float32MultiArray localizationArray;
nav_msgs::GridCells path_planned;

geometry_msgs::Point pt_to_send;

int estado = -1;
int countEstados = -1;

int Lane_size;
int proj_image_h=0;
int threshold_dist_y=0;
int pixeles_cambio_estado=0;
double nav_velocity_pixels = 0.0;

//std::string nombre_estado [NUM_STATES] = {"Dont Know Left", "Out Left", "Left Left", "Left Center", "Center Center", "Right Center", "Out Right", "Dont Know Right"};
int des_state = 16; //o 5

// estados: 	 NSI,   FI,   CI,   CD,   FD, NSD
void get_pts_lane(const nav_msgs::GridCells& array) {
	arr_lane_model.cells = array.cells;
	arr_lane_model.cell_width = array.cell_width;
	// printf("width %d height %d", array.cell_width, array.cell_height);
	if (array.cell_width > 0 && array.cells[0].x > 0)
		Lane_size = array.cell_width;
	else
		Lane_size=0;
	// lines_sensed = array.cell_width > 0 ?  lines_sensed | 4 : lines_sensed | 0;
}


// NOT YET
//void get_des_state(const std_msgs::Int16& val)
//{
//	des_state = val.data;
//}

void get_localization(const std_msgs::Float32MultiArray& locArray) {
	float max=0;
	for(int i=0;i<NUM_STATES;i++){
	 	if(locArray.data[i]>max){
	 		max=locArray.data[i];
	 	}
	}
	estado = -1; // no se pudo determinar el estado, ya que hay mas de uno posible
	countEstados=0;
	for(int i=NUM_STATES-1;i>=0;i--){
        if(locArray.data[i]==max){
            // ROS_INFO_STREAM("Estas en:" << nombre_estado[i]);
            estado = i;
            countEstados++;
        }
    }

    if (countEstados==1)
    	estado=estado;
    else
    	estado=-1; // no se pudo determinar el estado, ya que hay mas de uno posible
}

double navigation_velocity_pixels() {
	double d = 0.0862*RADIO; //mm, avance de una rueda 
	double velocity = -RPM*d/60; //mm/seg
	double time = 1/rate_hz; //seg
	double distance = velocity*time; // mm
	distance /= 66;
	return distance;
}


void planning() {

	// tengo que enviar el control, (theta, velocidad), basado en el centro del estado en el que estoy @estado y en el que quiero estar @des_state
	// el estado en el que estoy me lo da localization_array
	// el estado en el que quiero estar me lo da una maquina de estados de mayor nivel

	// tengo que estimar la posicion del estado en el que quiero estar utilizando ransac
	// la linea central es la mas confiable, pero deberia poder obtener la coordenada aunque no vea lineas

	// 1/8 mm sub-pixel resolution = 0.125 SR300 Intel
	//double distancia_pixeles =  nav_velocity_pixels; //pixel_res * conversion a cm * dist_cm, un pixel equivale a 66mm

	//if(distancia_pixeles < 10){
	//	distancia_pixeles=10;
	//}	

	//double pix_y = height_y - distancia_pixeles;

	// ROS_INFO_STREAM("Pixeles para sig mov: " << distancia_pixeles << ", y:" << pix_y);

	geometry_msgs::Point pt_est_Actual;
	path_planned.cell_height = 1;
	path_planned.cell_width = 1;
	path_planned.cells.clear();

	// obtener coordenada del estado actual en el futuro, de acuerdo a la velocidad a la que voy
	double X_centro=0;
	int diferencia_x=0;
	bool encontrado = false;
	// ROS_INFO_STREAM("Estado: " << estado);
	int mov_estado_futuro = 0; // para saber si la coordenada que me ayuda a obtener el centro pertenece a otro estado

	// Obtener coordenada del estado actual en el futuro
	if(Lane_size >0){
	for(int i=0; i<Lane_size;i++){
		if(abs(arr_lane_model.cells[i].y - proj_image_h) < threshold_dist_y){
			pt_est_Actual.x = arr_lane_model.cells[i].x;
       		    	pt_est_Actual.y = arr_lane_model.cells[i].y;
            		pt_est_Actual.z = 0;
            		encontrado = true;
            		break;
        	}
	}

	if(encontrado && estado >= 0){

		// calcular posibles COORDENADAS centrales de TODOS LOS ESTADOS

		path_planned.cell_width = NUM_STATES;
		path_planned.cell_height = 1;
		path_planned.cells.clear();
		geometry_msgs::Point pt;
		for(int i=0;i<NUM_STATES;i++){

			// las coordenadas de pt_est_Actual, pertenecen a @estado
			int dif_estados=i-estado;
			int diffEstadoPixeles=pixeles_cambio_estado*dif_estados;

			pt.x=pt_est_Actual.x + diffEstadoPixeles;
			pt.y=pt_est_Actual.y;
			pt.z=0;

			path_planned.cells.push_back(pt);
		}

		pub_path.publish(path_planned);
		// estado fijo carril central 5, @des_state
		pub_pathxy.publish(path_planned.cells[des_state]);
		// ROS_INFO_STREAM("Moving to: (" << path_planned.cells[des_state].x << " , " << path_planned.cells[des_state].y << " )" ) ;
	}
	}
	// obtener el angulo del estado actual al estado deseado
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "lane planning node");
	ROS_INFO_STREAM("lane_planning_node initialized");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");
	

	std::string node_name = ros::this_node::getName();

	priv_nh_.param<int>(node_name+"/RPM", RPM, -30);
	priv_nh_.param<double>(node_name+"/rate", rate_hz, 10.0);
	priv_nh_.param<int>(node_name+"/proj_image_h", proj_image_h, 160);
	priv_nh_.param<int>(node_name+"/threshold_dist_y", threshold_dist_y, 50);
	priv_nh_.param<int>(node_name+"/pixeles_cambio_estado", pixeles_cambio_estado, 33);

	pub_path = nh.advertise<nav_msgs::GridCells>("/planning", MY_ROS_QUEUE_SIZE);
	pub_pathxy = nh.advertise<geometry_msgs::Point>("/planningxy", MY_ROS_QUEUE_SIZE);
	
	ros::Subscriber sub_pts_left = nh.subscribe("/points/lane_model",MY_ROS_QUEUE_SIZE, &get_pts_lane);
	//ros::Subscriber sub_pts_center = nh.subscribe("/points/ransac_center",MY_ROS_QUEUE_SIZE, &get_pts_center);
	//ros::Subscriber sub_pts_right = nh.subscribe("/points/ransac_right",MY_ROS_QUEUE_SIZE, &get_pts_right);

	ros::Subscriber sub_localization = nh.subscribe("/localization_array",MY_ROS_QUEUE_SIZE, &get_localization);
	// ros::Subscriber sub_des_state = nh.subscribe("/planning/desire_state",MY_ROS_QUEUE_SIZE, &get_des_state);

	//nav_velocity_pixels = navigation_velocity_pixels();
	
	ros::Rate loop_rate(rate_hz);
	while(nh.ok())
	{
	    ros::spinOnce();
	    planning();
	    loop_rate.sleep();
	}
	return 0;
}

