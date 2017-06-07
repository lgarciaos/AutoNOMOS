#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>
#include <string>
#include <sstream>
#include <cmath>

static const uint32_t MY_ROS_QUEUE_SIZE = 1;
#define NUM_STATES 9*3
#define RADIO 300

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

std::string nombre_estado [9] = {"DNL",   "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR", "DNR"};


// movement
float p_exact;
float p_undershoot;
float p_undershoot_2;
float p_overshoot;
//float p_overshoot_2 = .05;

// sensor
float p_hit = 0.99;
float p_miss = 0.00;

float alpha = 12; //TODO

int movement = 45;
// PERCEPCION DE LIDAR

int L = 0;
int C = 0;
int R = 0;
int des_state = 5;

int ctrl_action = 45;

double nav_velocity_pixels = 0.0;
int RPM = 0;
int pixeles_cambio_estado=0;

int lanes_detected = 0;

float dist_rr;
float dist_cc;
float dist_ll;

// define if each distance is smaller than alpha
bool rr;
bool cc;
bool ll;

int U = 0;
double angulo_real;

//gets the left points
void get_pts_left(const nav_msgs::GridCells& array)
{
	arr_left.cells = array.cells;
	if (array.cell_width > 5 && array.cells[0].x > 0) {
		L = array.cell_width;
	}
	else {
		L=0;
	}
}
//gets the center points
void get_pts_center(const nav_msgs::GridCells& array)
{
	arr_center.cells = array.cells;
	if (array.cell_width > 5 && array.cells[0].x > 0) {
		C = array.cell_width;
	}
	else {
		C=0;
	}
}

//gets the right points
void get_pts_right(const nav_msgs::GridCells& array)
{
	arr_right.cells = array.cells;
	if (array.cell_width > 5 && array.cells[0].x > 0) {
		R = array.cell_width;
	}
	else {
		R=0;
	}
}

//transforms the motion into values for shift >> used before but maybe not useful anymore (290317)
void get_ctrl_action(const std_msgs::Int16& val)
{
	ctrl_action = val.data;
	// ROS_INFO_STREAM("recibido steering: " << val.data);
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
int det_hit (int state)
{
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
		case 1:	//NSI
			hit = !(rr || cc || ll) && lanes_detected == 0;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 4: //FI
			hit = !(ll || cc || rr ) && (lanes_detected == 1);
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 7:  // LL
			hit = (cc || rr ) && (lanes_detected >= 1 && lanes_detected <= 3);
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 10: //LC
			hit = !(rr || cc || ll) && (lanes_detected == 3); 
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 13: //CC, no puede estar en el centro viendo solo una linea
			hit = (cc || rr ) && ((lanes_detected == 3) || (lanes_detected >= 5 && lanes_detected <= 7));
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 16: //RC
			hit = !(cc || rr || ll) && ((lanes_detected == 3) ||  (lanes_detected >= 5 && lanes_detected <= 7)) ;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 19: //RR
			hit = (cc || rr ) && ((lanes_detected >= 1 && lanes_detected <= 3) || (lanes_detected >= 5 && lanes_detected <= 7));
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 22: //FD
			hit = !(rr || ll || cc) && (lanes_detected == 2 || lanes_detected == 4 || lanes_detected == 6);
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		case 25: //NSD
			hit = !(rr || cc || ll) && lanes_detected == 0;
			// if(hit) ROS_INFO_STREAM("Hit at state: " << state);
			break;
		default:
			hit=0;
	}
	return hit;
}

std_msgs::Float32MultiArray conv(std_msgs::Float32MultiArray p)
{
	lanes_detected = (L > 0);
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
	dist_rr = R > 0 ?  dist(pt_r, pt_car) : 1000;
	dist_cc = C > 0 ?  dist(pt_c, pt_car) : 1000;
	dist_ll = L > 0 ?  dist(pt_l, pt_car) : 1000;
	
	// define if each distance is smaller than alpha
	rr = dist_rr <= alpha;
	cc = dist_cc <= alpha;
	ll = dist_ll <= alpha;

	
	//printing for debugging
	//ROS_INFO_STREAM("rr: " << rr << "\tcc: " << cc << "\tll: " << ll << "\tlanes: " << lanes_detected);
	
	//ROS_INFO_STREAM("Detected R: " << R << ", C: " << C << ", L: "<< L << " -- dist_rr: " << dist_rr << ", dist_cc: " << dist_cc << ", dist_ll: " << dist_ll << ", alpha: " << alpha);

	std_msgs::Float32MultiArray q;
	float hits[NUM_STATES];
	for (int i = 0; i < NUM_STATES; ++i)
	{	
		// ROS_INFO_STREAM("-------------------------" << i << "------------------------"); 
		/*
		if (p.data[i] < 0.001)
		{
			bool hit = det_hit(i);
		//	ROS_INFO_STREAM("PROB @SENSE_1: " << p.data[i]  * (hit * p_hit + (1-hit) * 0.001) );
			q.data.push_back(p.data[i]  * (hit * p_hit + (1-hit) * 0.001) );
		} else {
			
		}
		*/
		int hit = det_hit(i);
		hits[i]=(hit * p_hit + (1-hit) * p_miss);

		double prob = p.data[i] * hits[i];
		// ROS_INFO_STREAM(p.data[i] << " ==> " << prob);
		q.data.push_back(prob);
	}

	// adjust ones to distribution
	//for (int i=0;i<NUM_STATES;i++){
	//	if(q.data[i]==p_hit){
	//		double temp=q.data[i];
	//		double valc=temp*2/3;
	//		double vals=temp*1/6;
	//		q.data[i-1]+=vals;
	//		q.data[i]=valc;
	//		q.data[i+1]+=vals;
	//	}
	//}

	const char * format = "z_k =[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n";
  	printf (format, hits[1],hits[4],hits[7],hits[10],hits[13],hits[16],hits[19],hits[22],hits[25]);
	

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

std_msgs::Float32MultiArray move(std_msgs::Float32MultiArray prob)
{
	std_msgs::Float32MultiArray q;

	// @ctrl_action angulo
	// @nav_velocity_pixels velocidad de navegacion

	/*
	double angulo_real = 0.0;
	if(ctrl_action>=0){
		angulo_real = (ctrl_action-45);
	}
	else {
		angulo_real = 90;
	}
	
	*/

	p_exact = .50;
	p_undershoot = .25;
	p_undershoot_2 = .05;
	p_overshoot = .20;

	U = 0;
	angulo_real = (ctrl_action+45);

	// a que estado podria llegar basado en velocidad y steering
	// double dist_x = cos(angulo_real*M_PI/180);//*nav_velocity_pixels;

	int mov_under = 0;
	int mov_over = 0;

	double diff=0;
	if(angulo_real > 90-10 && angulo_real < 90+10) { // 90 +- 45/2 grados
		U = 0;
		mov_under=1;
		mov_over=-1;

		diff = (angulo_real-90)/100;

		p_exact -= std::abs(diff);
		if(diff<0){
			p_undershoot += std::abs(diff) + p_undershoot_2 ;
			p_undershoot_2 = 0;
		}
		else {
			p_overshoot += std::abs(diff);
		}
	}
	else{
		if(angulo_real > 90){
			U=1; //mov izquierda
			mov_under=-1;
			mov_over=1;

			diff = (angulo_real-115)/100;

			p_exact -= std::abs(diff);
			if(diff<0){
				p_undershoot += std::abs(diff)/2;
				p_undershoot_2 += std::abs(diff)/2;
			}
			else {
				p_overshoot += std::abs(diff);
			}
		}
		else{
			U=-1; // mov derecha
			mov_under=1;
			mov_over=-1;

			diff = (angulo_real-70)/100;

			p_exact -= std::abs(diff);
			if(diff>=0){
				p_undershoot += std::abs(diff)/2;
				p_undershoot_2 += std::abs(diff)/2;
			}
			else {
				p_overshoot += std::abs(diff);
			}
		}
	}
	
	ROS_INFO_STREAM("Angulo: " << angulo_real << ", pExact: "<<p_exact<<", pUndershoot: "<<p_undershoot<<", pOvershoot: "<<p_overshoot<<", diff: " << diff);
	
	for (int i = 0; i < NUM_STATES; i++)
	{

		double s = 0.0;
		
		int mov = i+U;
		//HISTOGRAMA CICLICLO
		// int mod2 = (mov) % NUM_STATES;
		// if(mod2<0) mod2 = NUM_STATES+mod2;
		if(mov<0) mov=0;
		else if(mov>NUM_STATES-1) mov=NUM_STATES-1;
		s = p_exact * prob.data[mov];



		//UNDERSHOOT
		int mov2 = i+U+mov_under;
		if(mov2<0) mov2=0;
		else if(mov2>NUM_STATES-1) mov2=NUM_STATES-1;
		if(U==0)
			s += (1-p_exact)/2 * prob.data[mov2];
        else
        	s += p_undershoot * prob.data[mov2];
        //2
        mov2 = i+U+2*mov_under;
		if(mov2<0) mov2=0;
		else if(mov2>NUM_STATES-1) mov2=NUM_STATES-1;
		if(U==0)
			s += (1-p_exact)/2 * prob.data[mov2];
        else
        	s += p_undershoot_2 * prob.data[mov2];

        /*
        //HISTOGRAMA CICLICLO
        mov2 = i+U-2;
		//mod2 = (mov2) % NUM_STATES;
		//if(mod2<0) mod2 = NUM_STATES+mod2;
		if(mov2<0) mov2=0;
		else if(mov2>NUM_STATES-1) mov2=NUM_STATES-1;
        s += p_dundershoot * prob.data[mov2];
		*/
        


        // OVERSHOOT
        int mov3 = i+U+mov_over;
        if(mov3<0) mov3=0;
		else if(mov3>NUM_STATES-1) mov3=NUM_STATES-1;
		if(U==0)
			s += (1-p_exact)/2 * prob.data[mov3];
        else
        	s += p_overshoot * prob.data[mov3];
        /*2
        mov3 = i+U+2*mov_over;
        if(mov3<0) mov3=0;
		else if(mov3>NUM_STATES-1) mov3=NUM_STATES-1;
		if(U==0)
			s += (1-p_exact)/2 * prob.data[mov3];
        else
        	s += p_overshoot_2 * prob.data[mov3];
		*/

        /*
        //HISTOGRAMA CICLICLO
        mov3 = i+U+2;
		//mod2 = (mov3) % NUM_STATES;
		//if(mod2<0) mod2 = NUM_STATES+mod2;
        if(mov3<0) mov3=0;
		else if(mov3>NUM_STATES-1) mov3=NUM_STATES-1;
        s += p_dovershoot * prob.data[mov3];
		*/

		q.data.push_back(s);
	//	ROS_INFO_STREAM("p1: " << (edo_ini + .1) / NUM_STATES << " p2: " << p_bin );
	
	}

	return q;
}

int enQueEstadoEsta(std_msgs::Float32MultiArray locArray){
	float max=0;
	for(int i=0;i<NUM_STATES;i++){
	 	if(locArray.data[i]>max){
	 		max=locArray.data[i];
	 	}
	}

	int countEstados=0;
	int estado = -1;
	for(int i=NUM_STATES-1;i>=0;i--){
        if(locArray.data[i]==max){
            // ROS_INFO_STREAM("Estas en:" << nombre_estado[i]);
            estado = i;
            countEstados++;
        }
    }

    if (countEstados==1)
    	return (int)floor(estado/3);
    else
    	return -1; // no se pudo determinar el estado, ya que hay mas de uno posible
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
	// ROS_INFO_STREAM("lane_states_node initialized");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");
	ros::Rate loop_rate(rate_hz);
	
	std::string node_name = ros::this_node::getName();
        ROS_INFO_STREAM("Getting parameters");
        priv_nh_.param<float>(node_name+"/alpha", alpha,12);


	
	

	const char * format = "P(x)=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n";

	float bel_RC [NUM_STATES] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.1f,0.8f,0.1f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

	for (int i = 0; i < NUM_STATES; ++i)
	{
		// Iniciar con distribucion uniforme
		// p.data.push_back((float) (1/(float)NUM_STATES));
		// Iniciar con carril derecho
		p.data.push_back(bel_RC[i]);
	}

	//const char * format = "P(x)=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n";
	// ROS_INFO_STREAM("Array initialization: ");
  	// printf (format, p.data[1],p.data[4],p.data[7],p.data[10],p.data[13],p.data[16],p.data[19],p.data[22],p.data[25]);
		

	ROS_INFO_STREAM("alpha: " << alpha);
	ROS_INFO_STREAM("Array initialization: \n" << p);

	pub_loc = nh.advertise<std_msgs::Float32MultiArray>("/localization_array", rate_hz);

	ros::Subscriber sub_pts_left = nh.subscribe("/points/left",1, get_pts_left);
	ros::Subscriber sub_pts_center = nh.subscribe("/points/center",1, get_pts_center);
	ros::Subscriber sub_pts_right = nh.subscribe("/points/right",1, get_pts_right);
	ros::Subscriber sub_mov = nh.subscribe("/manual_control/steering",1,get_ctrl_action);
	ros::Subscriber sub_des_state = nh.subscribe("/desire_state",1, get_des_state);
	

	loop_rate.sleep();
	loop_rate.sleep();
	loop_rate.sleep();

	int estadoEstimado;
	
	int contador = 0;

	int estadoFijo = 0;

	while(nh.ok())
	{
		// L=0;
		// R=0;
		// C=0;

	    ros::spinOnce();
	    
	    if(!estadoFijo){

		    // ROS_INFO_STREAM("Sensing update");
			p = sense(p);
			
			// printf (format, p.data[0],p.data[1],p.data[2],p.data[3],p.data[4],p.data[5],p.data[6],p.data[7],p.data[8],p.data[9],p.data[10],p.data[11],p.data[12],p.data[13],p.data[14],p.data[15],p.data[16],p.data[17],p.data[18],p.data[19],p.data[20],p.data[21],p.data[22],p.data[23],p.data[24],p.data[25],p.data[26]);
			printf (format, p.data[1],p.data[4],p.data[7],p.data[10],p.data[13],p.data[16],p.data[19],p.data[22],p.data[25]);

			pub_loc.publish(p);

			int estadoEstimado = enQueEstadoEsta(p);
			printf("%d, %d, %d, %d, %.2f, %.2f, %.2f, %.2f, %s\n", contador++, L, C, R, dist_ll, dist_cc, dist_rr, angulo_real, nombre_estado[estadoEstimado].c_str());


		    
		    //ROS_INFO_STREAM("Motion update: ");
		    p = move(p);
		    
		    // print_state_order();
	    }

	    loop_rate.sleep();
	}
	return 0;
}

