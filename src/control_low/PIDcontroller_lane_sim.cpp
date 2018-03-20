// --pseudocode--
// previouserror = 0
// integral = 0 
// start:
// error = pE - measuredvalue
// integral = integral + error*dt
// derivative = (error - previouserror)/dt
// output = Kp*error + Ki*integral + Kd*derivative
// previouserror = error
// wait(dt)
// goto start
// -------------

// --codeImplementation--
// https://gist.github.com/bradley219/5373998


#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#define PI 3.14159265

static const uint32_t MY_ROS_QUEUE_SIZE = 1;

int librarobstaculo = 0;
double distancialibrarobstaculo=0;
double distancia_librarobstaculomasmenos=0.1;
double pendienteTrancazo =0.0;
bool rebasando = false;

double rate_hz = 10;

double max;
double min;
double dt;
double Kp;
double Kd;
double Ki;
double prevError = 0;
double integral = 0;
double pE;
double velocity;
geometry_msgs::Twist velocity_msg;
geometry_msgs::Twist positionObj;
std_msgs::Float32 vel;
std_msgs::Float32 steer;

ros::Publisher pub_speed_ste;
ros::Publisher pub_steer_ste;

double theta = 0.0; 

double steering_actual = 0.0;
float speed = 0.0;

// pActual es el angulo actual del steering en radianes
// pDestino angulo requerido en radianes
// dt delta t para D, pero no se utiliza
// Constantes Kp, Kd, Ki
double PIDtime(double pActual, double pDestino, double dt, double Kp, double Kd, double Ki){
	theta = pDestino; 
	double error = theta;
	double pOut = Kp * error;
	integral += error * dt;
	double iOut = Ki * integral;
	double derivative = (error - prevError) / dt;
	double dOut = Kd * derivative;
	double output = pOut + iOut + dOut;
	prevError = error;

	return output;
}


// reads steering from standarized topic
void get_ctrl_action_steer(const std_msgs::Float32& val) {
	steering_actual = val.data;
}

// reads speed from standarized topic
void get_ctrl_action_vel(const std_msgs::Float32& val) {
	// negative is forward
	speed = sqrt(val.data * val.data);
}

//void get_lidar(const geometry_msgs::Twist& msg) {
	//positionObj.linear.x = msg.linear.x;
	//positionObj.linear.y = msg.linear.y;
    //positionObj.angular.z = msg.linear.z; 
//}

void get_vel_vec(const geometry_msgs::Twist& msg) {

		double p = 0.0;
		float pid_res = 0.0;

		double posEsp = msg.angular.z; // pixeles: velocity_msg.linear.x;
		double posActual = steering_actual; // pixeles: pE; 
		
		printf("\n Angulo Esperado: %+010.4f, Actual: %+010.4f", posEsp, posActual);
		
		// OBSTACULO
		/*
		if(librarobstaculo){
			double distanciaObstaculo = sqrt((velocity_msg.linear.x*velocity_msg.linear.x)+(velocity_msg.linear.y*velocity_msg.linear.y));
			if((positionObj.angular.z > 70 && positionObj.angular.z < 120)){
				
				if(distanciaObstaculo < distancialibrarobstaculo){
					//APLICAR CORRECCION a la IZQUIERDA
					posEsp -= velocity*pendienteTrancazo; // -100px talvez
					// activar rebasando
					rebasando = true;
				}
			}
			else if(positionObj.angular.z < -70 && positionObj.angular.z > -120) {
				// si rebasando

				if(distanciaObstaculo < distancialibrarobstaculo){
				// aplicar correccion a la derecha
					posEsp += velocity*pendienteTrancazo; // -100px talvez
					// activar rebasando
					rebasando = false;
				}
			}
		}
		*/

		p = PIDtime(posActual, posEsp, dt, Kp, Kd, Ki);
		
		pid_res=p;
		// Restriction en las llantas
		if( pid_res > max )
			pid_res = max;
		else if( pid_res < min )
			pid_res = min;

		printf("\n Error theta: %+010.4f, Res PID: %+010.4f, Senal Servo: %+010.4f", theta, p, pid_res );

		vel.data = (float)velocity;
		steer.data = pid_res;

		pub_speed_ste.publish(vel);
		pub_steer_ste.publish(steer);
}

int main(int argc, char** argv){
		ros::init(argc, argv, "PID controller sim");
		ROS_INFO_STREAM("PID controller initialized");
		ros::NodeHandle priv_nh_("~");
		ros::NodeHandle nh;

		ros::Rate loop_rate(rate_hz);

		std::string node_name = ros::this_node::getName();
		std::string topico_velocidad;
		std::string topico_steering;

		ROS_INFO_STREAM("Parametros obtenidos");
		priv_nh_.param<double>(node_name+"/Kp", Kp, 0.6);
		priv_nh_.param<double>(node_name+"/Ki", Ki, 0.3);
		priv_nh_.param<double>(node_name+"/Kd", Kd, 0.0);
		priv_nh_.param<double>(node_name+"/dt", dt, 10.0);
		priv_nh_.param<double>(node_name+"/pE", pE, 80.0);
		priv_nh_.param<double>(node_name+"/min", min, 0.0);
		priv_nh_.param<double>(node_name+"/max", max, 90.0);
		priv_nh_.param<double>(node_name+"/velocity", velocity, 30.0);
		priv_nh_.param<int>(node_name+"/activa_librarobstaculo", librarobstaculo, 0);
		priv_nh_.param<double>(node_name+"/distancia_librarobstaculo", distancialibrarobstaculo, 9.0);
		priv_nh_.param<double>(node_name+"/distancia_librarobstaculomasmenos", distancia_librarobstaculomasmenos, 9.0);
		priv_nh_.param<double>(node_name+"/pendienteTrancazo", pendienteTrancazo, 0.66);
		priv_nh_.param<std::string>(node_name+"/topico_velocidad", topico_velocidad, "/AutoNOMOS_mini/manual_control/velocity");
		priv_nh_.param<std::string>(node_name+"/topico_steering", topico_steering, "/AutoNOMOS_mini/manual_control/steering");

		pub_speed_ste = nh.advertise<std_msgs::Float32>(topico_velocidad, MY_ROS_QUEUE_SIZE);
		pub_steer_ste = nh.advertise<std_msgs::Float32>(topico_steering, MY_ROS_QUEUE_SIZE);

		//CREO QUE ESTO NO SIRVE DE NADA
		ros::Subscriber sub_steering = nh.subscribe(topico_steering, MY_ROS_QUEUE_SIZE, &get_ctrl_action_steer);
		ros::Subscriber sub_velocity = nh.subscribe(topico_velocidad, MY_ROS_QUEUE_SIZE, &get_ctrl_action_vel);

		ros::Subscriber sub_lidar = nh.subscribe("/target_pose", MY_ROS_QUEUE_SIZE, &get_vel_vec);

		while (nh.ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		return 0;
	}
