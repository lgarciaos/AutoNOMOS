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
//#include <nav_msgs/GridCells.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#define PI 3.14159265


double rate_hz = 10;

// Variables globales 
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
std::string topicVel;
std::string topicSte;
geometry_msgs::Twist vel;

ros::Publisher pub_speed;
ros::Publisher pub_steering;

//nav_msgs::GridCells path_planning;

double theta = 0.0; 
/***
	@param pActual posicion origen (del vehiculo)
	@param pEsperada posicion destino
***/
double getThetaError (double pActual, double pEsperada){
	// Asumiendo 'y' = 100 fijo, que pE y p estan en unidades: pixeles
	double x = (pActual - pEsperada);
	double theta = atan2(x,100);
	// para convertir a grados	
	//theta = (theta * 180 / PI)/2; // quite el /2
	
	// theta = -theta; // por correccion con el carro 90 es izquierda
	// Regresa valores entre -45 y 45 grados

	return theta;

}

double PIDtimePixeles(double pActual, double pDestino, double dt, double max, double min, double Kp, double Kd, double Ki){
	// ROS_INFO_STREAM("PID time");
	theta= getThetaError(pActual, pDestino);
	double error = theta;
	double pOut = Kp * error;
	integral += error * dt;
	double iOut = Ki * integral;
	double derivative = (error - prevError) / dt;
	double dOut = Kd * derivative;
	double output = pOut + iOut + dOut;
	// ROS_INFO_STREAM("PIDres: " << output);
	

	// correccion en el carro
		// output += 45; 

	// cambiar los sentidos
	// output = 90-output;

	prevError = error;

	return output;
}



void get_pathxy(const geometry_msgs::Point& point){
		//path_planning.cells = path.cells;
		//path_planning.cell_width = path.cell_width;

		// ROS_INFO_STREAM("cells_width: " << path_planning.cell_width);

		//if(path_planning.cell_width > 0){
			double p = 0.0;
			double pid_res = 0.0;
			std_msgs::Int16 value_motor;
			std_msgs::Float64 value_steering;

			vel.linear.x=velocity;
			
			if(point.x >= 0) {
				// p = point.x;

				double posEsp = point.x;
				double posActual = pE; 
				// ROS_INFO_STREAM("PID: posPixel Esperada: " << pE << ", posPixel Actual:" << p );
				// 'p' en terminos de theta en grados de -45 a 45
				// p = getThetaError(posActual, posEsp);

				// El servomotor del coche siempre tiene que estar en 45 grados
				p = PIDtimePixeles(posActual, posEsp, dt, max, min, Kp, Kd, Ki);

				//pid_res = 45 + p; // por detalle con el carro de los angulos
				pid_res=p;
				ROS_INFO_STREAM("Servo: " << pid_res);
				// Restriction
				if( pid_res > max )
					pid_res = max;
				else if( pid_res < min )
					pid_res = min;

				ROS_INFO_STREAM("Error theta:" << theta <<", Res PID: " << p << ", Señal Servo:" << pid_res << ", Pos esp: " << posEsp );

				//value_motor.data = velocity;
				value_steering.data = pid_res;

				pub_speed.publish(vel); 
				pub_steering.publish(value_steering); 

				// ROS_INFO_STREAM("velocity: " << value_motor.data << ", steering: " << value_steering.data << " )");
			}
			else{
			// talvez ir derecho
			}
		
}


int main(int argc, char** argv){
		ros::init(argc, argv, "PID controller");
		// ROS_INFO_STREAM("PID controller initialized");
		ros::NodeHandle priv_nh_("~");
	//head_time_stamp = ros::Time::now();
		ros::NodeHandle nh;

		ros::Rate loop_rate(rate_hz);

		std::string node_name = ros::this_node::getName();
		// ROS_INFO_STREAM("Obteniendo p");

		priv_nh_.param<double>(node_name+"/Kp", Kp, 0.6);
		priv_nh_.param<double>(node_name+"/Ki", Ki, 0.3);
		priv_nh_.param<double>(node_name+"/Kd", Kd, 0.0);
		priv_nh_.param<double>(node_name+"/dt", dt, 10.0);

		priv_nh_.param<double>(node_name+"/pE", pE, 80.0);
		priv_nh_.param<double>(node_name+"/min", min, -0.5);
		priv_nh_.param<double>(node_name+"/max", max, 0.5);
		priv_nh_.param<double>(node_name+"/velocity", velocity, 0.1);

		// ROS_INFO_STREAM("Parametros obtenidos");

		pub_speed = nh.advertise<geometry_msgs::Twist>("/autonomos/cmd_vel", rate_hz);
		pub_steering = nh.advertise<std_msgs::Float64>("/autonomos/steer/steer_position_controller/command", rate_hz);

	// esto va mejor en el launch file

		// ros::Subscriber sub_pathxy = nh.subscribe("/planningxy",1, &get_pathxy);

		// ros::Subscriber sub_angle = nh.subscribe("/lane_model/angle",1,&get_Angle);

		// ROS_INFO_STREAM("antes de while");
		while (nh.ok())
		{
			// ROS_INFO_STREAM("while 1");
			ros::spinOnce();
		// convertir entre 0 y 90
		// pE siempre es la misma
			loop_rate.sleep();
		}
		return 0;
	}
