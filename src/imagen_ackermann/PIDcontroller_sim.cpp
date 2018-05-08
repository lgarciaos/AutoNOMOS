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
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#define PI 3.14159265

int librarobstaculo = 0;
double distancialibrarobstaculo=0;
double distancia_librarobstaculomasmenos=0.1;
double pendienteTrancazo =0.0;
bool rebasando = false;

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
geometry_msgs::Twist velocity_msg;
geometry_msgs::Twist positionObj;

geometry_msgs::Twist vel;

ros::Publisher pub_speed;
ros::Publisher pub_steering;

nav_msgs::GridCells path_planning;

double theta = 0.0; 
/*
	@param pActual posicion origen (del vehiculo)
	@param pEsperada posicion destino
*/
double getThetaError (double pActual, double pEsperada){
	// Asumiendo 'y' = 100 fijo, que pE y p estan en unidades: pixeles
	double x = (pActual - pEsperada);
	double theta = atan2(x,100);

	//conversion a grados	
	//theta = (theta * 180 / PI)/2; // quite el /2

	// theta = -theta; // por correccion con el carro 90 es izquierda
	// Regresa valores entre -45 y 45 grados

	return theta;

}

double PIDtime(double pActual, double pDestino, double dt, double Kp, double Kd, double Ki){
	// ROS_INFO_STREAM("PID time");
	theta= getThetaError(pActual, pDestino);
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


//void get_lidar(const geometry_msgs::Twist& msg) {
	//positionObj.linear.x = msg.linear.x;
	//positionObj.linear.y = msg.linear.y;
    //positionObj.angular.z = msg.linear.z; 
//}









void get_vel_vec(const geometry_msgs::Twist& msg) {
	velocity_msg.linear.x = msg.linear.x;
	velocity_msg.linear.y = msg.linear.y;
	velocity_msg.angular.z = msg.linear.z; 

	vel.linear.x=velocity;

    
		double p = 0.0;
		double pid_res = 0.0;
		std_msgs::Int16 value_motor;
		std_msgs::Float64 value_steering;


		if(velocity_msg.linear.x >= 0) {
			// p = point.x;

			double posEsp = velocity_msg.linear.x;
			double posActual = pE; 
			ROS_INFO_STREAM("PID: posPixel Esperada: " << posEsp << ", posPixel Actual:" << posActual );
			
			// OBSTACULO
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


			// El servomotor del coche siempre tiene que estar en 45 grados
			p = PIDtime(posActual, posEsp, dt, Kp, Kd, Ki);

			//pid_res = 45 + p; // por detalle con el carro de los angulos
			
			pid_res=p;
			// Restriction
			if( pid_res > max )
				pid_res = max;
			else if( pid_res < min )
				pid_res = min;

			ROS_INFO_STREAM("Error theta:" << theta <<", Res PID: " << p << ", Senal Servo:" << pid_res );

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
		ros::init(argc, argv, "PID controller sim");
		// ROS_INFO_STREAM("PID controller initialized");
		ros::NodeHandle priv_nh_("~");
	//head_time_stamp = ros::Time::now();
		ros::NodeHandle nh;

		ros::Rate loop_rate(rate_hz);

		std::string node_name = ros::this_node::getName();
		// ROS_INFO_STREAM("Obteniendo p");
		std::string topico_steering;
		std::string topico_velocidad;

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
		priv_nh_.param<std::string>(node_name+"/topico_steering", topico_steering, "/steering");
		priv_nh_.param<std::string>(node_name+"/topico_velocidad", topico_velocidad, "/velocidad");

		// ROS_INFO_STREAM("Parametros obtenidos");

		pub_speed = nh.advertise<geometry_msgs::Twist>(topico_velocidad, rate_hz);
		pub_steering = nh.advertise<std_msgs::Float64>(topico_steering, rate_hz);

		// esto va mejor en el launch file
		// ros::Subscriber sub_vel = nh.subscribe("/target_position_topic", 1000, &get_vel_vec);

		ros::Subscriber sub_lidar = nh.subscribe("/target_pose", 1000, &get_vel_vec);

		//pub_lidar = nh.advertise<geometry_msgs::Twist>("/target_pose", rate_hz);

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
