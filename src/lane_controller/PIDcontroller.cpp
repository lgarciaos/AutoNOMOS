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

ros::Publisher pub_speed;
ros::Publisher pub_steering;

nav_msgs::GridCells path_planning;

/*
	@param pE posicion origen (del vehiculo)
	@param p posicion destino
*/
	double getThetaError (double pE, double p){
	// Asumiendo 'y' = 100 fijo, que pE y p estan en unidades: pixeles
		double x = (pE - p);
		double theta = atan2(x,100);
		theta = (theta * 180 / PI) + 90;
		theta = -theta;
	// Regresa valores entre 0 y 180 grados

		return theta;

	}

	double PIDtime(double pE, double p, double dt, double max, double min, double Kp, double Kd, double Ki){
		ROS_INFO_STREAM("PID time");
		double error = pE - p;
		double pOut = Kp * error;
		integral += error * dt;
		double iOut = Ki * integral;
		double derivative = (error - prevError) / dt;
		double dOut = Kd * derivative;
		double output = pOut + iOut + dOut;

	// correccion en el carro
		// output += 45; 


    // Restriction
		if( output > max )
			output = max;
		else if( output < min )
			output = min;

	// cambiar los sentidos
	// output = 90-output;

		prevError = error;

		return output;
	}

	void get_path(const nav_msgs::GridCells& path){
		path_planning.cells = path.cells;
		path_planning.cell_width = path.cell_width;

		ROS_INFO_STREAM("cells_width: " << path_planning.cell_width);

		if(path_planning.cell_width > 0){
			double p = 0.0;
			double pid_res = 0.0;
			std_msgs::Int16 value_motor;
			std_msgs::Int16 value_steering;


			if(path_planning.cells[0].x >= 0) {
				p = path_planning.cells[0].x;


				ROS_INFO_STREAM("PID: pos Esperada: " << pE << ", pos Actual:" << p );
				// 'p' en terminos de theta en grados
				p = getThetaError(pE, p)/2;

				// El servomotor del coche siempre tiene que estar en 45 grados
				pid_res = PIDtime(45, p, dt, max, min, Kp, Kd, Ki);

				value_motor.data = velocity;
				value_steering.data = pid_res;

				pub_speed.publish(value_motor); 
				pub_steering.publish(value_steering); 

				ROS_INFO_STREAM("velocity: " << value_motor.data << ", steering: " << value_steering.data << " )");
			}
			else{
			// talvez ir derecho
			}
		}
	}


	int main(int argc, char** argv){
		ros::init(argc, argv, "PID controller");
		ROS_INFO_STREAM("PID controller initialized");
		ros::NodeHandle priv_nh_("~");
	//head_time_stamp = ros::Time::now();
		ros::NodeHandle nh;
		ros::Rate loop_rate(rate_hz);

		std::string node_name = ros::this_node::getName();
		ROS_INFO_STREAM("Obteniendo p");

		priv_nh_.param<double>(node_name+"/Kp", Kp, 0.6);
		priv_nh_.param<double>(node_name+"/Ki", Ki, 0.3);
		priv_nh_.param<double>(node_name+"/Kd", Kd, 0.0);
		priv_nh_.param<double>(node_name+"/dt", dt, 10.0);

		priv_nh_.param<double>(node_name+"/pE", pE, 80.0);
		priv_nh_.param<double>(node_name+"/min", min, 0.0);
		priv_nh_.param<double>(node_name+"/max", max, 90.0);
		priv_nh_.param<double>(node_name+"/velocity", velocity, 30.0);

		ROS_INFO_STREAM("Parametros obtenidos");

		pub_speed = nh.advertise<std_msgs::Int16>("/manual_control/speed", rate_hz);
		pub_steering = nh.advertise<std_msgs::Int16>("/manual_control/steering", rate_hz);

	// esto va mejor en el launch file

		ros::Subscriber sub_path = nh.subscribe("/planning",1, &get_path);

		ROS_INFO_STREAM("antes de while");
		while (nh.ok())
		{
			ROS_INFO_STREAM("while 1");
			ros::spinOnce();
		// convertir entre 0 y 90
		// pE siempre es la misma
			loop_rate.sleep();
		}
		return 0;
	}
