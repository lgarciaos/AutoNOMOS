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


#include <iostream>
#include <ros/ros.h>
// #include "stdmsgs/String.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/GridCells.h>
#include <cmath>
#include <stdio.h>

double rate_hz = 1;

// Variables globales 
double max = 90;
double min = 0;
double dt;
double Kp;
double Kd;
double Ki;
double prevError = 0;
double integral = 0;


/*
	@param pE posicion origen (del vehiculo)
	@param p posicion destino
*/
double PIDtime(double pE, double p, double dt, double max, double min, double Kp, double Kd, double Ki){
	double error = pE - p;
	double pOut = Kp * error;
	integral += error * dt;
	double iOut = Ki * integral;
	double derivative = (error - prevError) / dt;
	double dOut = Kd * derivative;
	double output = pOut + iOut + dOut;

    // Restriction
	if( output > max )
		output = max;
	else if( output < min )
		output = min;

	prevError = error;

	return output;
}

void get_path(const nav_msgs::GridCells& path){

}


int main(int argc, char** argv){
	ros::init(argc, argv, "PIDcontroller");
	ROS_INFO_STREAM("PID controller initialized");
	ros::NodeHandle nh;
	ros::Rate loop_rate(rate_hz);

	ros::Publisher pub_speed = nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/speed"), 1);
	ros::Publisher pub_steering = nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/steering"), 1);

	// esto va mejor en el launch file
	
	ros::Subscriber sub_path = nh.subscribe("/planning",1, get_path);

	double p = targetposition.linear.x;

	while (ros::ok())
	{
		// convertir entre 0 y 90
		// pE siempre es la misma
		value_ste.data = PIDtime(pE, p, dt, max, min, Kp, Kd, Ki,);

		pub.publish(value_motor); 
		pub_ste.publish(value_ste); 
 
		ROS_INFO_STREAM("\nvals: (" << value_motor.data << " , " << value_ste.data << " )");	
																																																																																									
		
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}