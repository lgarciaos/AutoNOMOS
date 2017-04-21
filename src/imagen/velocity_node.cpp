
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/LU>
#include <math.h>
#include <stdio.h>
#include <Eigen/Geometry>
#include <math.h>
#include <std_msgs/Int16.h>

using namespace Eigen;

geometry_msgs::Twist robot_position;
geometry_msgs::Twist velocity_msg;
double velMots[2];

double rate_hz = 1;

double* getMotorValue(double x_velocity, double y_velocity, double w_velocity){

	double c1,c2,offset;
//-------------------------------------ESTAS CONSTANTES AJUSTAN LOS PARAMETROS PARA EL STEER------------------------------------------
	c1=1;
	c2=1;
	offset=30;
	if(x_velocity>100)
		x_velocity = 100;
	if(x_velocity<-100)
		x_velocity  = -100;
    velMots[0] = -50;
    velMots[1] =((x_velocity*c1)-100)*(-0.875)*c2+offset;
//	if(velMots[1]>175(
//		velMots[1]=175;
//    std::cout <<". velMots[1]= " <<velMots[1];
    return velMots;
}

void get_vel_vec(const geometry_msgs::Twist& msg) {
	velocity_msg.linear.x = msg.linear.x;
	velocity_msg.linear.y = msg.linear.y;
    velocity_msg.angular.z = msg.linear.z; 
}


int main(int argc, char **argv){
	ros::init(argc,argv,"velocity_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("velocity_node initialized");
	ROS_INFO_STREAM(ros::this_node::getName());

	ros::Subscriber sub_vel = nh.subscribe("/target_position_topic", 1000, &get_vel_vec);
	double tiempo = 0;

        ros::Publisher pub = nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/speed"), 1);
        ros::Publisher pub_ste = nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/steering"), 1);

	while (ros::ok())
	{
	double* velMots = getMotorValue(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.angular.z);

        ros::Publisher pub = nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/speed"), 1);
        ros::Publisher pub_ste = nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/steering"), 1);


        double tiempo = 0;

        ros::Rate rate(rate_hz);
        ros::Time start_time ;
        ros::Duration duration ;

        double effort[3];

                std_msgs::Int16 value_motor;
                std_msgs::Int16 value_ste;
                value_motor.data =velMots[0];
                value_ste.data =velMots[1];
                pub.publish(value_motor);
                pub_ste.publish(value_ste);

		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}
