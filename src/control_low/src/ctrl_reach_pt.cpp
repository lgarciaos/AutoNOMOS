#include "ctrl_reach_pt.h"

ros::Publisher pub_speed;
ros::Publisher pub_steer;

void get_ste_sim(const std_msgs::Int16& val)
{
	ROS_INFO_STREAM("Steering sent: " << val.data);
	pub_steer.publish(val);
}

void get_vel_sim(const std_msgs::Int16& val)
{
	ROS_INFO_STREAM("Speed sent: " << val.data);
	pub_speed.publish(val);
}

void get_ste_real(const std_msgs::Int16& val)
{
	ROS_INFO_STREAM("NOT IMPLEMENTED YET...");
}

void get_vel_real(const std_msgs::Int16& val)
{
	ROS_INFO_STREAM("NOT IMPLEMENTED YET...");
}

void mySigintHandler(int sig)
{
	std_msgs::Int16 last_speed;
	last_speed.data = 0;

	ROS_INFO_STREAM("Stopping the car: sending speed 0");

	pub_speed.publish(last_speed);
	ros::shutdown();
}


int main(int argc, char** argv){
		ros::init(argc, argv, "low controller node");
		ROS_INFO_STREAM("low controller node initialized");
		ros::NodeHandle priv_nh_("~");
		ros::NodeHandle nh;

		// ros::Rate loop_rate(rate_hz);

		std::string node_name = ros::this_node::getName();

		ROS_INFO_STREAM("Parametros obtenidos");
		priv_nh_.param<int>("/autonomos_number", autonomos_number, -1);

		nh.param<bool> ("simulation/simulation", simulation, true);

		if (simulation) {
			boost::format format_aux = boost::format("/AutoNOMOS_mini_%1%");
			std::string str_num = autonomos_number == -1 ? "/AutoNOMOS_mini" :
				boost::str(format_aux % autonomos_number) ;

			pub_speed = nh.advertise<std_msgs::Int16>(str_num +
				"/manual_control/speed", MY_ROS_QUEUE_SIZE, true);
			pub_steer = nh.advertise<std_msgs::Int16>(str_num +
				"/manual_control/steering", MY_ROS_QUEUE_SIZE, true);

			sub_ste = nh.subscribe("/ctrl/steering", MY_ROS_QUEUE_SIZE, &get_ste_sim);
			sub_vel = nh.subscribe("/ctrl/speed",    MY_ROS_QUEUE_SIZE, &get_vel_sim);
		}
		else
		{
			pub_speed = nh.advertise<std_msgs::Int16>("/manual_control/speed", MY_ROS_QUEUE_SIZE);
			pub_steer = nh.advertise<std_msgs::Int16>("/manual_control/steering"	, MY_ROS_QUEUE_SIZE);
			sub_vel = nh.subscribe("/ctrl/steering", MY_ROS_QUEUE_SIZE, &get_vel_real);
			sub_ste = nh.subscribe("/ctrl/speed", MY_ROS_QUEUE_SIZE, &get_ste_real);
		}


		while (ros::ok())
		{
			// ste
			ros::spin();
			// loop_rate.sleep();
		}

		return 0;
	}
