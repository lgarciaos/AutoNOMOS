#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

class test_head
{
	
private:
    ros::Subscriber head_subscriber;
    ros::Publisher motor_publisher_move;
    ros::Publisher motor_publisher_steering;
    std_msgs::Int16 motor_command;

public: 
	void move(int msg){
    
	  	motor_command.data =  msg;
    	ROS_INFO("Moving at: %d (RPM)", msg);
    	motor_publisher_move.publish(motor_command);
  	
  	}

  	void steering(int msg){
  		motor_command.data = msg;
  		ROS_INFO("Steering to position: %d", msg);
  		motor_publisher_steering.publish(motor_command);

  	}

	test_head(ros::NodeHandle nh)
	{
		
    motor_publisher_move = nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/speed"), 10);
    motor_publisher_steering = nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_control/steering"), 10);

    
    ros::Duration(1).sleep();
    
//     motor_command.data = -500;
//     ROS_INFO("Speed 100");
//     motor_publisher.publish(motor_command); 
    test_head::move(-200);
    test_head::steering(45);
    ros::Duration(3).sleep(); //sleeps for a second
    
    //motor_command.data = "0";
    //ROS_INFO("Speed 0");
    //motor_publisher.publish(motor_command);
    test_head::move(0);
    test_head::steering(90);
    
    ros::Duration(1).sleep(); //sleeps for a second
	}
  
  	
  
};
// class test_head
// {
// public:
// 	test_head(ros::NodeHandle nh)
// 	{

// 	    light_publisher=nh.advertise<std_msgs::String>(nh.resolveName("manual_control/lights"), 10);
// 	    head_subscriber = nh.subscribe("model_car/yaw", 1, &test_head::headCallback,this);
// 	}
// 	~test_head(){}

//     void headCallback(const std_msgs::Float32& head)
// 	{
// 		if (head.data > 90.0 )
// 	    {
// 	    	if (light_command.data!="le")
// 	    	{
// 	    		light_command.data="le";
// 				ROS_INFO("head is bigger than 90");
// 				light_publisher.publish(light_command);
// 	    	}
// 	    }
// 	    else if (light_command.data!="ri")
// 	    {
// 	    	light_command.data="ri";
// 			ROS_INFO("head is less than 90");
// 			light_publisher.publish(light_command);
// 	    }     
// 	}
// private:
//     ros::Subscriber head_subscriber;
//     ros::Publisher light_publisher;
//     std_msgs::String light_command;

// };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_sensorless_node");
    ros::NodeHandle nh; 

    test_head test_head_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second

	while(ros::ok())
	{
		//rate.sleep();
		ros::spinOnce();
	}
    return 0;
}