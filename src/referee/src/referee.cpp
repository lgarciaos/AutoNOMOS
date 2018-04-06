#include "referee.h"


using namespace std;




referee::referee(ros::NodeHandle nh)
    : nh_(nh), priv_nh_("~")
{
    std::string node_name = ros::this_node::getName();

    ROS_INFO("Node name: %s",node_name.c_str());



}

referee::~referee()
{
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "cLaneDetectionFu");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE_HZ);

    referee node = referee(nh);

    ros::Publisher des_state = nh.advertise<std_msgs::Int16>("/des_state", 1);

    // //ROS_INFO_STREAM("Before while");
    std_msgs::Int16 des_state_msg;
    des_state_msg.data = STATE_RIGHT_CENTER;
    while(ros::ok())
    {
    	des_state.publish(des_state_msg);
        ros::spinOnce();
        loop_rate.sleep();
        // //ROS_INFO_STREAM("At while");
    }
    return 0;
}