#include "follow_path.h"


void get_path_callback(const motion_planning::ctrl_path msg)
{
	path = msg;
}

void get_next_ctrl()
{
	// std::cout << "The angle is: " << path.steering[current_ctrl] << " = " <<
	// 	path.steering[current_ctrl] * 180 + 90 << " ==> ";

	ste.data = path.steering[current_ctrl] * 180 + 90;
	// std::cout << "ste.data" << ste.data << '\n';
	vel.data = path.speed[current_ctrl] * - 3000;
	duration = path.duration[current_ctrl];
	current_ctrl++;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "follow_path_node");
	ROS_INFO_STREAM("follow_path_node initialized");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");

	ros::Subscriber sub_obs_poses  = nh.subscribe("/motion_planning/path", 1,
		&get_path_callback);

	ros::Publisher pub_ste = nh.advertise<std_msgs::Int16>("/ctrl/steering", 1);
	ros::Publisher pub_vel = nh.advertise<std_msgs::Int16>("/ctrl/speed", 1);

	current_point = 0;
	current_ctrl = 0;

	ros::topic::waitForMessage<motion_planning::ctrl_path>("/motion_planning/path");
	ros::spinOnce();
	while(ros::ok())
	{
		// pub_target_pose.publish(next_pose);
		get_next_ctrl();
		ROS_INFO_STREAM("i:\t" << current_ctrl << "\tDuration:\t" << duration);
		pub_ste.publish(ste);
		pub_vel.publish(vel);

		ros::Duration(duration).sleep();
		if (current_ctrl >= path.speed.size())
		{

			break;
		}
	}
ROS_INFO_STREAM("i:\t" << current_ctrl + 1 << "\tDuration:\t" << duration);
	vel.data = 0;
	pub_vel.publish(vel);
}
