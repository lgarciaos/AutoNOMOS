#include "follow_trajectory.h"


void get_path_callback(const motion_planning::car_trajectory msg)
{
	path = msg;
	trajectory_available = msg.path_len > 0;
}

bool get_next_trajectory_segment(navigation::trajectory_segment::Request &req,
								 navigation::trajectory_segment::Response &res)
{

	// ste.data = path.steering[current_ctrl] * 180 + 90;
	// vel.data = path.speed[current_ctrl];
	// duration = path.duration[current_ctrl];
	// current_ctrl++;
	if (trajectory_available && req.seq < path.path_len)
	{
		res.is_valid = 1;
		res.speed    = path.speed[req.seq];
		res.steering = path.steering[req.seq];
		res.duration = path.duration[req.seq];
	}
	else 
	{
		res.is_valid = 0;
		res.speed = 0;
		res.steering = 0;
		res.duration = -1;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "follow_trajectory_node");
	ROS_INFO_STREAM("follow_trajectory_node initialized");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");

	ros::Subscriber sub_obs_poses  = nh.subscribe("/motion_planning/path", 1, &get_path_callback);

	ros::ServiceServer service_next_ctrl = nh.advertiseService("/navigation/get_next_control", &get_next_trajectory_segment);

	ros::Publisher pub_ste = nh.advertise<std_msgs::Int16>("/ctrl/steering", 1);
	ros::Publisher pub_vel = nh.advertise<std_msgs::Int16>("/ctrl/speed", 1);

	trajectory_available = false;

	// current_point = 0;
	// current_ctrl = 0;

	// ros::topic::waitForMessage<motion_planning::car_trajectory>("/motion_planning/path");
	ros::spin();

	ROS_INFO_STREAM("Finishing...");

}
