#include "follow_trajectory.h"


void get_path_callback(const motion_planning::car_trajectory msg)
{
	ROS_WARN("getting new path at: %.3f", ros::Time::now().toSec());
	path = msg;
	for (int i = 0; i < msg.path_len; ++i)
	{
		path.speed[i] 	 = msg.speed[i];		
		path.steering[i] = msg.steering[i];
		path.duration[i] = msg.duration[i];		
		ROS_DEBUG_NAMED("trayectory_callback", "New traj => Seg: %d/%d \tvel: %.3f \tste: %.3f \tt: %.3f", 
			i, msg.path_len, msg.speed[i], msg.steering[i], msg.duration[i]);
	}
	path.path_len = msg.path_len;
	trajectory_available = msg.path_len > 0;
	next = 0;
}

bool get_next_trajectory_segment(navigation::trajectory_segment::Request &req,
								 navigation::trajectory_segment::Response &res)
{
	bool valid;
	if (trajectory_available && next < path.path_len)
	{
		valid = true;
		res.seq 	 = seq;
		res.speed    = path.speed[next];
		res.steering = path.steering[next];
		res.duration = path.duration[next];
		next++;
		seq++;
		ROS_WARN_NAMED("trayectory_service", "%.3f] Seg: %d/%d \tvel: %.3f \tste: %.3f \tt: %.3f", 
			ros::Time::now().toSec(), next, path.path_len, res.speed, res.steering, res.duration);
	}
	else 
	{
		valid = false;
		res.speed = 0;
		res.steering = 0;
		res.duration = 0;
	}
	return valid;
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
