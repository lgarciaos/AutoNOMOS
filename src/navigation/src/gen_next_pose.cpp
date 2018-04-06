#include "gen_next_pose.h"

void get_des_state(const std_msgs::Int16& val)
{
	state = val.data;
}

void get_rensac_left(const nav_msgs::GridCells& val)
{
	point_N_left = val.cells[N];
}

void get_rensac_center(const nav_msgs::GridCells& val)
{
	point_N_center = val.cells[N];
	// ROS_INFO_STREAM(val.cell_width);
}

void get_rensac_right(const nav_msgs::GridCells& val)
{
	point_N_right = val.cells[N];
}


geometry_msgs::Pose2D get_next_pose()
{ //De al edo deseado, calcular el siguiente punto obj de acuerdo al ctrl P del exámen :p
	// ROS_INFO_STREAM(__PRETTY_FUNCTION__);
	// ROS_INFO_STREAM("state: " << state);
	float x;
	float y;
	float diff_y;
	float diff_x;
	switch(state){
		case STATE_DONT_KNOW_LEFT : 
			next_pose.theta = 0;
			break;
		case STATE_OUTSIDE_LEFT : 
			next_pose.theta = 0;
			break;
		case STATE_LEFT_LEFT : 
			next_pose.theta = 0;
			break;
		case STATE_LEFT_CENTER : 
			next_pose.theta = 0;
			break;
		case STATE_CENTER_CENTER : 
			next_pose.theta = 0;
			break;
		case STATE_RIGHT_CENTER : 
			x = (point_N_right.x + point_N_center.x) / 2;
			y = (point_N_right.y + point_N_center.y) / 2;
			// y = point_N_right.y;
			diff_y = y - y_car;
			diff_x = x - x_car;
			next_pose.theta = fabs(atan2(diff_y, diff_x) * 180 / M_PI);
			ROS_INFO_STREAM("right: ( " << point_N_right.x << " , " << point_N_right.y << " )");	
			ROS_INFO_STREAM(" left: ( " << point_N_center.x << " , " << point_N_center.y << " )");	
			ROS_INFO_STREAM("  car: ( " << x_car << " , " << y_car << " )");	
			ROS_INFO_STREAM("des point: ( " << x << " , " << y << " )");	
			ROS_INFO_STREAM("diff point: ( " << diff_x << " , " << diff_y << " )");	
			ROS_INFO_STREAM("next_pose: " << next_pose.theta);

			break;
		case STATE_RIGHT_RIGHT : 
			next_pose.theta = 0;
			break;
		case STATE_OUTSIDE_RIGHT :
			next_pose.theta = 0; 
			break;
		case STATE_DONT_KNOW_RIGHT : 
			next_pose.theta = 0;
			break;		
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lane_states_node");
	ROS_INFO_STREAM("lane_states_node initialized");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");
	ros::Rate rate(rate_hz);

	ros::Subscriber sub_des_state = nh.subscribe("des_state",1, &get_des_state);
	ros::Subscriber sub_ransac_left = nh.subscribe("/points/ransac_left",1, &get_rensac_left);
	ros::Subscriber sub_ransac_center = nh.subscribe("/points/ransac_center",1, &get_rensac_center);
	ros::Subscriber sub_ransac_right = nh.subscribe("/points/ransac_right",1, &get_rensac_right);
	

	ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("/target_pose", 1);
	while(ros::ok())
	{
		pub_target_pose.publish(next_pose);
		get_next_pose();
		ros::spinOnce();
		rate.sleep();
	}
}