#include "sim_tools.cpp"

////////////////////////////////////////////////////////////////////////////////
//               methods only for testing the functions
////////////////////////////////////////////////////////////////////////////////

void get_model_states(const gazebo_msgs::ModelStates& msg)
{
    ROS_INFO_STREAM("At: " << __PRETTY_FUNCTION__);

    gazebo_msgs::ModelStates new_msg = msg;
    std::vector<geometry_msgs::Pose> obstacles_pos;
    std::vector<geometry_msgs::Twist> obstacles_twist;


    // copy(msg.begin(), msg.end(), new_msg);
    get_obstacles_poses(new_msg);
    // for (auto element : obstacles_pos)
    // {
    //   ROS_INFO_STREAM("Pose: " << element);
    // }
    std::vector<geometry_msgs::Point> points = generate_grid(.5, .5);
    std::vector<geometry_msgs::Point> points_w_obs = remove_obst_points(points);
    paint_points(points_w_obs);
}

int main(int argc, char **argv)
{
    int rate_hz = 10;
    ros::init(argc, argv, "sim_tools_testing_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(rate_hz);

    ros::Subscriber sub_model_states  = nh.subscribe("/gazebo/model_states", 1, &get_model_states);

    // ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("/target_pose", 1);
    ROS_INFO_STREAM("sim_tools_testing_node initiated");
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
