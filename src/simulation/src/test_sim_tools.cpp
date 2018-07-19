#include "sim_tools.cpp"
#include "sparse_rrt.cpp"
////////////////////////////////////////////////////////////////////////////////
//               methods only for testing the functions
////////////////////////////////////////////////////////////////////////////////

gazebo_msgs::ModelStates model_states;
std::string points_creation;
geometry_msgs::Point initial_pt, end_pt;
int path_counter;

void get_model_states(const gazebo_msgs::ModelStates& msg)
{

    model_states = msg;
    // gazebo_msgs::ModelStates new_msg;
    // std::vector<geometry_msgs::Pose> obstacles_pos;
    // std::vector<geometry_msgs::Twist> obstacles_twist;
    //
    // get_obstacles_poses(new_msg);
    //
    // delete_markers();
    // std::vector<geometry_msgs::Point> points = generate_grid(.5, .5);
    // std::vector<geometry_msgs::Point> points_w_obs = remove_obst_points(points);
    // geometry_msgs::Point ini_p;
    // geometry_msgs::Point end_p;
    // ini_p.x = -10;
    // ini_p.y = -7.5;
    // end_p.x = -1;
    // end_p.y = 9;
    //
    // a_star(points_w_obs, ini_p, end_p);
}

void create_path()
{
  std::cout << path_counter << ": creating new path" << '\n';
  std::vector<geometry_msgs::Pose> obstacles_pos;
  std::vector<geometry_msgs::Twist> obstacles_twist;

  path_counter++;

  get_obstacles_poses(model_states);

  // delete_markers();

  // geometry_msgs::Point ini_pt;
  // ini_pt.x = -10;
  // ini_pt.y = -7.5;

  std::vector<geometry_msgs::Point> points;
  std::vector<geometry_msgs::Point> points_w_obs;

  // std::cout << "points:" << points_creation << '\n';
  if (points_creation == GRID)
  {
    points = generate_grid(.5, .5);
    points_w_obs = remove_obst_points(points);
    set_points_without_obstacles(points_w_obs);
    a_star(initial_pt, end_pt, points_creation);
  }
  else if(points_creation == CTRL_6)
  {
    a_star(initial_pt, end_pt, points_creation);
  }
  else if(points_creation == RRT)
  {
    std::cout << "RRT" << '\n';
  }
  else
  {
    ROS_FATAL_STREAM("parameter not valid: " << points_creation);
    ros::shutdown();
  }



  // a_star(points_w_obs, ini_p, end_p);
}

int main(int argc, char **argv)
{
    int rate_hz = 10;
    ros::init(argc, argv, "sim_tools_testing_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Rate loop_rate(rate_hz);
    path_counter = 0;

    nh_priv.param<std::string>("points_creation", points_creation, "GRID");
    nh_priv.param<double>("initial_point_x", initial_pt.x, -10);
    nh_priv.param<double>("initial_point_y", initial_pt.y, -7.5);
    nh_priv.param<double>("final_point_x", end_pt.x, -1);
    nh_priv.param<double>("final_point_y", end_pt.y, 9);
    ros::Subscriber sub_model_states  = nh.subscribe("/gazebo/model_states", 1, &get_model_states);

    // ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("/target_pose", 1);

    ROS_INFO_STREAM("sim_tools_testing_node initiated");
    ros::spinOnce();

    while(ros::ok())
    {
        ros::spinOnce();
        if (model_states.name.size() > 0)
        {
          create_path();
        }
        loop_rate.sleep();
    }
    return 0;
}
