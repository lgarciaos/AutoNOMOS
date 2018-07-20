// #define _GLIBCXX_USE_CXX11_ABI 0
#include "a_star.h"

// ros
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int64MultiArray.h>

// sparce rrt
#include "utilities/parameter_reader.hpp"
#include "utilities/condition_check.hpp"
#include "utilities/random.hpp"
#include "utilities/timer.hpp"


#define RRT "RRT"

std::vector<geometry_msgs::Pose> model_states;
std::string algorithm;
geometry_msgs::Point initial_pt, end_pt;
std::vector<geometry_msgs::Pose> vec_obstacles_poses;
std::vector<int> vec_obstacles_type;
int path_counter;

double integration_step;
std::string stopping_type;
double stopping_check;
std::string stats_type;
double param_stats_check;
int min_time_steps;
int max_time_steps;
int random_seed;
double sst_delta_near;
double sst_delta_drain;
std::string planner;
std::string system_par;
geometry_msgs::Pose2D start_state;
geometry_msgs::Pose2D goal_state;
double goal_radius;
bool intermediate_visualization;

void a_star_solve(a_star_t* a_star)
{
  std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';

  condition_check_t checker(stopping_type,stopping_check);
	condition_check_t* stats_check=NULL;
	if(stats_check!=0)
	{
		stats_check = new condition_check_t(stats_type, param_stats_check);
	}
  std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';

	checker.reset();
	std::cout<<"Starting the planner: "<< planner << std::endl;

  std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
  if(stats_check==NULL)
	{
		do
		{
      std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
			a_star->step();
		}
		while(!checker.check());
		std::vector<std::pair<double*,double> > controls;
		a_star->get_solution(controls);
		double solution_cost = 0;
		for(unsigned i=0;i<controls.size();i++)
		{
			solution_cost+=controls[i].second;
		}
		// std::cout << "Time: " << checker.time() << " Iterations: " <<
    //   checker.iterations() << " Nodes: " << a_star -> get_total_nodes() <<
    //   " Solution Quality: " << solution_cost << std::endl ;
		// a_star->visualize_tree(0);
		// a_star->visualize_nodes(0);
	}
	else
	{
		int count = 0;
		bool execution_done = false;
		bool stats_print = false;
		while(true)
		{
			do
			{
				a_star->step();
				execution_done = checker.check();
				stats_print = stats_check->check();
			}
			while(!execution_done && !stats_print);
			if(stats_print)
			{
				std::vector<std::pair<double*, double> > controls;
				a_star->get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				// std::cout << "Time: " << checker.time() << " Iterations: " <<
        //   checker.iterations() << " Nodes: " << a_star -> get_total_nodes() <<
        //   " Solution Quality: " << solution_cost << std::endl ;
				stats_print = false;
				// if(params::intermediate_visualization)
				// {
				// 	a_star->visualize_tree(count);
				// 	a_star->visualize_nodes(count);
				// 	count++;
				// }
				stats_check->reset();
			}
			if (execution_done)
			{
				std::vector<std::pair<double*,double> > controls;
				a_star->get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				// std::cout << "Time: " << checker.time() << " Iterations: " <<
        //   checker.iterations() << " Nodes: " << a_star -> get_total_nodes() <<
        //   " Solution Quality: " << solution_cost << std::endl ;
				// a_star->visualize_tree(count);
				// a_star->visualize_nodes(count);
				break;
			}
		}
	}
}

void create_path()
{
  std::cout << path_counter << ": creating new path" << '\n';
  std::vector<geometry_msgs::Pose> obstacles_pos;
  std::vector<geometry_msgs::Twist> obstacles_twist;

  path_counter++;

  std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
  std::vector<geometry_msgs::Point> points;
  std::vector<geometry_msgs::Point> points_w_obs;

  if (planner == GRID)
  {
    a_star_t* a_star_grid = new a_star_t();
    a_star_grid -> set_type(planner);
    a_star_grid -> set_obstacles(vec_obstacles_poses, vec_obstacles_type);
    points = a_star_grid -> generate_grid(.5, .5, -10, -10, 0, 10);
    a_star_grid -> remove_obst_points(points);
    a_star_solve(a_star_grid);
    // a_star(initial_pt, end_pt, planner);
  }
  else if(planner == CTRL)
  {
    std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
    a_star_t* a_star_ctrl = new a_star_t();
    a_star_ctrl -> setup_planning();
    std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
    a_star_ctrl -> set_type(planner);
    std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
    a_star_ctrl -> set_obstacles(vec_obstacles_poses, vec_obstacles_type);
    std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
    a_star_ctrl -> set_start_state(start_state);
    std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
    a_star_ctrl -> set_goal_state(goal_state, goal_radius);
    std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
    a_star_solve(a_star_ctrl);
    // a_star(initial_pt, end_pt, planner);
  }
  else if(planner == RRT)
  {
    std::cout << "RRT" << '\n';
  }
  else
  {
    ROS_FATAL_STREAM("parameter not valid: " << planner);
    ros::shutdown();
  }

}

void get_obstacles_poses_callback(const geometry_msgs::PoseArray& msg)
{
  for (auto e : msg.poses )
  {
    vec_obstacles_poses.push_back(e);
  }
  // vec_obstacles_poses = msg.poses;
}

void get_obstacles_types_callback(const std_msgs::Int64MultiArray& msg)
{
  for (auto e: msg.data )
  {
    vec_obstacles_type.push_back(e);
  }
  // vec_obstacles_type = msg.data;
}

int main(int argc, char **argv)
{
    int rate_hz = 10;
    ros::init(argc, argv, "sim_tools_testing_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Rate loop_rate(rate_hz);
    path_counter = 0;

    // nh_priv.param<std::string>("points_creation", algorithm, "A_STAR_GRID");
    nh_priv.param<double>     ("initial_point_x",  initial_pt.x, -10);
    nh_priv.param<double>     ("initial_point_y",  initial_pt.y, -7.5);
    nh_priv.param<double>     ("final_point_x",    end_pt.x, -1);
    nh_priv.param<double>     ("final_point_y",    end_pt.y, 9);
    nh_priv.param<double>     ("integration_step", integration_step, .002);
  	nh_priv.param<std::string>("stopping_type",    stopping_type, "time");
  	nh_priv.param<double>     ("stopping_check",   stopping_check, 15);
  	nh_priv.param<std::string>("stats_type",       stats_type, "time");
  	nh_priv.param<double>     ("stats_check",      param_stats_check, 0);
  	nh_priv.param<int>        ("min_time_steps",   min_time_steps, 20);
  	nh_priv.param<int>        ("max_time_steps",   max_time_steps, 200);
  	nh_priv.param<int>        ("random_seed",      random_seed, 0);
  	nh_priv.param<double>     ("sst_delta_near",   sst_delta_near, 0.4);
  	nh_priv.param<double>     ("sst_delta_drain",  sst_delta_drain, 0.2);
  	nh_priv.param<std::string>("planner",          planner, "A_STAR_CTRL");
  	nh_priv.param<std::string>("system",           system_par, "point");
  	nh_priv.param<double>     ("goal_radius",      goal_radius, 0.5);
  	nh_priv.param<bool>       ("intermediate_visualization",
      intermediate_visualization, false);
    nh_priv.param<double>     ("start_state/x",     start_state.x, -10);
    nh_priv.param<double>     ("start_state/y",     start_state.y, -7.5);
    nh_priv.param<double>     ("start_state/theta", start_state.theta, 0);
    nh_priv.param<double>     ("goal_state/x",      goal_state.x, -1);
    nh_priv.param<double>     ("goal_state/y",      goal_state.y, 9);
    nh_priv.param<double>     ("goal_state/theta",  goal_state.theta, 0);

    ros::Subscriber sub_obs_poses  = nh.subscribe("/obstacles/poses", 1,
      &get_obstacles_poses_callback);
    ros::Subscriber sub_obs_types  = nh.subscribe("/obstacles/types", 1,
      &get_obstacles_types_callback);

    // ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("/target_pose", 1);

    ROS_INFO_STREAM("sim_tools_testing_node initiated");
    ros::spinOnce();

    while(ros::ok())
    {
        ros::spinOnce();
        if (vec_obstacles_poses.size() > 0)
        {
          create_path();
        }
        loop_rate.sleep();
    }
    return 0;
}
