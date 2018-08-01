// #define _GLIBCXX_USE_CXX11_ABI 0
#include "a_star.h"
#include "autonomos.hpp"
#include "rrt_ros.hpp"
#include "sst_ros.hpp"

// ros
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

// sparce rrt
#include "utilities/parameter_reader.hpp"
#include "utilities/condition_check.hpp"
#include "utilities/random.hpp"
#include "utilities/timer.hpp"
#include "systems/point.hpp"
#include "systems/car.hpp"
// #include "motion_planners/sst.hpp"
// #include "motion_planners/rrt.hpp"
#include "utilities/parameter_reader.hpp"


#define RRT "RRT"
#define SST "SST"

std::vector<geometry_msgs::Pose> model_states;
std::string algorithm;
geometry_msgs::Point initial_pt, end_pt;
std::vector<geometry_msgs::Pose> vec_obstacles_poses;
std::vector<int> vec_obstacles_type;
int path_counter;


geometry_msgs::Pose2D params_start_state, params_goal_state;

bool simulation;
int sim_iters;
std::vector<ros::Publisher> pub_gazebo_lines_visualizer;
ros::Publisher pub_target_pose;
ros::Publisher pub_start_pose;
int gz_total_lines;


double obstacles_radius;

std_msgs::Float64MultiArray vec_lines_closed;
std_msgs::Float64MultiArray vec_lines_path;
std_msgs::Float64MultiArray vec_lines_opened;

int dummy;
a_star_t* a_star_ptr;

planner_t* planner;


// functions
void publish_lines(bool dealloc);
void a_star_solve();
void create_path();
void create_path();
void get_obstacles_poses_callback(const geometry_msgs::PoseArray& msg);
void get_obstacles_types_callback(const std_msgs::Int64MultiArray& msg);
void rrt_sst_solver();

void a_star_solve()
{

  condition_check_t checker(params::stopping_type, params::stopping_check);
	condition_check_t* stats_check=NULL;
	if(stats_check!=0)
	{
		stats_check = new condition_check_t(params::stats_type, params::stats_check);
	}

	checker.reset();

  if(stats_check==NULL)
	{
		do
		{
			a_star_ptr->step();
      if (params::intermediate_visualization)
      {
        publish_lines(false);
        pub_target_pose.publish(params_goal_state);
        std::cout << "continue?" << '\n';
        std::cin >> dummy;
        if (!dummy)
        {
          ros::shutdown();
        }
      }
		}
		while(! a_star_ptr -> pose_reached() );


		std::vector<std::pair<double*,double> > controls;
		a_star_ptr->get_solution(controls);
		double solution_cost = 0;
		for(unsigned i=0;i<controls.size();i++)
		{
			solution_cost+=controls[i].second;
		}
    // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
		// std::cout << "Time: " << checker.time() << " Iterations: " <<
    //   checker.iterations() << " Nodes: " << a_star_ptr -> get_total_nodes() <<
    //   " Solution Quality: " << solution_cost << std::endl ;
		// a_star_ptr->visualize_tree(0);
		// a_star_ptr->visualize_nodes(0);
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
				a_star_ptr -> step();
				execution_done = checker.check();
				stats_print = stats_check->check();
			}
      while(! a_star_ptr -> pose_reached() );
			// while(!execution_done && !stats_print);
			if(stats_print)
			{
				std::vector<std::pair<double*, double> > controls;
				a_star_ptr -> get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				// std::cout << "Time: " << checker.time() << " Iterations: " <<
        //   checker.iterations() << " Nodes: " << a_star_ptr -> get_total_nodes() <<
        //   " Solution Quality: " << solution_cost << std::endl ;
				stats_print = false;
				// if(params::intermediate_visualization)
				// {
				// 	a_star_ptr->visualize_tree(count);
				// 	a_star_ptr->visualize_nodes(count);
				// 	count++;
				// }
				stats_check->reset();
			}
			if (execution_done)
			{
				// std::vector<std::pair<double*,double> > controls;
				// a_star_ptr->get_solution(controls);
				// double solution_cost = 0;
				// for(unsigned i=0;i<controls.size();i++)
				// {
				// 	solution_cost+=controls[i].second;
				// }
				// std::cout << "Time: " << checker.time() << " Iterations: " <<
        //   checker.iterations() << " Nodes: " << a_star_ptr -> get_total_nodes() <<
        //   " Solution Quality: " << solution_cost << std::endl ;
				// a_star_ptr -> visualize_tree(count);
				// a_star_ptr -> visualize_nodes(count);
				break;
			}
		}
	}
}

void create_path()
{
  std::cout << path_counter << ": creating new path for " << params::planner << std::endl;
  std::vector<geometry_msgs::Pose> obstacles_pos;
  std::vector<geometry_msgs::Twist> obstacles_twist;

  path_counter++;

  std::vector<geometry_msgs::Point> points;
  std::vector<geometry_msgs::Point> points_w_obs;

  if (params::planner == GRID)
  {

    // a_star_ptr = new a_star_t();
    a_star_ptr -> set_type(params::planner);
    a_star_ptr -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);
    points = a_star_ptr -> generate_grid(.5, .5, -10, -10, 0, 10);
    a_star_ptr -> remove_obst_points(points);
    a_star_solve();
    // vec_lines_closed = a_star_ptr -> get_closed_lines();

    // a_star_ptr(initial_pt, end_pt, planner);
  }
  else if(params::planner == CTRL)
  {
    // a_star_ptr = new a_star_t();
    a_star_ptr -> reset_nodes();
    a_star_ptr -> setup_planning();
    a_star_ptr -> set_type(params::planner);
    a_star_ptr -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);
    // printf("start: (%.1f, %.1f, %.1f)\n", start_state.x, start_state.y, start_state.theta );
    // printf("goal: (%.1f, %.1f, %.1f) +- %.1f\n", goal_state.x, goal_state.y, goal_state.theta, goal_radius );
    a_star_ptr -> set_start_state(params_start_state);
    a_star_ptr -> set_goal_state(params_goal_state, params::goal_radius);
    a_star_solve();

    // if (simulation)
    // {
    //   vec_lines_path   = a_star_ptr -> get_path_lines();
    //   vec_lines_opened = a_star_ptr -> get_opened_lines();
    //   vec_lines_closed = a_star_ptr -> get_closed_lines();
    // }
    // a_star(initial_pt, end_pt, planner);
  }
  else if (params::planner == RRT)
  {
    autonomos_t* system_aux = new autonomos_t();
    system_aux -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);
    planner = new rrt_ros_t(system_aux);
    rrt_sst_solver();
  }
  else if (params::planner == SST)
  {
    autonomos_t* system_aux = new autonomos_t();
    system_aux -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);
    planner = new sst_ros_t(system_aux);
    rrt_sst_solver();
  }
  else
  {
    ROS_FATAL_STREAM("parameter not valid: " << params::planner);
    ros::shutdown();
  }

}

void rrt_sst_solver()
{
  // double* params_start_state_ = new double[3];
  // double* params_goal_state_ = new double[3];
  // params_start_state_[0] = 0;//start_state.x;
  // params_start_state_[1] = 0;//start_state.y;
  // // start_state_[2] = start_state.theta;
  // params_goal_state_[0] = 9; //goal_state.x;
  // params_goal_state_[1] = 9; //goal_state.y;
  // // goal_state_[2] = goal_state.theta;
  params::goal_radius = .5;
  planner->set_start_state(params::start_state);
	planner->set_goal_state(params::goal_state, params::goal_radius);
	planner->setup_planning();
  // dummy variables
  int dummy_cont;

  // std::cin >> dummy_cont;

	condition_check_t checker(params::stopping_type, params::stopping_check);
	condition_check_t* stats_check=NULL;
	if(params::stats_check != 0)
	{
		stats_check = new condition_check_t(params::stats_type, params::stats_check);
	}
  checker.reset();
	std::cout << "Starting the planner: " << params::planner << " for the system: "
    << params::system << std::endl;
	if(stats_check==NULL)
	{
		do
		{
			planner->step();
      // std::cout << "line: " << __LINE__ << "\tcont: " << dummy_cont << '\n';
      // dummy_cont++;
		}
		while(!checker.check());
		std::vector<std::pair<double*,double> > controls;
		planner->get_solution(controls);
		double solution_cost = 0;
    // std::cout << "controls size: " << controls.size() << '\n';

		for(unsigned i = 0; i < controls.size(); i++)
		{
			solution_cost += controls[i].second;
		}
    std::cout << "Planner:\t" << params::planner << "\tTime:\t" <<
      checker.time() << "\tIterations:\t" << checker.iterations() << "\tNodes\t"
      << planner -> number_of_nodes << "\tSolution Quality\t" << solution_cost
      << std::endl ;
		planner -> visualize_tree(0);
		planner -> visualize_nodes(0);

    // planner.root;
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
				planner->step();
				execution_done = checker.check();
				stats_print = stats_check->check();
			}
			while(!execution_done && !stats_print);
			if(stats_print)
			{
				std::vector<std::pair<double*,double> > controls;
				planner->get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost += controls[i].second;
				}
				std::cout << "Time: " << checker.time() << " Iterations: " <<
          checker.iterations() << " Nodes: " << planner -> number_of_nodes <<
          " Solution Quality: " << solution_cost << std::endl ;
				stats_print = false;
				if(params::intermediate_visualization)
				{
					planner->visualize_tree(count);
					planner->visualize_nodes(count);
					count++;
				}
				stats_check->reset();
			}
			if (execution_done)
			{
				std::vector<std::pair<double*,double> > controls;
				planner->get_solution(controls);
				double solution_cost = 0;
        std::cout << "controls size: " << controls.size() << '\n';
				for(unsigned i = 0; i < controls.size(); i++)
				{
					solution_cost+=controls[i].second;
				}
				std::cout << "Time: " << checker.time() << " Iterations: " <<
          checker.iterations() << " Nodes: " << planner -> number_of_nodes <<
          " Solution Quality: " << solution_cost << std::endl ;
				planner->visualize_tree(count);
				planner->visualize_nodes(count);
				break;
			}
		}
	}
	std::cout << "Done planning." << std::endl;
}

void get_obstacles_poses_callback(const geometry_msgs::PoseArray& msg)
{
  std::cout << "Line: " << __LINE__ << '\n';
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

void publish_lines(bool dealloc)
{
  // std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << '\n';
  if (params::planner == CTRL)
  {
    vec_lines_path   = a_star_ptr -> get_path_lines();
    vec_lines_opened = a_star_ptr -> get_opened_lines();
    vec_lines_closed = a_star_ptr -> get_closed_lines();
    pub_gazebo_lines_visualizer[0].publish(vec_lines_path);
    pub_gazebo_lines_visualizer[1].publish(vec_lines_opened);
    pub_gazebo_lines_visualizer[2].publish(vec_lines_closed);
  }
  else if (params::planner == RRT)
  {
    // dynamic_cast<Derived<int> *>
    rrt_ros_t *rrt_aux = dynamic_cast<rrt_ros_t*>(planner);
    pub_gazebo_lines_visualizer[0].publish(rrt_aux -> get_vector_path());
    pub_gazebo_lines_visualizer[1].publish(rrt_aux -> get_vector_tree());
  }
  else if (params::planner == SST)
  {
    // dynamic_cast<Derived<int> *>
    sst_ros_t *sst_aux = dynamic_cast<sst_ros_t*>(planner);
    pub_gazebo_lines_visualizer[0].publish(sst_aux -> get_vector_path());
    pub_gazebo_lines_visualizer[1].publish(sst_aux -> get_vector_tree());
    if (dealloc)
    {
      sst_aux -> dealloc_tree();
      delete planner;
    }
  }
  pub_target_pose.publish(params_goal_state);
  pub_start_pose.publish(params_start_state);
}


int main(int argc, char **argv)
{
    int rate_hz = 10;
    ros::init(argc, argv, "sim_tools_testing_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Rate loop_rate(rate_hz);
    path_counter = 0;

    int min_time_steps_aux, max_time_steps_aux;

    // motion planning parameters
    nh_priv.param<double>      ("integration_step", params::integration_step, .002);
  	nh_priv.param<std::string> ("stopping_type",    params::stopping_type, "time");
  	nh_priv.param<double>      ("stopping_check",   params::stopping_check, 15);
  	nh_priv.param<std::string> ("stats_type",       params::stats_type, "time");
  	nh_priv.param<double>      ("stats_check",      params::stats_check, 0);
  	nh_priv.param<int>         ("min_time_steps",   min_time_steps_aux, 20);
  	nh_priv.param<int>         ("max_time_steps",   max_time_steps_aux, 200);
  	nh_priv.param<int>         ("random_seed",      params::random_seed, 0);
  	nh_priv.param<double>      ("sst_delta_near",   params::sst_delta_near, 0.4);
  	nh_priv.param<double>      ("sst_delta_drain",  params::sst_delta_drain, 0.2);
  	nh_priv.param<std::string> ("planner",          params::planner, CTRL);
  	nh_priv.param<std::string> ("system",           params::system, "car");
  	nh_priv.param<double>      ("goal_radius",      params::goal_radius, 0.5);
  	nh_priv.param<bool>        ("intermediate_visualization",
      params::intermediate_visualization, false);
    nh_priv.param<double>     ("start_state/x",     params_start_state.x, -10);
    nh_priv.param<double>     ("start_state/y",     params_start_state.y, -7.5);
    nh_priv.param<double>     ("start_state/theta", params_start_state.theta, 0);
    nh_priv.param<double>     ("goal_state/x",      params_goal_state.x, -1);
    nh_priv.param<double>     ("goal_state/y",      params_goal_state.y, 9);
    nh_priv.param<double>     ("goal_state/theta",  params_goal_state.theta, 0);


    // global parameters
    nh.param<bool>            ("simulation/simulation",            simulation,  true);
    nh.param<int>             ("simulation/iterations", sim_iters,  1);
    nh.param<int>             ("simulation/gz_total_lines", gz_total_lines,  0);
    nh.param<double>          ("obstacles_radius",      obstacles_radius,  0);

    // params::integration_step = params_integration_step;
  	// params::stopping_type = params_stopping_type;
  	// params::stopping_check = params_stopping_check;
  	// params::stats_type = params_stats_type;
  	// params::stats_check = params_stats_check;
  	// params::intermediate_visualization = params_intermediate_visualization;
  	// params::min_time_steps = params_min_time_steps;
  	// params::max_time_steps = params_max_time_steps;
  	// params::random_seed = params_random_seed;
  	// params::sst_delta_near = params_sst_delta_near;
  	// params::sst_delta_drain = params_sst_delta_drain;
  	// params::planner = params_planner;
  	// params::system = params_system;
    // params::goal_radius = params_goal_radius;
  	// params::start_state = params_start_state;

    params::min_time_steps = min_time_steps_aux;
  	params::max_time_steps = max_time_steps_aux;

    // if (params::planner == RRT || params::planner == SST)
    // {
    //   params_start_state.theta += M_PI / 2;
    //   params_goal_state.theta += M_PI / 2;
    // }

    params::start_state = new double[3];
    params::start_state[0] = params_start_state.x;
    params::start_state[1] = params_start_state.y;
    params::start_state[2] = params_start_state.theta;

    params::goal_state = new double[3];
    params::goal_state[0] = params_goal_state.x;
    params::goal_state[1] = params_goal_state.y;
    params::goal_state[2] = params_goal_state.theta;
  	// params::goal_state = params_goal_state;


  	//Parameters for image output.
  	extern double tree_line_width;
  	extern double solution_line_width;
  	extern int image_width;
  	extern int image_height;
  	extern double node_diameter;
  	extern double solution_node_diameter;


    ros::Subscriber sub_obs_poses  = nh.subscribe("/obstacles/poses", 1,
      &get_obstacles_poses_callback);
    ros::Subscriber sub_obs_types  = nh.subscribe("/obstacles/types", 1,
      &get_obstacles_types_callback);

    pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("/goal_pose", 1);
    pub_start_pose = nh.advertise<geometry_msgs::Pose2D>("/start_pose", 1);
    // ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("/target_pose", 1);

    if (simulation)
    {

      for(int i = 0; i < gz_total_lines; i++)
      {
        std::stringstream i_ss;
        i_ss << "/gz_visual/lines_" << i;
	      pub_gazebo_lines_visualizer.push_back(
          nh.advertise<std_msgs::Float64MultiArray>(i_ss.str(), rate_hz));
      }
    }

    std_srvs::Empty empty;

    ROS_INFO_STREAM("sim_tools_testing_node initiated");
    ros::spinOnce();

    if(params::planner == CTRL || params::planner == GRID)
    {
      a_star_ptr = new a_star_t();
    }
    ros::spinOnce();
    loop_rate.sleep();
    int iters = 0;
    // for (size_t iters = 0; iters < sim_iters; iters++)
    // while (iters < sim_iters)
    while(ros::ok())
    {
      ros::spinOnce();

      // std::cout << "iteration: " << iters << "\tobs: " << vec_obstacles_poses.size() << '\n';
        ros::service::call("/gazebo/reset_simulation", empty);
        if (vec_obstacles_poses.size() > 0)
        {
          create_path();
          publish_lines(true);
          // std::cin >> dummy;
          iters++;
        }
        if (iters == sim_iters)
        {
          break;
        }
        else
        {
          loop_rate.sleep();
        }
    }
    return 0;
}

























///////////////////////////////////////////////////////////////////////////////
//                                  TODO
///////////////////////////////////////////////////////////////////////////////
// TODO: implement the collision_detector on autonomos.cpp to use with sparcerrt
