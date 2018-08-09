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

// own
#include "motion_planning/ctrl_path.h"
#include "sim_params.h"

#define RRT "RRT"
#define SST "SST"

namespace sim_params
{
	bool simulation;
	int sim_iters;
	int gz_total_lines;
  bool plot_lines;
	bool publish_ctrl_path;
}

std::vector<geometry_msgs::Pose> model_states;
std::string algorithm;
geometry_msgs::Point initial_pt, end_pt;
std::vector<geometry_msgs::Pose> vec_obstacles_poses;
std::vector<int> vec_obstacles_type;
int path_counter;


geometry_msgs::Pose2D params_start_state, params_goal_state;

// sim_params::simulation = true;
// sim_params::publish_ctrl_path = true;
// sim_params::gz_total_lines = 0;
// sim_params::plot_lines = false;

std::vector<ros::Publisher> pub_gazebo_lines_visualizer;
ros::Publisher pub_target_pose;
ros::Publisher pub_start_pose;
ros::Publisher pub_ctrl_path;

std::string ctrl_to_use;

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
void publish_ctrl_path(std::vector<std::pair<double*,double> >& controls);


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
    autonomos_t* system_aux = new autonomos_t(ctrl_to_use);
    system_aux -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);
    planner = new rrt_ros_t(system_aux);
    rrt_sst_solver();
  }
  else if (params::planner == SST)
  {
    autonomos_t* system_aux = new autonomos_t(ctrl_to_use);

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

  params::goal_radius = .5;
  planner->set_start_state(params::start_state);
	planner->set_goal_state(params::goal_state, params::goal_radius);
	planner->setup_planning();
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
		}
		while(!checker.check());
		std::vector<std::pair<double*,double> > controls;
		planner -> get_solution(controls);
		double solution_cost = 0;

		for(unsigned i = 0; i < controls.size(); i++)
		{
			solution_cost += controls[i].second;
		}
    if (sim_params::publish_ctrl_path)
    {
      publish_ctrl_path(controls);
    }
    std::cout << "Planner:\t" << params::planner << "\tTime:\t" <<
      checker.time() << "\tIterations:\t" << checker.iterations() << "\tNodes:\t"
      << planner -> number_of_nodes << "\tSolution Quality:\t" << solution_cost
      << "\tcontroller:\t" << ctrl_to_use << std::endl ;
		planner -> visualize_tree(0);
		planner -> visualize_nodes(0);

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
				planner -> get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost += controls[i].second;
				}
        // sst_ros_t *sst_aux = dynamic_cast<sst_ros_t*>(planner);
        publish_ctrl_path(controls);
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

void publish_ctrl_path(std::vector<std::pair<double*,double> >& controls)
{
  std::cout << __PRETTY_FUNCTION__ << '\n';
  motion_planning::ctrl_path res;
  for(unsigned i = 0; i < controls.size(); i++)
  {
    res.speed.push_back(controls[i].first[0]);
    res.steering.push_back(controls[i].first[1]);
    res.duration.push_back(controls[i].second);
  }
  pub_ctrl_path.publish(res);

}

void get_obstacles_poses_callback(const geometry_msgs::PoseArray& msg)
{
  std::cout << "Line: " << __LINE__ << '\n';
  for (auto e : msg.poses )
  {
    vec_obstacles_poses.push_back(e);
  }
}

void get_obstacles_types_callback(const std_msgs::Int64MultiArray& msg)
{
  for (auto e: msg.data )
  {
    vec_obstacles_type.push_back(e);
  }
}

void publish_lines(bool dealloc)
{
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
    rrt_ros_t *rrt_aux = dynamic_cast<rrt_ros_t*>(planner);
    pub_gazebo_lines_visualizer[0].publish(rrt_aux -> get_vector_path());
    pub_gazebo_lines_visualizer[1].publish(rrt_aux -> get_vector_tree());
    if (dealloc)
    {
      delete planner;
    }
  }
  else if (params::planner == SST)
  {
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
    std_srvs::Empty empty;
    ros::service::call("/gazebo/reset_simulation", empty);

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

    nh_priv.param<std::string>     ("ctrl_to_use",  ctrl_to_use, RANDOM_CTRL);


    // global parameters
    nh.param<bool>  ("simulation/simulation", sim_params::simulation,  true);
    nh.param<bool>  ("simulation/publish_ctrl_path", sim_params::publish_ctrl_path,  true);
    nh.param<int>   ("simulation/gz_total_lines", sim_params::gz_total_lines,  0);
    nh.param<bool>  ("simulation/plot_lines", sim_params::plot_lines,  false);
    nh.param<int>   ("simulation/iterations", sim_params::sim_iters,  1);

    nh.param<double>("obstacles_radius",      obstacles_radius,  0);


    params::min_time_steps = min_time_steps_aux;
  	params::max_time_steps = max_time_steps_aux;

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
    pub_ctrl_path = nh.advertise<motion_planning::ctrl_path>(
      "/motion_planning/path", 1, true);

    if (sim_params::simulation)
    {

      for(int i = 0; i < sim_params::gz_total_lines; i++)
      {
        std::stringstream i_ss;
        i_ss << "/gz_visual/lines_" << i;
	      pub_gazebo_lines_visualizer.push_back(
          nh.advertise<std_msgs::Float64MultiArray>(i_ss.str(), rate_hz, true));
      }
    }


    ROS_INFO_STREAM("sim_tools_testing_node initiated");
    // ros::spinOnce();

    if(params::planner == CTRL || params::planner == GRID)
    {
      a_star_ptr = new a_star_t();
    }
    // ros::spinOnce();
    loop_rate.sleep();
    int iters = 0;
    while(ros::ok())
    {
      ros::service::call("/gazebo/reset_simulation", empty);
      ros::spinOnce();

      // std::cout << "iteration: " << iters << "\tobs: " << vec_obstacles_poses.size() << '\n';
        if (vec_obstacles_poses.size() > 0)
        {
          create_path();
          if (sim_params::plot_lines)
          {
            publish_lines(true);
          }
          // std::cin >> dummy;
          iters++;
        }
        if (iters == sim_params::sim_iters)
        {
          if (publish_ctrl_path)
          {
            std::cout << "Input something and press enter to finish process" << '\n';
            //std::cin >> dummy;
          }
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
// TODO:  Change the ctrl_to_use from int to string and use an array at autonomos
//
