// #define _GLIBCXX_USE_CXX11_ABI 0

// std
#include <chrono>

#include "a_star.h"
#include "autonomos.hpp"
#include "rrt_ros.hpp"
#include "sst_ros.hpp"
#include "dirt_ros.hpp"

// ros
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

// sparce rrt lib
#include "utilities/timer.hpp"
#include "utilities/random.hpp"
#include "utilities/condition_check.hpp"

// own
#include "motion_planning/car_trajectory.h"
#include "motion_planning/Line_Segment.h"
#include "sim_params.h"

#define RRT "RRT"
#define SST "SST"
#define DIRT "DIRT"

using std::string;
using std::cout;
using std::endl;

namespace sim_params
{
	bool simulation;
	int sim_iters;
	int gz_total_lines;
  bool plot_lines;
	bool publish_car_trajectory;
  string model_name;
}

namespace params
{
  double integration_step;
  std::string stopping_type;
  double stopping_check;
  std::string stats_type;
  double stats_check;
  bool intermediate_visualization;
  unsigned min_time_steps;
  unsigned max_time_steps;
  int random_seed;
  double sst_delta_near;
  double sst_delta_drain;
  std::string planner;
  std::string system;
  double* start_state;
  double* goal_state;
  double goal_radius;

}

///////////////
// VARIABLES //
///////////////
const int rate_hz = 10;

int dummy;
int path_counter;
int iters = 0;

std::chrono::high_resolution_clock::time_point start_time;

double obstacles_radius;

std::vector<int> vec_obstacles_type;
std::vector<geometry_msgs::Pose> model_states;
std::vector<geometry_msgs::Pose> vec_obstacles_poses;
std::vector<ros::Publisher> pub_gazebo_lines_visualizer;

std::string algorithm;
std::string ctrl_to_use;

std_msgs::Float64MultiArray vec_lines_closed;
std_msgs::Float64MultiArray vec_lines_path;
std_msgs::Float64MultiArray vec_lines_opened;

geometry_msgs::Point initial_pt, end_pt;
geometry_msgs::Pose2D params_start_state, params_goal_state;

ros::Publisher pub_target_pose;
ros::Publisher pub_start_pose;
ros::Publisher pub_car_trajectory;
ros::Publisher pub_sim_line;

a_star_t* a_star_ptr;

planner_t* planner;

motion_planning::car_trajectory car_trajectory_msg;

///////////////
// FUNCTIONS //
///////////////
void publish_lines(bool dealloc);
void a_star_solve();
void create_path();
void create_path();
void get_obstacles_poses_callback(const geometry_msgs::PoseArray& msg);
void get_obstacles_types_callback(const std_msgs::Int64MultiArray& msg);
void rrt_sst_solver();
void publish_car_trajectory(std::vector<std::pair<double*,double> >& controls);
void publish_sln_trajectory();
void publish_sln_tree(tree_node_t* node);
void publish_sln_tree_1(tree_node_t* node, motion_planning::Line_Segment& ls_traj);


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
    autonomos_t* system_aux = new autonomos_t(ctrl_to_use, true);
    system_aux -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);
    planner = new rrt_ros_t(system_aux);
    rrt_sst_solver();
  }
  else if (params::planner == SST)
  {
    autonomos_t* system_aux = new autonomos_t(ctrl_to_use, true);

    system_aux -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);
    planner = new sst_ros_t(system_aux);
    rrt_sst_solver();
  }
  else if(params::planner == DIRT)
  {
    autonomos_t* system_aux = new autonomos_t(ctrl_to_use, true);
    system_aux -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);
    planner = new dirt_ros_t(system_aux);
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

  params::goal_radius = .1;
  planner->set_start_state(params::start_state);
	planner->set_goal_state(params::goal_state, params::goal_radius);
  ROS_WARN_STREAM("Start: (" << params::start_state[0] << ", " << params::start_state[1] << ", " << params::start_state[2] << ")");
  ROS_WARN_STREAM("Goal:  (" << params::goal_state[0] << ", " << params::goal_state[1] << ", " << params::goal_state[2] << ")");
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
    if (sim_params::publish_car_trajectory)
    {
      publish_car_trajectory(controls);
    }
    std::cout << "Planner:\t" << params::planner << "\tTime:\t" <<
      checker.time() << "\tIterations:\t" << checker.iterations() << "\tNodes:\t"
      << planner -> number_of_nodes << "\tSolution Quality:\t" << solution_cost
      << "\tcontroller:\t" << ctrl_to_use << std::endl ;
		// planner -> visualize_tree(0);
		// planner -> visualize_nodes(0);

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
        publish_car_trajectory(controls);
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

void publish_car_trajectory(std::vector<std::pair<double*,double> >& controls)
{
  std::cout << __PRETTY_FUNCTION__ << '\n';
  // motion_planning::car_trajectory res;

  car_trajectory_msg.path_len = controls.size();
  for(unsigned i = 0; i < controls.size(); i++)
  {
    car_trajectory_msg.speed.push_back(controls[i].first[0]);
    car_trajectory_msg.steering.push_back(controls[i].first[1]);
    car_trajectory_msg.duration.push_back(controls[i].second);
  }
  pub_car_trajectory.publish(car_trajectory_msg);

  car_trajectory_msg.header.seq++;
}

void get_obstacles_poses_callback(const geometry_msgs::PoseArray& msg)
{
  // std::cout << "Line: " << __LINE__ << '\n';
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

void publish_sln_tree(tree_node_t* node)
{
  motion_planning::Line_Segment ls_traj;
  ls_traj.color = "Gazebo/Purple";
  ls_traj.header.seq = 0;
  ls_traj.line_list = false;

  ls_traj.header.stamp = ros::Time::now();
  ls_traj.header.frame_id = "tree_seg_" + std::to_string(iters) ;
  publish_sln_tree_1(node, ls_traj);
  pub_sim_line.publish(ls_traj);

}

void publish_sln_tree_1(tree_node_t* node, motion_planning::Line_Segment& ls_traj)
{
  // seq_ant += std::to_string(ls_traj.header.seq);
  
  geometry_msgs::Point32 point;

  for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
  {
    // svg::Polyline traj_line(svg::Stroke(params::tree_line_width, svg::Color::Blue));
    point.x = node -> point[0];
    point.y = node -> point[1];
    point.z = 0.01;
    ls_traj.points.push_back(point);
    point.x = (*i) -> point[0];
    point.y = (*i) -> point[1];
    point.z = 0.01;
    ls_traj.points.push_back(point);

    ROS_DEBUG_STREAM_NAMED("TRAJ_PUBLISHER","( " << node -> point[0] << ", " << node -> point[1] << ", " << node -> point[2] << " )" );
    publish_sln_tree_1(*i, ls_traj);
    // ls_traj.header.seq = ls_traj.header.seq + 1;

  }

}

void publish_sln_trajectory(std::vector<tree_node_t*>  sln)
{
  // ROS_WARN_STREAM("Printing solution:");
  motion_planning::Line_Segment ls_traj;
  geometry_msgs::Point32 point;

  ls_traj.color = "Gazebo/Green";
  // bool first_iter = true;
  ls_traj.header.seq = 0;
  ls_traj.line_list = true;
  for (auto&  node : sln) 
  {
    ROS_DEBUG_STREAM_NAMED("TRAJ_PUBLISHER","( " << node -> point[0] << ", " << node -> point[1] << ", " << node -> point[2] << " )" );
    ls_traj.header.stamp = ros::Time::now();
    ls_traj.header.frame_id = "traj_seg_" + std::to_string(iters);
    // cout << ls_traj.header.frame_id << endl;
    point.x = node -> point[0];
    point.y = node -> point[1];
    point.z = 0.1;
    ls_traj.points.push_back(point);

  }
  pub_sim_line.publish(ls_traj);

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
    std::vector<tree_node_t*>  sln;
    tree_node_t* root;

    rrt_aux -> get_last_solution_path(sln);
    root = rrt_aux -> get_root();
    publish_sln_tree(root);
    publish_sln_trajectory(sln);

    if (dealloc)
    {
      delete planner;
    }
  }
  else if (params::planner == SST)
  {
    sst_ros_t *sst_aux = dynamic_cast<sst_ros_t*>(planner);
    std::vector<tree_node_t*>  sln;
    sst_aux -> get_last_solution_path(sln);
    publish_sln_trajectory(sln);
    // pub_gazebo_lines_visualizer[0].publish(sst_aux -> get_vector_path());
    // pub_gazebo_lines_visualizer[1].publish(sst_aux -> get_vector_tree());
    if (dealloc)
    {
      sst_aux -> dealloc_tree();
      delete planner;
    }
  }
  else if (params::planner == DIRT)
  {
    dirt_ros_t *dirt_aux = dynamic_cast<dirt_ros_t*>(planner);
    std::vector<tree_node_t*>  sln;
    dirt_aux -> get_last_solution_path(sln);
    publish_sln_trajectory(sln);

    // pub_gazebo_lines_visualizer[0].publish(dirt_aux -> get_vector_path());
    // pub_gazebo_lines_visualizer[1].publish(dirt_aux -> get_vector_tree());
    
  }
  pub_target_pose.publish(params_goal_state);
  pub_start_pose.publish(params_start_state);
}

void init_variables()
{
  ROS_WARN_STREAM("MP NODE: " << __FUNCTION__ <<  ": beginning..." );

  // init outgoing control message 
  // start_time = std::chrono::high_resolution_clock::now();
  car_trajectory_msg.header.seq = 0;
  car_trajectory_msg.header.stamp.sec = 0;
  car_trajectory_msg.header.stamp.nsec = 0;
  car_trajectory_msg.header.frame_id = params::planner;

  // filling start_state
  params::start_state = new double[3];
  params::start_state[0] = params_start_state.x;
  params::start_state[1] = params_start_state.y;
  params::start_state[2] = params_start_state.theta;

  // filling goal_state
  params::goal_state = new double[3];
  params::goal_state[0] = params_goal_state.x;
  params::goal_state[1] = params_goal_state.y;
  params::goal_state[2] = params_goal_state.theta;

  //Parameters for image output.
  extern double tree_line_width;
  extern double solution_line_width;
  extern int image_width;
  extern int image_height;
  extern double node_diameter;
  extern double solution_node_diameter;

  // if using A*, init the planner
  if(params::planner == CTRL || params::planner == GRID)
  {
    a_star_ptr = new a_star_t();
  }
  ROS_WARN_STREAM("MP NODE: " << __FUNCTION__ <<  ": Ended" );
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_planning_node");
  ROS_WARN_STREAM("MP NODE: " <<  __FUNCTION__ << ": node initialized");
  ROS_WARN("MP NODE: debug node initialized");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Rate loop_rate(rate_hz);
  path_counter = 0;
  std_srvs::Empty empty;
  ros::service::call("/gazebo/reset_simulation", empty);

  int min_time_steps_aux, max_time_steps_aux;

  ROS_WARN_STREAM("MP NODE: " << __FUNCTION__ <<  ": Reading parameters");
  // motion planning parameters
  nh_priv.param<double>       ("integration_step", params::integration_step, .001);
  nh_priv.param<string>       ("stopping_type",    params::stopping_type, "time");
  nh_priv.param<double>       ("stopping_check",   params::stopping_check, 15);
  nh_priv.param<string>       ("stats_type",       params::stats_type, "time");
  nh_priv.param<double>       ("stats_check",      params::stats_check, 0);
  nh_priv.param<int>          ("min_time_steps",   min_time_steps_aux, 20);
  nh_priv.param<int>          ("max_time_steps",   max_time_steps_aux, 200);
  nh_priv.param<int>          ("random_seed",      params::random_seed, 0);
  nh_priv.param<double>       ("sst_delta_near",   params::sst_delta_near, 0.4);
  nh_priv.param<double>       ("sst_delta_drain",  params::sst_delta_drain, 0.2);
  nh_priv.param<string>       ("planner",          params::planner, CTRL);
  nh_priv.param<string>       ("system",           params::system, "car");
  nh_priv.param<double>       ("goal_radius",      params::goal_radius, 0.5);
  nh_priv.param<bool>         ("intermediate_visualization", params::intermediate_visualization, false);
  nh_priv.param<double>       ("start_state/x",     params_start_state.x, -10);
  nh_priv.param<double>       ("start_state/y",     params_start_state.y, -7.5);
  nh_priv.param<double>       ("start_state/theta", params_start_state.theta, 0);
  nh_priv.param<double>       ("goal_state/x",      params_goal_state.x, -1);
  nh_priv.param<double>       ("goal_state/y",      params_goal_state.y, 9);
  nh_priv.param<double>       ("goal_state/theta",  params_goal_state.theta, 0);
  nh_priv.param<string>       ("ctrl_to_use",  ctrl_to_use, RANDOM_CTRL);

  // global parameters

  std::map<string, string> map_sim;
  nh.param<bool>   ("simulation/simulation", sim_params::simulation,  true);
  nh.param<bool>   ("simulation/publish_car_trajectory", sim_params::publish_car_trajectory,  true);
  nh.param<int>    ("simulation/gz_total_lines", sim_params::gz_total_lines,  0);
  nh.param<bool>   ("simulation/plot_lines", sim_params::plot_lines,  false);
  nh.param<int>    ("simulation/iterations", sim_params::sim_iters,  1);
  nh.param<bool>   ("simulation/publish_ctrl_path", sim_params::publish_car_trajectory,  true);
  nh.param<string> ("simulation/model_name", sim_params::model_name,  "AutoNOMOS_mini");
  // nh.param<std::map>    ("simulation", map_sim);
  
  nh.param<double>("obstacles_radius",      obstacles_radius,  0);

  ROS_WARN_STREAM("MP NODE: " << __FUNCTION__ <<  ": Parameters read");
  std::cout << "Using the following simulation parameters:" << '\n';
  std::cout << "\tsimulation: " << (sim_params::simulation?"true":"false") << '\n';
  std::cout << "\tpublish_car_trajectory: " << (sim_params::publish_car_trajectory?"true":"false") << '\n';
  std::cout << "\tgz_total_lines: " << sim_params::gz_total_lines << '\n';
  std::cout << "\tplot_lines: " << (sim_params::plot_lines?"true":"false") << '\n';
  std::cout << "\tsim_iters: " << sim_params::sim_iters << '\n';

  params::min_time_steps = min_time_steps_aux;
  params::max_time_steps = max_time_steps_aux;

  ROS_WARN_STREAM("MP NODE: " << __FUNCTION__ <<  ": Initializing subscribers");
  ros::Subscriber sub_obs_poses  = nh.subscribe("/obstacles/poses", 1,
    &get_obstacles_poses_callback);
  ros::Subscriber sub_obs_types  = nh.subscribe("/obstacles/types", 1,
    &get_obstacles_types_callback);

  // publishers
  ROS_WARN_STREAM("MP NODE: " << __FUNCTION__ <<  ": Initializing publishers");
  pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("/goal_pose", 1, true);
  pub_start_pose = nh.advertise<geometry_msgs::Pose2D>("/start_pose", 1, true);
  pub_car_trajectory = nh.advertise<motion_planning::car_trajectory>(
    "/motion_planning/path", 1, true);
  if (sim_params::simulation)
  {
    string sim_topic_name = "/" + sim_params::model_name + "/gz_visual/line_segment";
    ROS_WARN_STREAM(sim_topic_name);
    pub_sim_line = nh.advertise<motion_planning::Line_Segment>(sim_topic_name, 100000, true);
  }

  init_variables();

  // ROS_WARN_STREAM("sim_tools_testing_node initiated");
  // ros::spinOnce();

  loop_rate.sleep();


  ROS_WARN_STREAM("MP NODE: " <<  __FUNCTION__ <<  ": Going into while(ros::ok())");
  while(ros::ok())
  {
    ros::service::call("/gazebo/reset_simulation", empty);
    ros::spinOnce();

    // std::cout << "iteration: " << iters << "\tobs: " << vec_obstacles_poses.size() << '\n';
    if (vec_obstacles_poses.size() >= 0)
    {
      create_path();
      if (sim_params::plot_lines)
      {
        std::cout << "Plotting lines..." << '\n';
        publish_lines(true);
      }
      // std::cin >> dummy;
      iters++;
    }
    if (iters == sim_params::sim_iters)
    {
      if (sim_params::publish_car_trajectory)
      {
        std::cout << "Input something and press enter to finish process" << '\n';
        std::cin >> dummy;
      }
      break;
    }
    else
    {
      loop_rate.sleep();
    }
  }
  std::cout << "Finishing motion_planning_node." << std::endl;
  return 0;
}

























///////////////////////////////////////////////////////////////////////////////
//                                  TODO
///////////////////////////////////////////////////////////////////////////////
// TODO:  Change the ctrl_to_use from int to string and use an array at autonomos
//
