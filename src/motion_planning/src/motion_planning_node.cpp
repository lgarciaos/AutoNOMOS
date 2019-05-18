// #define _GLIBCXX_USE_CXX11_ABI 0

// std
#include <chrono>

#include "a_star.h"
#include "autonomos.hpp"
#include "motion_planners/rrt.hpp"
#include "motion_planners/sst.hpp"
#include "motion_planners/dirt.hpp"
  
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
// #include "sim_params.h"

#define RRT "RRT"
#define SST "SST"
#define DIRT "DIRT"

// output colors
#define GREEN_BOLD "\033[32;1m"
#define GREEN "\033[32m"
#define OUT_RESET "\033[0m"

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
  bool publish_sln;
  bool publish_tree;

}

namespace params
{
  double integration_step;
  string stopping_type;
  double stopping_check;
  string stats_type;
  double stats_check;
  bool intermediate_visualization;
  unsigned min_time_steps;
  unsigned max_time_steps;
  int random_seed;
  double sst_delta_near;
  double sst_delta_drain;
  string planner_name;
  string system;
  double* start_state;
  double* goal_state;
  double goal_radius;
  bool global_planning;
  string ctrl_to_use;
  double perception_radius;
  double pos_x_bound;
  double neg_x_bound;
  double pos_y_bound;
  double neg_y_bound;
  double delta_t;
}

///////////////
// VARIABLES //
///////////////
const int rate_hz = 10;

std::vector<string> gz_green = {"Gazebo/White", "Gazebo/Yellow", "Gazebo/Grey"};
std::vector<string> gz_purple = {"Gazebo/Green", "Gazebo/BlueLaser"};//, "Gazebo/Red"};

autonomos_t* system_aux;
condition_check_t* checker;
condition_check_t* stats_check;


int subscriptions_established;
int dummy;
int path_counter;
int iterations = 0;

std::chrono::high_resolution_clock::time_point start_time;

double obstacles_radius;

std::vector<int> vec_obstacles_type;
std::vector<geometry_msgs::Pose> model_states;
std::vector<geometry_msgs::Pose> vec_obstacles_poses;
std::vector<ros::Publisher> pub_gazebo_lines_visualizer;

std::string algorithm;

std_msgs::Float64MultiArray vec_lines_closed;
std_msgs::Float64MultiArray vec_lines_path;
std_msgs::Float64MultiArray vec_lines_opened;

geometry_msgs::Point initial_pt, end_pt;
geometry_msgs::Pose2D params_start_state, params_goal_state;

ros::Publisher pub_target_pose;
ros::Publisher pub_start_pose;
ros::Publisher pub_car_trajectory;
ros::Publisher pub_sim_line;
ros::Subscriber sub_robot_pose;

a_star_t* a_star_ptr;

planner_t* planner;


///////////////
// FUNCTIONS //
///////////////
void publish_lines();
void a_star_solve();
void init_planner();
void get_obstacles_poses_callback(const geometry_msgs::PoseArray& msg);
void get_obstacles_types_callback(const std_msgs::Int64MultiArray& msg);
void rrt_sst_solver();
void publish_car_trajectory(std::vector<std::tuple<double*, double, double*> >& controls);
void publish_sln_trajectory();
void publish_sln_tree(tree_node_t* node);
void publish_sln_tree_1(tree_node_t* node, motion_planning::Line_Segment& ls_traj);
void get_robot_pose_callback(const geometry_msgs::Pose2D& pose);
void run_planner();
void replan_setup();


void run_planner()
{
  if (params::planner_name == GRID || params::planner_name == CTRL)
  {
    a_star_solve();
  }
  else
  {
    rrt_sst_solver();
  }
}

void replan_setup()
{
  checker -> set_condition_check(params::delta_t);
  checker -> reset();

  planner -> replanning_update_tree(params::delta_t, params::start_state);
  planner -> set_goal_state(params::goal_state, params::goal_radius);
  system_aux -> set_current_loc(params::start_state[0], params::start_state[1], params::start_state[2]);

  system_aux -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);

}

void get_robot_pose_callback(const geometry_msgs::Pose2D& pose)
{ 
  subscriptions_established |= 4; 
  ROS_WARN_STREAM(__PRETTY_FUNCTION__);
  if (system_aux != NULL)
  {
    ROS_DEBUG_STREAM("system_aux not NULL");
    system_aux -> set_current_loc(pose.x, pose.y, pose.theta);
    params::start_state[0] = pose.x;
    params::start_state[1] = pose.y;
    params::start_state[2] = pose.theta;
    sub_robot_pose.shutdown();
  }
  else
  {
    ROS_WARN_STREAM("system_aux NULL " << system_aux);
  }

}

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
        publish_lines();
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
	}
	else
	{
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
			if(stats_print)
			{
				std::vector<std::pair<double*, double> > controls;
				a_star_ptr -> get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}

				stats_print = false;

				stats_check->reset();
			}
			if (execution_done)
			{
				break;
			}
		}
	}
}

void init_planner()
{
  std::cout << path_counter << ": Initializing new planner: " << params::planner_name << std::endl;

  path_counter++;

  if (params::planner_name == GRID || params::planner_name == CTRL)
  {
    a_star_ptr -> set_type(params::planner_name);
    a_star_ptr -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);

    if(params::planner_name == GRID)
    {
      std::vector<geometry_msgs::Point> points;
      points = a_star_ptr -> generate_grid(.5, .5, -10, -10, 0, 10);
      a_star_ptr -> remove_obst_points(points);
    }
    else
    {
      a_star_ptr -> reset_nodes();
      a_star_ptr -> setup_planning();
      a_star_ptr -> set_start_state(params_start_state);
      a_star_ptr -> set_goal_state(params_goal_state, params::goal_radius);
    }
  }
  else 
  {
    if (params::planner_name == RRT)
    {
      planner = new rrt_t(system_aux);
    }
    else if (params::planner_name == SST)
    {
      planner = new sst_t(system_aux);
    } 
    else if(params::planner_name == DIRT)
    {
      planner = new dirt_t(system_aux);
    }
    checker = new condition_check_t(params::stopping_type, params::stopping_check);
    // stats_check=NULL;
    if(params::stats_check != 0)
    {
      stats_check = new condition_check_t(params::stats_type, params::stats_check);
    }
    planner -> set_start_state(params::start_state);
    planner -> set_goal_state(params::goal_state, params::goal_radius);
    system_aux -> set_obstacles(vec_obstacles_poses, vec_obstacles_type, obstacles_radius);
    planner -> setup_planning();
    checker -> reset();

  }

}

void rrt_sst_solver()
{
	ROS_WARN_STREAM("Starting the planner: " << params::planner_name << " for the system: " << params::system);
  ROS_WARN("From -> To:\t(%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)", 
    params::start_state[0], params::start_state[1], params::start_state[2],
    params::goal_state[0],  params::goal_state[1],  params::goal_state[2]);

	if(stats_check==NULL)
	{
		do
		{
			planner->step();
		}
		while(!checker -> check());
		std::vector<std::tuple<double*, double, double*> > controls;
		planner -> get_solution(controls);
		double solution_cost = 0;

		for(unsigned i = 0; i < controls.size(); i++)
		{
			solution_cost += std::get<1>(controls[i]);
		}
    if (sim_params::publish_car_trajectory)
    {
      publish_car_trajectory(controls);
    }
    std::cout << GREEN_BOLD << "Planner:\t" << GREEN << params::planner_name <<
      "\tTime:\t" << GREEN << checker -> time() << GREEN_BOLD <<  "\tIterations:\t" << 
      GREEN << checker -> iterations() << GREEN_BOLD << "\tNodes:\t" << GREEN << 
      planner -> number_of_nodes << GREEN_BOLD << "\tSln_Quality:\t" << GREEN << 
      solution_cost << GREEN_BOLD << "\tcontroller:\t" << GREEN << params::ctrl_to_use << 
      OUT_RESET << std::endl ;
	}
	else
	{
    ROS_WARN_STREAM("Stats_check OK");

		int count = 0;
		bool execution_done = false;
		bool stats_print = false;
		while(true)
		{
			do
			{
				planner->step();
				execution_done = checker -> check();
				stats_print = stats_check->check();
			}
			while(!execution_done && !stats_print);
			if(stats_print)
			{
				std::vector<std::tuple<double*,double, double*> > controls;
				planner -> get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost += std::get<1>(controls[i]);
				}
        // sst_ros_t *sst_aux = dynamic_cast<sst_ros_t*>(planner);
        publish_car_trajectory(controls);
				std::cout << "Time: " << checker -> time() << " Iterations: " <<
          checker -> iterations() << " Nodes: " << planner -> number_of_nodes <<
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
				std::cout << "Time: " << checker -> time() << " Iterations: " <<
          checker -> iterations() << " Nodes: " << planner -> number_of_nodes <<
          " Solution Quality: " << solution_cost << std::endl ;
				planner->visualize_tree(count);
				planner->visualize_nodes(count);
				break;
			}
		}
	}
	std::cout << "Done planning." << std::endl;
}

void publish_car_trajectory(std::vector<std::tuple<double*, double, double*> >& controls)
{
  std::cout << __PRETTY_FUNCTION__ << '\n';
  // motion_planning::car_trajectory res;

  // car_trajectory_msg.path_len = controls.size();
  // for(unsigned i = 0; i < controls.size(); i++)
  motion_planning::car_trajectory car_trajectory_msg;
  

  long unsigned int i = 0;
  double acum_duration = 0;
  ROS_WARN("acum_duration: %.3f\tdelta_t: %.3f", acum_duration, params::delta_t);

  while(acum_duration < params::delta_t && i < controls.size())
  {
    car_trajectory_msg.speed.push_back(std::get<0>(controls[i])[0]);
    car_trajectory_msg.steering.push_back(std::get<0>(controls[i])[1]);
    car_trajectory_msg.duration.push_back(std::get<1>(controls[i]));
    acum_duration += std::get<1>(controls[i]);
    params::start_state[0] = std::get<2>(controls[i])[0];
    params::start_state[1] = std::get<2>(controls[i])[1];
    params::start_state[2] = std::get<2>(controls[i])[2];
    // ROS_WARN("Traj next point: ( %.3f, %.3f, %.3f )", 
      // params::start_state[0], params::start_state[1], params::start_state[2]);
    ROS_WARN("Traj next speed: %.3f, dur: %.3f", std::get<0>(controls[i])[0], std::get<1>(controls[i]));
    // ROS_WARN("acum_duration: %.3f\tdelta_t: %.3f", acum_duration, params::delta_t);
    i++;
  }
  car_trajectory_msg.path_len = i;
  car_trajectory_msg.header.seq = iterations;
  car_trajectory_msg.header.stamp = ros::Time::now();;
  car_trajectory_msg.header.frame_id = "traj_" + std::to_string(iterations);

  pub_car_trajectory.publish(car_trajectory_msg);

  car_trajectory_msg.header.seq++;
}

void get_obstacles_poses_callback(const geometry_msgs::PoseArray& msg)
{
  // std::cout << "Line: " << __LINE__ << '\n';
  subscriptions_established |= 1;
  for (auto e : msg.poses )
  {
    vec_obstacles_poses.push_back(e);
  }
}

void get_obstacles_types_callback(const std_msgs::Int64MultiArray& msg)
{
  subscriptions_established |= 2; 
  ROS_WARN_STREAM(__PRETTY_FUNCTION__);
  for (auto e: msg.data )
  {
    vec_obstacles_type.push_back(e);
  }
}

void publish_sln_tree(tree_node_t* node)
{
  motion_planning::Line_Segment ls_traj;
  ls_traj.line_list = false;
  ls_traj.header.stamp = ros::Time::now();
  ls_traj.header.seq = iterations % gz_purple.size();
  ls_traj.color = gz_purple[ls_traj.header.seq];
  ls_traj.header.frame_id = "tree_seg_" + std::to_string(ls_traj.header.seq) ;

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

  ls_traj.header.stamp = ros::Time::now();
  ls_traj.header.seq = iterations % gz_green.size();
  ls_traj.header.frame_id = "traj_seg_" + std::to_string(ls_traj.header.seq);
  ls_traj.color = gz_green[ls_traj.header.seq];
  ls_traj.line_list = true;
  for (auto&  node : sln) 
  {
    ROS_DEBUG_STREAM_NAMED("TRAJ_PUBLISHER","( " << node -> point[0] << ", " << node -> point[1] << ", " << node -> point[2] << " )" );
    point.x = node -> point[0];
    point.y = node -> point[1];
    point.z = 0.1;
    ls_traj.points.push_back(point);
  }
  pub_sim_line.publish(ls_traj);

}

void publish_lines()
{
  if (params::planner_name == CTRL)
  {
    vec_lines_path   = a_star_ptr -> get_path_lines();
    vec_lines_opened = a_star_ptr -> get_opened_lines();
    vec_lines_closed = a_star_ptr -> get_closed_lines();
    pub_gazebo_lines_visualizer[0].publish(vec_lines_path);
    pub_gazebo_lines_visualizer[1].publish(vec_lines_opened);
    pub_gazebo_lines_visualizer[2].publish(vec_lines_closed);
  }
  else
  {
    std::vector<tree_node_t*>  sln;
    tree_node_t* root;

    root = planner -> get_root();
    planner -> get_last_solution_path(sln);

    if (sim_params::publish_tree)
    {
      publish_sln_tree(root);
    }
    if (sim_params::publish_sln)
    {
      publish_sln_trajectory(sln);
    }

    // if (dealloc) <== TODO:dealoc somewhere else
    // {
    //   ROS_WARN_STREAM("Deallocating planner...");
    //   delete planner;
    // }
  }
  pub_target_pose.publish(params_goal_state);
  pub_start_pose.publish(params_start_state);
}

void init_variables()
{
  ROS_WARN_STREAM("MP NODE: " << __FUNCTION__ <<  ": beginning..." );

  // init outgoing control message 
  // start_time = std::chrono::high_resolution_clock::now();
  

  // Keep track of how many subscribers have gotten a callback
  subscriptions_established = 0;

  params::start_state = new double[3];
  
  planner = NULL;
  checker = NULL;
  system_aux = NULL;
  stats_check = NULL;
  
  // if using A*, init the planner
  if(params::planner_name == CTRL || params::planner_name == GRID)
  {
    a_star_ptr = new a_star_t();
  }
  else if (params::planner_name == RRT || params::planner_name == SST || params::planner_name == DIRT)
  {
    system_aux = new autonomos_t(params::ctrl_to_use, params::global_planning);
    
    if (params::global_planning)
    {
      system_aux -> set_bounds(params::pos_x_bound, params::neg_x_bound, params::pos_y_bound, params::neg_y_bound);
    }
    else
    {
      system_aux -> set_bounds( params::perception_radius);
    }
  }
  else
  {
    ROS_FATAL_STREAM("Not a valid planner: " << params::planner_name);
    ros::shutdown();
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
  params::goal_state = new double[3];

  ROS_WARN_STREAM("MP NODE: " << __FUNCTION__ <<  ": Reading parameters");
  // motion planning parameters
  nh_priv.param<double>       ("integration_step",  params::integration_step, .001);
  nh_priv.param<string>       ("stopping_type",     params::stopping_type, "time");
  nh_priv.param<double>       ("stopping_check",    params::stopping_check, 15);
  nh_priv.param<string>       ("stats_type",        params::stats_type, "time");
  nh_priv.param<double>       ("stats_check",       params::stats_check, 0);
  nh_priv.param<int>          ("min_time_steps",    min_time_steps_aux, 20);
  nh_priv.param<int>          ("max_time_steps",    max_time_steps_aux, 200);
  nh_priv.param<int>          ("random_seed",       params::random_seed, 0);
  nh_priv.param<double>       ("sst_delta_near",    params::sst_delta_near, 0.4);
  nh_priv.param<double>       ("sst_delta_drain",   params::sst_delta_drain, 0.2);
  nh_priv.param<string>       ("planner",           params::planner_name, CTRL);
  nh_priv.param<string>       ("system",            params::system, "car");
  nh_priv.param<double>       ("goal_radius",       params::goal_radius, 0.5);
  nh_priv.param<double>       ("goal_state/x",      params::goal_state[0], -1);
  nh_priv.param<double>       ("goal_state/y",      params::goal_state[1], 9);
  nh_priv.param<double>       ("goal_state/theta",  params::goal_state[2], 0);
  nh_priv.param<bool>         ("global_planning",   params::global_planning, true);
  nh_priv.param<string>       ("ctrl_to_use",       params::ctrl_to_use, RANDOM_CTRL);
  nh_priv.param<double>       ("replanning/delta_t",         params::delta_t, 1);
  nh_priv.param<double>       ("bounding/pos_x_bound",       params::pos_x_bound, -0.0);
  nh_priv.param<double>       ("bounding/neg_x_bound",       params::neg_x_bound, -10.0);
  nh_priv.param<double>       ("bounding/pos_y_bound",       params::pos_y_bound, +10.0);
  nh_priv.param<double>       ("bounding/neg_y_bound",       params::neg_y_bound, -10.0);
  nh_priv.param<double>       ("bounding/perception_radius", params::perception_radius, 6);
  nh_priv.param<bool>         ("intermediate_visualization", params::intermediate_visualization, false);

  // global parameters
  nh.param<bool>   ("simulation/simulation", sim_params::simulation,  true);
  nh.param<bool>   ("simulation/publish_car_trajectory", sim_params::publish_car_trajectory,  true);
  nh.param<int>    ("simulation/gz_total_lines", sim_params::gz_total_lines,  0);
  nh.param<bool>   ("simulation/publish_sln", sim_params::publish_sln,  false);
  nh.param<bool>   ("simulation/publish_tree", sim_params::publish_tree,  false);
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
    ROS_WARN_STREAM("Initializing sim sub/pub:");
    string pose_topic_name = "/" + sim_params::model_name + "/pose";
    string sim_topic_name = "/" + sim_params::model_name + "/gz_visual/line_segment";
    pub_sim_line = nh.advertise<motion_planning::Line_Segment>(sim_topic_name, 100000, true);
    sub_robot_pose = nh.subscribe(pose_topic_name, 1, &get_robot_pose_callback);
    ROS_WARN_STREAM("\tpublishing to: " << pub_sim_line.getTopic());
    ROS_WARN_STREAM("\tsubscribed to: " << sub_robot_pose.getTopic());
  }
  init_variables();

  // ROS_WARN_STREAM("sim_tools_testing_node initiated");

  loop_rate.sleep();

  ROS_WARN_STREAM("MP NODE: " <<  __FUNCTION__ <<  ": Going into while(ros::ok())");
  ros::service::call("/gazebo/reset_simulation", empty);
  while(ros::ok())
  {
    ros::spinOnce();
    if (subscriptions_established == 7)
    {
      ROS_WARN_STREAM("Iteration: " << iterations);
      if(planner == NULL) // If this is the first iteration, init the planner
      {
        init_planner();
      }
      else // if is the 2nd or more iteration, do the replaning steps
      {
        replan_setup();
      }
      // run the planner
      run_planner();

      publish_lines();
      
      iterations++;
    }
    else
    {
      ROS_WARN_STREAM("not enough subscriptions established: " << subscriptions_established);
    }
    std::cout << "Input something and press enter to continue..." << '\n';
    // std::cin >> dummy;
  }
  std::cout << "Finishing motion_planning_node." << std::endl;
  return 0;
}

























///////////////////////////////////////////////////////////////////////////////
//                                  TODO
///////////////////////////////////////////////////////////////////////////////
// TODO:  Change theparams::ctrl_to_use from int to string and use an array at autonomos
//
