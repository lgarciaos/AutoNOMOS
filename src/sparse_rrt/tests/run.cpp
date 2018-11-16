/**
 * @file run.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "utilities/parameter_reader.hpp"
#include "utilities/condition_check.hpp"
#include "utilities/random.hpp"
#include "utilities/timer.hpp"

#include "systems/pendulum.hpp"
#include "systems/point.hpp"
#include "systems/car.hpp"
#include "systems/rally_car.hpp"
#include "systems/cart_pole.hpp"
#include "systems/two_link_acrobot.hpp"
#include "motion_planners/sst.hpp"
#include "motion_planners/dirt.hpp"
#include "motion_planners/rrt.hpp"

#include <iostream>

int main(int ac, char* av[])
{
	read_parameters(ac,av);
	//****************After reading in from input, we need to instantiate classes
	init_random(params::random_seed);
	system_t* system;
	if(params::system=="point")
	{
		system = new point_t();
	}
	else if(params::system=="pendulum")
	{
		system = new pendulum_t();
	}
	else if(params::system=="car")
	{
		system = new car_t();
	}
	else if(params::system=="rally_car")
	{
		system = new rally_car_t();
	}
	else if(params::system=="cart_pole")
	{
		system = new cart_pole_t();
	}
	else if(params::system=="two_link_acrobot")
	{
		system = new two_link_acrobot_t();
	}

	planner_t* planner;
	if(params::planner=="rrt")
	{
		planner = new rrt_t(system);
	}
	else if(params::planner=="sst")
	{
		planner = new sst_t(system);
	}
	else if(params::planner=="dirt")
	{
		planner = new dirt_t(system);
	}
	planner->set_start_state(params::start_state);
	planner->set_goal_state(params::goal_state,params::goal_radius);
	planner->setup_planning();

	condition_check_t checker(params::stopping_type,params::stopping_check);
	condition_check_t* stats_check=NULL;
	if(params::stats_check!=0)
	{
		stats_check = new condition_check_t(params::stats_type,params::stats_check);
	}

	checker.reset();
	std::cout<<"Starting the planner: "<<params::planner<<" for the system: "<<params::system<<std::endl;
	if(stats_check==NULL)
	{
		do
		{
			planner->step();
		}
		while(!checker.check());
		std::vector<std::pair<double*,double> > controls;
		planner->get_solution(controls);
		double solution_cost = 0;
		for(unsigned i=0;i<controls.size();i++)
		{
			solution_cost+=controls[i].second;
		}
		std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
		planner->visualize_tree(0);
		planner->visualize_nodes(0);
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
					solution_cost+=controls[i].second;
				}
				std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
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
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
				planner->visualize_tree(count);
				planner->visualize_nodes(count);
				break;
			}
		}
	}
	std::cout<<"Done planning."<<std::endl;

}