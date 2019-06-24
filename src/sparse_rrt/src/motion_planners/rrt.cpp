/**
 * @file rrt.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 * Modified by: Edgar Granados, 2019, ITAM.
 * 
 */

#include "motion_planners/rrt.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"

#include <iostream>
#include <deque>
#include <queue>

void rrt_t::setup_planning()
{
	//init internal variables
	sample_state = system->alloc_state_point();
	sample_control = system->alloc_control_point();
	metric_query = new tree_node_t();
	metric_query->point = system->alloc_state_point();

    close_nodes = (proximity_node_t**)malloc(MAX_KK * sizeof (proximity_node_t*));
    distances = (double*)malloc(MAX_KK * sizeof (double));

	//initialize the metric
	metric = new graph_nearest_neighbors_t();
	metric->set_system(system);
	//create the root of the tree
	root = new tree_node_t();
	number_of_nodes++;
	root->point = system->alloc_state_point();
	system->copy_state_point(root->point,start_state);
	//add root to nearest neighbor structure
	add_point_to_metric(root);

}

void rrt_t::replanning_update_tree(double delta_t, double* &new_state_point)
{
	// std::vector<tree_node_t*> aux_solution_path;
	system->copy_state_point(sample_state,goal_state);
	system->copy_state_point(metric_query->point,sample_state);
	unsigned val = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,goal_radius);

    double length = 999999999;
    tree_node_t* aux_nearest;
    for(unsigned i=0;i<val;i++)
    {
        tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
        double temp = v->cost ;
        if( temp < length)
        {
            length = temp;
            aux_nearest = v;
        }
    }
	//now nearest should be the closest node to the goal state
	if(system->distance(goal_state,aux_nearest->point) < goal_radius)
	{
		std::deque<tree_node_t*> path;
		while(aux_nearest->parent!=NULL)
		{
			path.push_front(aux_nearest);
			aux_nearest = aux_nearest->parent;
		}
		long unsigned int i = 0;
		double acum_duration = 0;

		std::queue<tree_node_t*> orphans_queue;
		while(acum_duration < delta_t && i < path.size())
		{
			acum_duration += path[i]->parent_edge->duration;
			ROS_DEBUG("i: %d\t#children: %d\tdur: %.3f", i, root -> children.size(), acum_duration);

			for(auto & child : root -> children)
			{
				if (child == path[i])
				{
					root = child;
					root -> parent = NULL;
				}
				else
				{
					// ROS_WARN("deleting child: (%.3f, %.3f, %.3f)",
						// child -> point[0], child -> point[1], child -> point[2]);
					// orphans_queue.push(child);
				}
			}
			i++;
		}
		ROS_WARN("New root point: (%.3f, %.3f, %.3f)", 
			root -> point[0], root -> point[1], root -> point[2]);
		system->copy_state_point(start_state, root -> point);
		system->copy_state_point(new_state_point, root -> point);

	}
}

void rrt_t::get_solution(std::vector<std::tuple<double*,double, double*> >& controls)
{
	last_solution_path.clear();
	system->copy_state_point(sample_state,goal_state);
	system->copy_state_point(metric_query->point,sample_state);
	unsigned val = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,goal_radius);

    double length = 999999999;
    for(unsigned i=0;i<val;i++)
    {
        tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
        double temp = v->cost ;
        if( temp < length)
        {
            length = temp;
            nearest = v;
        }
    }
	//now nearest should be the closest node to the goal state
	if(system->distance(goal_state,nearest->point) < goal_radius)
	{
		std::deque<tree_node_t*> path;
		while(nearest->parent!=NULL)
		{
			path.push_front(nearest);
			nearest = nearest->parent;
		}
		last_solution_path.push_back(root);
		ROS_WARN("The root of sln is: (%.3f, %.3f, %.3f)", root -> point[0], root -> point[1], root -> point[2]);
		for(unsigned i=0;i<path.size();i++)
		{
			last_solution_path.push_back(path[i]);
			controls.push_back(std::tuple<double*,double, double*>(NULL,0, NULL));
			std::get<0>(controls.back()) = system->alloc_control_point();
			system->copy_control_point(std::get<0>(controls.back()) ,path[i]->parent_edge->control);
			std::get<1>(controls.back()) = path[i]->parent_edge->duration;
			std::get<2>(controls.back()) = path[i] -> point;
		}
	}
}

void rrt_t::get_solution(std::vector<std::pair<double*,double> >& controls)
{
	last_solution_path.clear();
	system->copy_state_point(sample_state,goal_state);
	system->copy_state_point(metric_query->point,sample_state);
	unsigned val = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,goal_radius);

    double length = 999999999;
    for(unsigned i=0;i<val;i++)
    {
        tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
        double temp = v->cost ;
        if( temp < length)
        {
            length = temp;
            nearest = v;
        }
    }
	//now nearest should be the closest node to the goal state
	if(system->distance(goal_state,nearest->point) < goal_radius)
	{
		std::deque<tree_node_t*> path;
		while(nearest->parent!=NULL)
		{
			path.push_front(nearest);
			nearest = nearest->parent;
		}
		last_solution_path.push_back(root);
		for(unsigned i=0;i<path.size();i++)
		{
			last_solution_path.push_back(path[i]);
			controls.push_back(std::pair<double*,double>(NULL,0));
			controls.back().first = system->alloc_control_point();
			system->copy_control_point(controls.back().first,path[i]->parent_edge->control);
			controls.back().second = path[i]->parent_edge->duration;
		}
	}
}
void rrt_t::step()
{
	random_sample();
	nearest_vertex();
	if(propagate())
	{
		add_to_tree();
	}
}

void rrt_t::add_point_to_metric(tree_node_t* state)
{
	proximity_node_t* new_node = new proximity_node_t(state);
	state->prox_node = new_node;
	metric->add_node(new_node);
}

void rrt_t::random_sample()
{
	system->random_state(sample_state);
	system->random_control(sample_control);
}

void rrt_t::nearest_vertex()
{
	system->copy_state_point(metric_query->point,sample_state);
	double distance;
	nearest = (tree_node_t*)metric->find_closest(metric_query,&distance)->get_state();
}
bool rrt_t::propagate()
{
	return system->propagate(nearest->point,sample_control,params::min_time_steps,params::max_time_steps,sample_state,duration);
}
void rrt_t::add_to_tree()
{
	//create a new tree node
	tree_node_t* new_node = new tree_node_t();
	new_node->point = system->alloc_state_point();
	system->copy_state_point(new_node->point,sample_state);
	//create the link to the parent node
	new_node->parent_edge = new tree_edge_t();
	new_node->parent_edge->control = system->alloc_control_point();
	system->copy_control_point(new_node->parent_edge->control,sample_control);
	new_node->parent_edge->duration = duration;
	//set this node's parent
	new_node->parent = nearest;
	new_node->cost = nearest->cost + duration;
	//set parent's child
	nearest->children.insert(nearest->children.begin(),new_node);
	add_point_to_metric(new_node);
	number_of_nodes++;

}

