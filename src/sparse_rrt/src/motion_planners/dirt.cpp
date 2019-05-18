/**
 * @file dirt.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2018, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 * Modified by: Edgar Granados, 2019, ITAM.
 * 
 */

#include "motion_planners/dirt.hpp"
#include "utilities/random.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"

#include <iostream>
#include <deque>


void dirt_t::setup_planning()
{
	best_goal = NULL;
	largest_dir_radius=0;
	//init internal variables
	sample_state = system->alloc_state_point();
	sample_control = system->alloc_control_point();
	metric_query = new dirt_node_t();
	metric_query->point = system->alloc_state_point();

    close_nodes = (proximity_node_t**)malloc(MAX_KK * sizeof (proximity_node_t*));
    distances = (double*)malloc(MAX_KK * sizeof (double));

	//initialize the metrics
	metric = new graph_nearest_neighbors_t();
	metric->set_system(system);
	//create the root of the tree
	root = new dirt_node_t();
	root->point = system->alloc_state_point();
	system->copy_state_point(root->point,start_state);
	add_point_to_metric(root);
	((dirt_node_t*)root)->h = system->heuristic(root->point,goal_state);
	child_selection=true;
	nearest = ((dirt_node_t*)root);
	number_of_nodes++;

}

void dirt_t::get_solution(std::vector<std::pair<double*,double> >& controls)
{
	last_solution_path.clear();
	if(best_goal==NULL)
		return;
	nearest = best_goal;
	
	//now nearest should be the closest node to the goal state
	std::deque<tree_node_t*> path;
	while(nearest->parent!=NULL)
	{
		path.push_front(nearest);
		nearest = (dirt_node_t*)nearest->parent;
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

void dirt_t::get_solution(std::vector<std::tuple<double*,double, double*> >& controls)
{
	ROS_FATAL("NOT IMPLEMENTED YET");
}

void dirt_t::replanning_update_tree(double delta_t, double* &new_state_point)
{
	ROS_FATAL("NOT IMPLEMENTED YET");	
}

void dirt_t::step()
{
	nearest_vertex();
	propagate();
}

void dirt_t::add_point_to_metric(tree_node_t* state)
{
	proximity_node_t* new_node = new proximity_node_t(state);
	state->prox_node = new_node;
	metric->add_node(new_node);
}
void dirt_t::nearest_vertex()
{
	system->random_state(sample_state);
	if(child_selection)
	{
		child_selection=false;
		return;
	}
	//performs the best near query
	system->copy_state_point(metric_query->point,sample_state);
	unsigned val = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,largest_dir_radius);

	if(system->distance(sample_state,close_nodes[0]->get_state()->point)>largest_dir_radius)
	{
		system->copy_state_point(metric_query->point,close_nodes[0]->get_state()->point);
		val = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,largest_dir_radius);
		system->copy_state_point(sample_state,metric_query->point);
	}
	//determine all nodes that the "sample" falls in
	std::vector<dirt_node_t*> candidates;
	for(int i=0;i<val;i++)
	{
		auto node = (dirt_node_t*)close_nodes[i]->get_state();
		if(system->distance(sample_state,close_nodes[i]->get_state()->point)<=node->dir_radius)
		{
			candidates.push_back(node);
		}
	}

	nearest = (dirt_node_t*)(candidates.size()>0?candidates[uniform_int_random(0,candidates.size()-1)]:close_nodes[0]->get_state());
}
bool dirt_t::propagate()
{
	if(nearest->man_index.size()==0)
	{
		nearest->maneuvers = system->maneuver_generation(nearest->point,nearest->num_maneuvers);
		nearest->num_maneuvers = 1;

		std::vector<double> pred_values;
		int index = 0;
		for(auto temp_eg : nearest->maneuvers)
		{
			pred_values.push_back(system->heuristic(nearest->point,std::get<2>(temp_eg)));
			nearest->man_index.push_back(index++);
		}
		//sort these indices based on the prediction values (in opposite order for the ability to pop off the back of the vector)
        std::sort(nearest->man_index.begin(),nearest->man_index.end(),
                [this,pred_values](const int& a, const int& b)
                {
                    return pred_values[a]>pred_values[b];
                });
	}


	while(nearest->man_index.size()!=0)
	{
		auto eg = nearest->maneuvers[nearest->man_index.back()];
		nearest->man_index.pop_back();

		if(best_goal==NULL || nearest->cost + duration <= best_goal->cost)//bnb
		{
			system->copy_state_point(sample_state,std::get<2>(eg));
			system->copy_control_point(sample_control,std::get<0>(eg));
			duration = std::get<1>(eg);
			add_to_tree();
			delete std::get<2>(eg);
			delete std::get<0>(eg);
			return true;
		}

		delete std::get<2>(eg);
		delete std::get<0>(eg);
	}
	return false;
}
void dirt_t::add_to_tree()
{
	//create a new tree node
	dirt_node_t* new_node = new dirt_node_t();
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
	new_node->h = system->heuristic(sample_state,goal_state);
	//set parent's child
	nearest->children.insert(nearest->children.begin(),new_node);
	number_of_nodes++;

	new_node->dir_radius = system->distance(new_node->point,nearest->point);

	//update dir radii based on closest things nearby
	system->copy_state_point(metric_query->point,new_node->point);
	unsigned num_neighbors = metric->find_delta_close(metric_query,close_nodes,distances,std::max(largest_dir_radius,new_node->dir_radius));

	//update new node dominance radius
	for(unsigned i=0;i<num_neighbors;i++)
	{
		auto node = (dirt_node_t*)close_nodes[i]->get_state();
		if( new_node->cost+new_node->h > node->cost + node->h)
		{
			new_node->dir_radius = std::min(new_node->dir_radius,system->distance(new_node->point,node->point));
		}
	}

	//update existing nodes dominance radius
	for(unsigned i=0;i<num_neighbors;i++)
	{
		auto node = (dirt_node_t*)close_nodes[i]->get_state();
	    double sibling_distance = system->distance(node->point,new_node->point);

		if( new_node->cost + new_node->h < node->cost + node->h)
		{
			node->dir_radius = std::min(sibling_distance,node->dir_radius);
		}
	}
	largest_dir_radius = std::max(largest_dir_radius,new_node->dir_radius);
	((dirt_node_t*)root)->dir_radius = largest_dir_radius;

    if(best_goal==NULL && system->distance(new_node->point,goal_state)<goal_radius)
    {
    	best_goal = new_node;
    	branch_and_bound((dirt_node_t*)root);
    }
    else if(best_goal!=NULL && best_goal->cost > new_node->cost && system->distance(new_node->point,goal_state)<goal_radius)
    {
    	best_goal = new_node;
    	branch_and_bound((dirt_node_t*)root);
    }
	add_point_to_metric(new_node);

	if(nearest->h > new_node->h)
	{
		child_selection = true;
		nearest = new_node;
	}

}

void dirt_t::branch_and_bound(dirt_node_t* node)
{
    std::list<tree_node_t*> children = node->children;
    for (std::list<tree_node_t*>::iterator iter = children.begin(); iter != children.end(); ++iter)
    {
    	branch_and_bound((dirt_node_t*)(*iter));
    }
    if(is_leaf(node) && !is_best_goal(node) && node->cost + node->h > best_goal->cost)
    {
    	if(!node->inactive)
    	{
	    	remove_point_from_metric(node);
	    }
    	remove_leaf(node);
    }
}

void dirt_t::remove_point_from_metric(tree_node_t* node)
{
	proximity_node_t* old_node = node->prox_node;
	metric->remove_node(old_node);
	delete old_node;
}

bool dirt_t::is_leaf(tree_node_t* node)
{
	return node->children.size()==0;
}

void dirt_t::remove_leaf(tree_node_t* node)
{
	if(node->parent != NULL)
	{
		tree_edge_t* edge = node->parent_edge;
		node->parent->children.remove(node);
		number_of_nodes--;
		delete edge->control;
		delete node->point;
		delete node;
	}
}

bool dirt_t::is_best_goal(tree_node_t* v)
{
	if(best_goal==NULL)
		return false;
    tree_node_t* new_v = best_goal;

    while(new_v->parent!=NULL)
    {
        if(new_v == v)
            return true;

        new_v = new_v->parent;
    }
    return false;

}

