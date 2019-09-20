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
			// ROS_DEBUG("i: %d\t#children: %d\tdur: %.3f", i, root -> children.size(), acum_duration);

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
		// ROS_WARN("New root point: (%.3f, %.3f, %.3f)", 
			// root -> point[0], root -> point[1], root -> point[2]);
		system->copy_state_point(start_state, root -> point);
		system->copy_state_point(new_state_point, root -> point);

	}
}

bool rrt_t::get_solution_1(tree_node_t* node, int node_num, double& total_cost)
{
	// ROS_WARN("Num: %d\tCost: %.3f\trisk: %.3f\ttotal_cost: %.3f", node_num, node -> cost, node -> risk, total_cost);
	double r_k = node -> risk;
	double risk_mult = 1;

	if ( r_k >= SMALL_EPSILON )
	{
		risk_mult = pow(1 + r_k, log(node_num));
	}


	if (  r_k > risk_aversion )
	{
		ROS_WARN("\tAT: ( %.3f, %.3f, %.3f )", node -> point[0], node -> point[1], node -> point[2]);
		ROS_WARN("\tr_k: %.3f\tnode_num: %d\trisk_mult: %.3f", r_k, node_num, risk_mult);
		ROS_WARN("\tEXITING ==> risk_mult: %.3f\t inverse: %.3f", risk_mult, inv_risk_aversion);
		return false;
	}
	
	total_cost += node -> cost * risk_mult;
	
	if (node -> parent == NULL)
	{
		return true;
	}
	
	return get_solution_1(node -> parent, ++node_num, total_cost);
	// bool res;
	// if ( r_k >= risk_aversion)
	// {
	// 	return false;
	// }
	// else if (node -> parent != NULL)
	// {
	// 	res = get_solution_1(node -> parent, ++node_num, total_cost);
	// }
	// // else
	// // {
	// 	// total_cost = node -> cost * risk_mult;
	// 	// return true;
	// // }
	// total_cost += node -> cost * risk_mult;
	// return res;


}

void rrt_t::get_solution(std::vector<std::tuple<double*,double, double*, double> >& controls, bool asses_risk)
{

	last_solution_path.clear();
	system->copy_state_point(sample_state,goal_state);
	system->copy_state_point(metric_query->point,sample_state);
	unsigned val = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,goal_radius);

    double length = 999999999;

	if (asses_risk)
	{
		// ROS_WARN("Assesing risk...");
		
		double temp_cost;
		bool valid_sln;
		for (unsigned i = 0; i < val; ++i)
		{
    	    tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
			temp_cost = 0;

    	    valid_sln = get_solution_1(v, 1, temp_cost);
			if ( valid_sln && temp_cost < length )
			{
				// ROS_WARN("New length: %.3f", temp_cost);
				length = temp_cost;
				nearest = v;
			}
		}


	}
	else
	{
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
				controls.push_back(std::tuple<double*,double, double*, double>(NULL,0, NULL, 0));
				std::get<0>(controls.back()) = system->alloc_control_point();
				system->copy_control_point(std::get<0>(controls.back()) ,path[i]->parent_edge->control);
				std::get<1>(controls.back()) = path[i]->parent_edge->duration;
				std::get<2>(controls.back()) = path[i] -> point;
				std::get<3>(controls.back()) = path[i] -> risk;
			}
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

// void rrt_t::forward_risk_propagation()
// {
// 	propagate_risk_forward(root, 1);
// }

void rrt_t::update_tree_risks()
{

	int i_dyn_obs = 0;
	while (system -> get_next_dynamic_state( sample_state , i_dyn_obs))
	{
		system->copy_state_point(metric_query->point, sample_state);
		int val = metric -> find_delta_close_and_closest( metric_query, close_nodes, distances, goal_radius, true );
	
		for (int i = 0; i < val; ++i)
		{
			tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
	        v -> risk = 1;
	        propagate_risk_backwards(v, 2);
		}

		i_dyn_obs++;
	}

}

// void rrt_t::propagate_risk_backwards(tree_node_t* node, int parent_num)
// {
// 	// if (node -> parent == NULL)
// 	// {
// 	// 	node -> risk = 0;
// 	// }
// 	// else if (node -> risk > SMALL_EPSILON )
// 	// {
// 	if ( node -> parent != NULL && node -> parent -> risk < risk_aversion )
// 	{
// 		node -> parent -> risk = propagating_function(node -> parent -> risk, node -> risk, (double)parent_num);
// 		// ROS_WARN("Duration: %.3f", node -> parent_edge -> duration);
// 		// ROS_WARN("node risk: %s\t parent risk: %.2f\tparent num: %d",
// 		// 	boost::lexical_cast<std::string>(node -> risk).c_str(), node -> parent -> risk, parent_num);
// 		propagate_risk_backwards( node -> parent, ++parent_num);
// 	}
// }
// bool rrt_t::propagate_risk_forward(tree_node_t* node, int node_num)
// {

// 	bool all_zero = node -> risk <= SMALL_EPSILON;

// 	for (auto child : node -> children)
// 	{
// 		if ( child -> risk > SMALL_EPSILON)
// 		{
// 			all_zero = false;
// 			propagate_risk_forward(child, ++node_num);
// 		}
// 		else if (child -> risk > 0.0)
// 		{
// 			all_zero &= propagate_risk_forward(child, ++node_num);
// 		}

// 	}

// 	// ROS_WARN("Risk: %.3f\tAll zero: %s\ti: %d", node -> risk, all_zero ? "TRUE" : "FALSE", node_num );
// 	if ( all_zero)
// 	{
// 		node -> risk = 0;
// 	}
// 	else
// 	{
// 		// ROS_WARN("Old risk: %.3f", node -> risk);
// 		node -> risk = log(1 + node -> risk) / node_num;
// 		// ROS_WARN("\tNew risk: %.3f", node -> risk);
// 	}
// 	return all_zero;

// }

