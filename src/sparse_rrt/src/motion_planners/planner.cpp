/**
 * @file planner.cpp
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

#include "motion_planners/planner.hpp"

void planner_t::set_start_state(double* in_start)
{
	if(start_state==NULL)
		start_state = system->alloc_state_point();
	system->copy_state_point(start_state,in_start);
}

void planner_t::set_goal_state(double* in_goal,double in_radius)
{
	if(goal_state==NULL)
		goal_state = system->alloc_state_point();
	system->copy_state_point(goal_state,in_goal);
	goal_radius = in_radius;
}

void planner_t::visualize_tree(int image_counter)
{
	std::stringstream s;
    s<<"tree_"<<image_counter<<".svg";
    std::string dir(s.str());
    svg::Dimensions dimensions(params::image_width, params::image_height);
    svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));

    visualize_edge(root,doc,dimensions);

	svg::Circle circle(system->visualize_point(start_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_path(doc,dimensions);
    system->visualize_obstacles(doc,dimensions);

    doc.save();
}

void sort(std::vector<tree_node_t*>& nodes)
{
	for(unsigned i=0;i<nodes.size();i++)
	{
		tree_node_t* x = nodes[i];
		unsigned j = i;
		while(j>0 && nodes[j-1]->cost > x->cost)
		{
			nodes[j] = nodes[j-1];
			j--;
		}
		nodes[j] = x;
	}
}

void planner_t::visualize_nodes(int image_counter)
{
	std::stringstream s;
    s<<"nodes_"<<image_counter<<".svg";
    std::string dir(s.str());
    svg::Dimensions dimensions(params::image_width, params::image_height);
    svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));
    sorted_nodes.clear();
    get_max_cost();
    sort(sorted_nodes);

    for(unsigned i=sorted_nodes.size()-1;i!=0;i--)
    {
	    visualize_node(sorted_nodes[i],doc,dimensions);
	}

	svg::Circle circle(system->visualize_point(start_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_nodes(doc,dimensions);
    system->visualize_obstacles(doc,dimensions);

    doc.save();
}
void planner_t::visualize_edge(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim)
{

	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		svg::Polyline traj_line(svg::Stroke(params::tree_line_width, svg::Color::Blue));

		traj_line<<system->visualize_point(node->point,dim);
		traj_line<<system->visualize_point((*i)->point,dim);
		doc<<traj_line;

		visualize_edge(*i,doc,dim);

	}

}

void planner_t::visualize_node(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim)
{

	svg::Circle circle(system->visualize_point(node->point,dim),params::node_diameter,svg::Fill( svg::Color((node->cost/max_cost)*255,(node->cost/max_cost)*255,(node->cost/max_cost)*255) ) );
	doc<<circle;
	// for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	// {
	// 	visualize_node(*i,doc,dim);
	// }

}

void planner_t::visualize_solution_path( svg::Document& doc, svg::Dimensions& dim)
{
	if(last_solution_path.size()!=0)
	{
		svg::Polyline traj_line(svg::Stroke(params::solution_line_width, svg::Color::Black));
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			traj_line<<system->visualize_point(last_solution_path[i]->point,dim);
		}
		doc<<traj_line;
	}
}
void planner_t::visualize_solution_nodes( svg::Document& doc, svg::Dimensions& dim)
{

	if(last_solution_path.size()!=0)
	{
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			svg::Circle circle(system->visualize_point(last_solution_path[i]->point,dim),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
			doc<<circle;
		}
	}
}

void planner_t::get_max_cost()
{
	max_cost = 0;
	get_max_cost(root);
}

void planner_t::get_max_cost(tree_node_t* node)
{
	sorted_nodes.push_back(node);
	if(node->cost > max_cost)
		max_cost = node->cost;
	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		get_max_cost(*i);
	}
}

void planner_t::get_last_solution_path(std::vector<tree_node_t*> & last_sln)
{
  last_sln = last_solution_path;
}

tree_node_t* planner_t::get_root()
{
  return root;
}

double planner_t::propagating_function(double parent_risk, double current_risk, double gamma)
{
	ROS_ASSERT_MSG(parent_risk > 1.0, "PARENT RISK GREATER THAN 1");

	// double res;
	double current, parent;
	current = 1 - exp(-gamma * current_risk);
	// if (parent_risk >= 1.0)
	// {
	// 	parent = 1 - exp(-gamma * pare);// should equal 0
	// 	// ROS_WARN("parent_risk: %f\tcurrent_risk: %f\tcurrent: %f", parent_risk, current_risk, current);
	// }
	// else 
	// {
		parent = 1 - exp(-gamma * parent_risk);
	// }
	// current = current > 1 ? 1 : current;
	// parent = parent > 1 ? 1 : parent;
	return ( current + parent ) / gamma ;
}

void planner_t::set_risk_aversion(double risk_aversion)
{
	// assert( ( "Risk aversion should be greater than 0!", risk_aversion > 0 ) );
	this -> risk_aversion = risk_aversion;
	if ( risk_aversion > 0 )
	{
		inv_risk_aversion = 1 / risk_aversion;
	}
	else 
	{
		inv_risk_aversion = 9999999999;
	}
}


bool planner_t::propagate_risk_forward(tree_node_t* node, int node_num)
{

	bool all_zero = node -> risk <= SMALL_EPSILON;

	for (auto child : node -> children)
	{
		if ( child -> risk > SMALL_EPSILON)
		{
			all_zero = false;
			propagate_risk_forward(child, ++node_num);
		}
		else if (child -> risk > 0.0)
		{
			all_zero &= propagate_risk_forward(child, ++node_num);
		}

	}

	// ROS_WARN("Risk: %.3f\tAll zero: %s\ti: %d", node -> risk, all_zero ? "TRUE" : "FALSE", node_num );
	if ( all_zero)
	{
		node -> risk = 0;
	}
	else
	{
		// ROS_WARN("Old risk: %.3f", node -> risk);
		node -> risk = log(1 + node -> risk) / node_num;
		// ROS_WARN("\tNew risk: %.3f", node -> risk);
	}
	return all_zero;

}

void planner_t::propagate_risk_backwards(tree_node_t* node, int parent_num)
{
	// if (node -> parent == NULL)
	// {
	// 	node -> risk = 0;
	// }
	// else if (node -> risk > SMALL_EPSILON )
	// {
	if ( node -> parent != NULL && node -> parent -> risk < risk_aversion )
	{
		node -> parent -> risk = propagating_function(node -> parent -> risk, node -> risk, (double)parent_num);
		// ROS_WARN("Duration: %.3f", node -> parent_edge -> duration);
		// ROS_WARN("node risk: %s\t parent risk: %.2f\tparent num: %d",
		// 	boost::lexical_cast<std::string>(node -> risk).c_str(), node -> parent -> risk, parent_num);
		propagate_risk_backwards( node -> parent, ++parent_num);
	}
}

void planner_t::forward_risk_propagation()
{
	propagate_risk_forward(root, 1);
}

bool planner_t::get_solution_1(tree_node_t* node, int node_num, double& total_cost)
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
		ROS_DEBUG("\tAT: ( %.3f, %.3f, %.3f )", node -> point[0], node -> point[1], node -> point[2]);
		ROS_DEBUG("\tr_k: %.3f\tnode_num: %d\trisk_mult: %.3f", r_k, node_num, risk_mult);
		ROS_DEBUG("\tEXITING ==> risk_mult: %.3f\t inverse: %.3f", risk_mult, inv_risk_aversion);
		return false;
	}
	
	total_cost += node -> cost * risk_mult;
	
	if (node -> parent == NULL)
	{
		return true;
	}
	
	return get_solution_1(node -> parent, ++node_num, total_cost);

}