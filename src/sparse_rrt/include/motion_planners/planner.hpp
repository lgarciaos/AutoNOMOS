/**
 * @file planner.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 * Modified by: Edgar Granados, 2019, ITAM.
 */

#ifndef SPARSE_PLANNER_HPP
#define SPARSE_PLANNER_HPP

#include <vector>
#include <assert.h>

#include "utilities/parameter_reader.hpp"
#include "systems/system.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"
#include "motion_planners/tree_node.hpp"

// ROS
#include <ros/console.h>
#include <ros/assert.h>


#define SMALL_EPSILON 0.01
#define PROP_NUM 100

/**
 * @brief The base class for motion planners.
 * @details The base class for motion planners. This class provides 
 * methods for visualizing the tree structures produced by motion
 * planners, in addition to initialization functions.
 */
class planner_t
{
public: 
	/**
	 * @brief Planner Constructor
	 * @details Planner Constructor
	 * 
	 * @param in_system The system this planner will plan for.
	 */
	planner_t(system_t* in_system)
	{
		system = in_system;
		start_state = NULL;
		goal_state = NULL;
		number_of_nodes=0;
	}
	virtual ~planner_t()
	{

	}

	/**
	 * @brief Perform any initialization tasks required before calling step().
	 * @details Perform any initialization tasks required before calling step().
	 */
	virtual void setup_planning() = 0;

	/**
	 * @brief Get the solution path.
	 * @details Query the tree structure for the solution plan for this given system.
	 * 
	 * @param controls The list of controls and durations which comprise the solution.
	 */
	virtual void get_solution(std::vector<std::pair<double*,double> >& controls) = 0;

	/**
	 * @brief Get the solution path.
	 * @details Query the tree structure for the solution plan for this given system.
	 * 
	 * @param controls The list of controls, durations and point-state which comprise the solution.
	 */
	virtual void get_solution(std::vector<std::tuple<double*, double, double*, double> >& controls, bool asses_risk = false ) = 0;

	/**
	 * @brief Perform an iteration of a motion planning algorithm.
	 * @details Perform an iteration of a motion planning algorithm.
	 */
	virtual void step() = 0;

	/**
	 * @brief Set the start state for the planner.
	 * @details Set the start state for the planner.
	 * 
	 * @param in_start The start state.
	 */
	virtual void set_start_state(double* in_start);

	/**
	 * @brief Set the goal state for the planner.
	 * @details Set the goal state for the planner.
	 * 
	 * @param in_goal The goal state
	 * @param in_radius The radial size of the goal region centered at in_goal.
	 */
	virtual void set_goal_state(double* in_goal,double in_radius);

	/**
	 * @brief Generate an image visualizing the tree.
	 * @details Generate an image visualizing the tree.
	 * 
	 * @param image_counter A subscript for the image file name. Allows for multiple image output.
	 */
	void visualize_tree(int image_counter);

	/**
	 * @brief Generate an image visualizing the nodes in the tree.
	 * @details Generate an image visualizing the nodes in the tree. The nodes will have a grayscale
	 * color corresponding to their relative cost to the maximum in the tree.
	 * 
	 * @param image_counter A subscript for the image file name. Allows for multiple image output.
	 */
	void visualize_nodes(int image_counter);

	/**
	 * @brief Find the maximum cost node in the tree.
	 * @details Find the maximum cost node in the tree.
	 */
	void get_max_cost();

	/** @brief The number of nodes in the tree. */
	unsigned number_of_nodes;

	/**
	 * @brief Get the root of the tree
	 * @details Return the root of the generated tree
	 * @return The root of the tree
	 */
	tree_node_t* get_root();

	/**
	 * @brief Get the solition path
	 * @details Get the last solution path
	 * 
	 * @param last_sln Variable where to store the path
	 */
	void get_last_solution_path(std::vector<tree_node_t*> & last_sln);

	/**
	 * @brief Set the risk aversion
	 * @details Set the risk aversion parameter
	 * 
	 * @param risk_aversion Risk aversion value in (0, 1]
	 */
	void set_risk_aversion(double risk_aversion);

	/**
	 * @brief Update the tree for replanning
	 * @details Remove old/not valid branches of the tree to be used in the next replanning cycle
	 * 
	 * @param delta_t The duration to determine the old/not valid branches
	 */
	virtual void replanning_update_tree(double delta_t, double* &new_state_point) = 0;

	/**
	 * @brief Set the dynamic obstacles for this iteration
	 * @details Set the dynamic obstacles for this iteration
	 */
	virtual void forward_risk_propagation() = 0;
	
	/**
	 * @brief Update node risks
	 * @details Update node risks according to the dynamic obstacles (eventually to the static also?)
	 */
	virtual void update_tree_risks() = 0;

protected:

	/**
	 * @brief Create geometries for visualizing the solution path.
	 * @details Create geometries for visualizing the solution path.
	 * 
	 * @param doc The image storage.
	 * @param dim The size of the image.
	 */
	virtual void visualize_solution_path( svg::Document& doc, svg::Dimensions& dim);

	/**
	 * @brief Create geometries for visualizing the nodes along the solution path.
	 * @details Create geometries for visualizing the nodes along the solution path.
	 * 
	 * @param doc The image storage.
	 * @param dim The size of the image.
	 */
	virtual void visualize_solution_nodes( svg::Document& doc, svg::Dimensions& dim);

	/**
	 * @brief A recursive function for finding the highest cost in the tree.
	 * @details A recursive function for finding the highest cost in the tree.
	 * 
	 * @param node The current node being examined.
	 */
	virtual void get_max_cost(tree_node_t* node);

	/**
	 * @brief Creates a single edge geometry with a node's parent.
	 * @details Creates a single edge geometry with a node's parent.
	 * 
	 * @param node The target node of the edge.
	 * @param doc The image storage.
	 * @param dim The size of the image.
	 */
	virtual void visualize_edge(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim);

	/**
	 * @brief Creates a single node geometry.
	 * @details Creates a single node geometry.
	 * 
	 * @param node The node to visualize.
	 * @param doc The image storage.
	 * @param dim The size of the image.
	 */
	virtual void visualize_node(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim);

	virtual void propagate_risk_backwards(tree_node_t* node, int parent_num) = 0;
	
	virtual bool propagate_risk_forward(tree_node_t* node, int node_num) = 0;
	
	double propagating_function(double parent_risk, double current_risk, double gamma);


 	/**
 	 * @brief The stored solution from previous call to get_solution.
 	 */
    std::vector<tree_node_t*> last_solution_path;

    /**
     * @brief A temporary storage for sorting nodes based on cost.
     */
    std::vector<tree_node_t*> sorted_nodes;

    /**
     * @brief The tree of the motion planner starts here.
     */
	tree_node_t* root;

	/**
	 * @brief The nearest neighbor data structure.
	 */
	graph_nearest_neighbors_t* metric;

	/**
	 * @brief The system being planned for.
	 */
	system_t* system;

	/**
	 * @brief The start state of the motion planning query.
	 */
	double* start_state;

	/**
	 * @brief The goal state of the motion planning query.
	 */
	double* goal_state;

	/**
	 * @brief The size of the spherical goal region around the goal state.
	 */
	double goal_radius;

	/**
	 * @brief The maximum cost found in the tree.
	 */
	double max_cost;

	/**
	 * @brief The user-defined risk aversion
	 */
	double risk_aversion;

	/**
	 * @brief The inverse of risk_aversion
	 */
	double inv_risk_aversion;


};


#endif