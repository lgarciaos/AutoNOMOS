/**
 * @file rrt.hpp
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

#ifndef SPARSE_PLANNER_RRT_HPP
#define SPARSE_PLANNER_RRT_HPP

#include "systems/system.hpp"
#include "motion_planners/planner.hpp"

#include <boost/lexical_cast.hpp>


/**
 * @brief The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 * @details The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 */
class rrt_t : public planner_t
{
public:
	/**
	 * @copydoc planner_t::planner_t(system_t*)
	 */
	rrt_t(system_t* in_system) : planner_t(in_system)
	{

	}
	virtual ~rrt_t(){}

	/**
	 * @copydoc planner_t::setup_planning()
	 */
	virtual void setup_planning();

	/**
	 * @copydoc planner_t::get_solution(std::vector<std::pair<double*,double> >&)
	 */
	virtual void get_solution(std::vector<std::pair<double*,double> >& controls);

	/**
	 * @copydoc planner_t::get_solution(std::vector<std::tuple<double*,double, double*> >&)
	 */
	virtual void get_solution(std::vector<std::tuple<double*,double, double*, double> >& controls, bool asses_risk = false);

	/**
	 * @copydoc planner_t::step()
	 */
	virtual void step();

	/**
	 * @copydoc planer_t::replanning_update_tree(double delta_t)
	 */
	virtual void replanning_update_tree(double delta_t, double* &new_state_point);

	/**
	 * @copydoc planer_t::forward_risk_propagation()
	 */
	virtual void forward_risk_propagation();
	
	/**
	 * @copydoc planer_t::update_tree_risks()
	 */
	virtual void update_tree_risks();
	
protected:
	
	/**
	 * @brief A randomly sampled state.
	 */
	double* sample_state;

	/**
	 * @brief A randomly sampled control.
	 */
	double* sample_control;


	/**
	 * @brief A resulting duration of a propagation step.
	 */
	double duration;

	/**
	 * @brief Storage used to query the nearest neighbor structure.
	 */
	tree_node_t* metric_query;

	/**
	 * @brief The result of a query in the nearest neighbor structure.
	 */
	tree_node_t* nearest;
	
	/**
	 * @brief A set of nodes used to get solutions.
	 */
	proximity_node_t** close_nodes;
	/**
	 * @brief A set of distances used to get solutions.
	 */
	double* distances;

	/**
	 * @brief Perform the random sampling step of RRT.
	 * @details Perform the random sampling step of RRT.
	 */
	void random_sample();

	/**
	 * @brief Find the nearest node to the randomly sampled state.
	 * @details Find the nearest node to the randomly sampled state.
	 */
	void nearest_vertex();

	/**
	 * @brief Perform a local propagation from the nearest state.
	 * @details Perform a local propagation from the nearest state.
	 * @return If the trajectory is collision free or not.
	 */
	bool propagate();

	/**
	 * @brief If propagation was successful, add the new state to the tree.
	 * @details If propagation was successful, add the new state to the tree.
	 */
	void add_to_tree();

	/**
	 * @brief Add a state into the nearest neighbor structure for retrieval in later iterations.
	 * @details Add a state into the nearest neighbor structure for retrieval in later iterations.
	 * 
	 * @param node The node to add.
	 */
	void add_point_to_metric(tree_node_t* node);


	void propagate_risk_backwards(tree_node_t* node, int parent_num);

	void propagate_risk_forward(tree_node_t* node, int node_num);
	
private:
	/**
	 * @brief aux recursive function to get the total cost with risk
	 * @details Auxiliary function to use recursivly to compute the cost when using risk nodes
	 * 
	 * @param node The current node from which the risk and cost are going to be evaluated
	 * @param node_num The number of the current node backwards (the root would be the ith node)
	 * 
	 * @return If the trajectory is feasable given the risk parameters
	 */
	bool get_solution_1(tree_node_t* node, int node_num, double& cost);


};

#endif