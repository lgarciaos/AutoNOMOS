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
 */

#ifndef RRT_ROS_HPP
#define RRT_ROS_HPP

#include <vector>

#include "utilities/parameter_reader.hpp"
#include "systems/system.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"
#include "motion_planners/tree_node.hpp"
#include "motion_planners/planner.hpp"
#include "motion_planners/rrt.hpp"
#include "motion_planners/sst.hpp"

// ros msgs
#include "std_msgs/Float64MultiArray.h"

/**
 * @brief The base class for motion planners.
 * @details The base class for motion planners. This class provides
 * methods for visualizing the tree structures produced by motion
 * planners, in addition to initialization functions.
 */
class rrt_ros_t : public rrt_t
{
public:
	/**
	 * @brief Planner Constructor
	 * @details Planner Constructor
	 *
	 * @param in_system The system this planner will plan for.
	 */
  rrt_ros_t(system_t* in_system) : rrt_t(in_system){};
    //sst_t(in_system), rrt_t(in_system){};

  virtual ~rrt_ros_t();

  std_msgs::Float64MultiArray get_vector_path();
  std_msgs::Float64MultiArray get_vector_tree();

  // std_msgs::Float64MultiArray get_vector_();

protected:

  // using planner_t::last_solution_path;
  // using sst_t::last_solution_path;


  void create_floatmultiarray_tree(tree_node_t* node);

  std_msgs::Float64MultiArray tree_floatmultiarray;

};


#endif
