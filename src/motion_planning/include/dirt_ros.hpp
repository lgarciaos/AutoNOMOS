#ifndef DIRT_ROS_HPP
#define DIRT_ROS_HPP

#include <vector>

#include "utilities/parameter_reader.hpp"
#include "systems/system.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"
#include "motion_planners/tree_node.hpp"
#include "motion_planners/planner.hpp"
#include "motion_planners/rrt.hpp"
#include "motion_planners/sst.hpp"
#include "motion_planners/dirt.hpp"

// ros msgs
#include <std_msgs/Float64MultiArray.h>




/**
 * @brief The base class for motion planners.
 * @details The base class for motion planners. This class provides
 * methods for visualizing the tree structures produced by motion
 * planners, in addition to initialization functions.
 */
class dirt_ros_t : public dirt_t
{
public:
	/**
	 * @brief Planner Constructor
	 * @details Planner Constructor
	 *
	 * @param in_system The system this planner will plan for.
	 */
  dirt_ros_t(system_t* in_system) : dirt_t(in_system)
  {
    // ctrl_to_use = ctrl_to_use_in;
  };
    //sst_t(in_system), rrt_t(in_system){};

  virtual ~dirt_ros_t();

  // void set_ctrl_to_use(int ctrl_to_use_in);
  std_msgs::Float64MultiArray get_vector_path();
  std_msgs::Float64MultiArray get_vector_tree();
  // motion_planning::ctrl_path get_ctrl_path(
  //   std::vector<std::pair<double*,double> >& controls);

  // void random_sample();

  void dealloc_tree();
  // std_msgs::Float64MultiArray get_vector_();

protected:

  // using planner_t::last_solution_path;
  // using sst_t::last_solution_path;


  void create_floatmultiarray_tree(tree_node_t* node);
  void dealloc_tree_aux(tree_node_t* node);
  void dealloc_metric();


  std_msgs::Float64MultiArray tree_floatmultiarray;

};


#endif
