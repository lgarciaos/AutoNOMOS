/**
 * @file a_star.h
 *
 */

#ifndef A_STAR_H
#define A_STAR_H

// std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iterator>
#include <regex>
#include <cassert>

// ros
#include <tf/transform_datatypes.h>

// ros msgs
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>

// ignition
// #include <ignition/math.hh>
// #include <ignition/math/Quaternion.hh>
// sparse_rrt
#include "systems/system.hpp"
#include "motion_planners/planner.hpp"

// own
#include "node_g.cpp"
#include "priority_queue_nodes.cpp"

// types of a_star
#define GRID "A_STAR_GRID"
#define CTRL "A_STAR_CTRL"

// types of obstacles
#define RECTANGLE 0
#define CIRCLE 1

// car dimensions
#define CAR_SIZE_X 0.2
#define CAR_SIZE_Y 0.125

/**
 * @brief The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 * @details The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 */
class a_star_t
{
  public:

    a_star_t()
    {
      std::cout << "a_star initialized" << '\n';
    }
    virtual ~a_star_t();

    /**
  	 * @brief Perform any initialization tasks required before calling step().
  	 * @details Perform any initialization tasks required before calling step().
  	 */
  	void setup_planning();

  	/**
  	 * @brief Get the solution path.
  	 * @details Query the tree structure for the solution plan for this given system.
  	 *
  	 * @param controls The list of controls and durations which comprise the solution.
  	 */
  	void get_solution(std::vector<std::pair<double*,double> >& controls);

  	/**
  	 * @brief Perform an iteration of a motion planning algorithm.
  	 * @details Perform an iteration of a motion planning algorithm.
  	 */
  	void step();

  	/**
  	 * @brief Set the start state for the planner.
  	 * @details Set the start state for the planner.
  	 *
  	 * @param in_start The start state.
  	 */
  	void set_start_state(geometry_msgs::Pose2D in_start);

  	/**
  	 * @brief Set the goal state for the planner.
  	 * @details Set the goal state for the planner.
  	 *
  	 * @param in_goal The goal state
  	 * @param in_radius The radial size of the goal region centered at in_goal.
  	 */
  	void set_goal_state(geometry_msgs::Pose2D in_goal,double in_radius);

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

    /**
  	 * @brief set the type: GRID or CTRL
  	 * @details sets the type of search is going to be performed: GRID or CTRL
  	 */
    void set_type(std::string in_type);

    void set_obstacles(std::vector<geometry_msgs::Pose> obstacle_poses,
      std::vector<int> obstacles_type);

    std::vector<geometry_msgs::Point> generate_grid(double x_inc, double y_inc,
      double grid_init_x, double grid_init_y, double grid_end_x,
      double grid_end_y);

    std::vector<geometry_msgs::Point> remove_obst_points(std::vector<geometry_msgs::Point> points_in);

    int get_total_nodes();

    /**
     * @brief The type of alg: grid or control.
     */
    std::string type;

    std::vector<geometry_msgs::Pose> obstacles_poses;
    std::vector<int> obstacles_type;

  protected:

    /**
  	 * @brief return the distance from one node to the other
  	 * @details return the distance from one node to the other
  	 */
    double distance(node_g* n1, node_g* n2);

    /**
  	 * @brief get the points adjecent to the current point
  	 * @details get the points adjecent to the current point depending on type
  	 */
    std::vector<node_g*> get_adj_points(node_g* node_ini,
                  std::vector<node_g*> nodes, std::string points_creation);

    /**
     * @brief gets the actual cost from last to next
     * @details gets the actual cost from last to next
     */
    double actual_cost(node_g *last, node_g *next);

    /**
     * @brief The heuristic used
     * @details Now the heuristsc is the distance
     */
    double h(node_g* now, node_g* goal);

    /**
     * @brief checks if element is in the vector
     * @details checks if element is in the vector
     */
    bool is_element_in_vector(std::vector<node_g*> vector, node_g* element);

    /**
     * @brief determines the distance driven by the car
     * @details determines the distance driven by the car
     */
    double get_distance_driven(int vel);

    bool is_collision_free(node_g* start_node, node_g* end_node);

    bool node_in_car(node_g* node, geometry_msgs::Pose center_car, bool print);

    bool path_intersects_obstacle(node_g* start_node, node_g* end_node,
      geometry_msgs::Pose obstacle, int obstacle_type);

    /**
  	 * @brief The nodes that are still candidates.
  	 */
    priority_queue_nodes open;

    /**
     * @brief The nodes that were already searched.
     */
    std::vector<node_g*> closed;

    /**
     * @brief All the nodes of the grid.
     */
    std::vector<node_g*> nodes_grid;

    /**
     * @brief The goal to be reached.
     */
    node_g* goal;

    /**
     * @brief The tolerance radius to the goal
     */
    double goal_radius;



    std::vector<geometry_msgs::Point> grid_points;

  private:


};

#endif
