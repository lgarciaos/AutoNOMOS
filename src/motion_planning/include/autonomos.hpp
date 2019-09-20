#ifndef AUTONOMOS_HPP
#define AUTONOMOS_HPP

#include "systems/system.hpp"

#include "collision_detector.h"

// ros msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

// ros
#include <ros/console.h>
#include <ros/assert.h>

// libccd
#include <ccd/ccd.h>
#include <ccd/quat.h>
#include <ccd/vec3.h>

#define RANDOM_CTRL "RANDOM_CTRL"
#define BANG_BANG   "BANG_BANG"

using std::string;

class autonomos_t : public system_t
{
public:
	autonomos_t(std::string ctrl_to_use_in, bool global_planning, string name);

	virtual ~autonomos_t();

	double distance(double* point1, double* point2, bool only_geometric = false);

	void random_state(double* state);

	void random_control(double* control){random_control(control, false);};
	void random_control(double* control, bool allow_reverse = false);

	bool propagate(double* start_state, double* control, int min_step,
    int max_step, double* result_state, double& duration );

	void enforce_bounds();

	bool valid_state();

	svg::Point visualize_point(double* state, svg::Dimensions dims);

	void set_obstacles(motion_planning::obstacles_array::Response msg);


	void bang_bang_ctrl(double* control);

	// std::set<obstacle_t> get_dynamic_obstacles();
	bool get_next_dynamic_state(double *state, int i);


	/**
	 * @brief Set rectangular bounds
	 * @details Set rectangular bounds
	 * 
	 * @param pos_x_bound Max +X allowed
	 * @param neg_x_bound Min -X allowed
	 * @param pos_y_bound Max +Y allowed
	 * @param neg_y_bound Min -Y allowed
	 */
	void set_bounds( double pos_x_bound, double neg_x_bound, double pos_y_bound, double neg_y_bound );

	/**
	 * @brief Set circular local bound
	 * @details Set circular local bound
	 * 
	 * @param planning_rad_range Radius of circular local bound
	 */
	void set_bounds( double planning_rad_range );

	void set_current_loc(double x, double y, double theta);

	// void set_allow_reverse(bool allow_reverse){allow_reverse = allow_reverse;}

private:

	static void support(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);

	/**
	 * @brief Enforce global rectangular bound
	 * @details If X or Y is outside the desired planning
	 *  	rectangle world, dont use that state
	 */
	void global_rectangular_bound();
	
	/**
	 * @brief enforce local circular bound
	 * @details If the distance of the propose 
	 * 		state is more than the planning_rad_range
	 * 		prevent using such state
	 */
	void local_circular_bound();

	/**
	 * Store if some state goes out of bounds
	 */
	bool in_bounds; 

	/**
	 * True if planning globally
	 */
	bool global;

	/**
	 * To store the current pose of the robot
	 */
	geometry_msgs::Pose2D current_loc;

	// bool allow_reverse;

  	ccd_t ccd;
  	obstacle_t robot_obj;

    std::set<obstacle_t> static_obstacles;
    std::set<obstacle_t> dynamic_obstacles;

	double pos_x_bound; ///< rectangular bounds 
	double neg_x_bound; ///< rectangular bounds 
	double pos_y_bound; ///< rectangular bounds 
	double neg_y_bound; ///< rectangular bounds

	double planning_rad_range; ///< circular radius bound

	// collision_detector_t* collision_detector;
	double obstacles_radius;
	std::string ctrl_to_use;

	string autonomos_name;

	// std::set<obstacle_t>::iterator it_obs;
};


#endif
