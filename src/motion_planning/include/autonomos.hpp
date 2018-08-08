#ifndef AUTONOMOS_HPP
#define AUTONOMOS_HPP

#include "systems/system.hpp"

#include "collision_detector.h"

// ros msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

#define RANDOM_CTRL "RANDOM_CTRL"
#define BANG_BANG   "BANG_BANG"

class autonomos_t : public system_t
{
public:
	autonomos_t(std::string ctrl_to_use_in);

	virtual ~autonomos_t();

	double distance(double* point1, double* point2);

	void random_state(double* state);

	void random_control(double* control);

	bool propagate(double* start_state, double* control, int min_step,
    int max_step, double* result_state, double& duration );

	void enforce_bounds();

	bool valid_state();

	svg::Point visualize_point(double* state, svg::Dimensions dims);

	void set_obstacles(std::vector<geometry_msgs::Pose> vec_obstacles_poses,
	  std::vector<int> vec_obstacles_type, double in_obstacles_radius);

	void bang_bang_ctrl(double* control);

private:
	collision_detector_t* collision_detector;
	double obstacles_radius;
	std::string ctrl_to_use;

};


#endif
