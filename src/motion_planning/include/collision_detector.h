#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>

// libccd
#include <ccd/ccd.h>
#include <ccd/quat.h>
// own
#include "obstacle_t.hpp"
#include "motion_planning/obstacles_array.h"
// types of obstacles
#define RECTANGLE 0
#define CIRCLE 1

// car dimensions
#define CAR_SIZE_Y 0.2
#define CAR_SIZE_X 0.125 

/**
 * @brief
 * @details
 */
class collision_detector_t
{
  public:

    std::vector<geometry_msgs::Pose> obstacles_poses;
    std::vector<int> obstacles_types;

    std::set<obstacle_t> static_obstacles;
    std::set<obstacle_t> dynamic_obstacles;

    collision_detector_t()
    {
    }
    ~collision_detector_t(){};

    void support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec);

    void set_obstacles(motion_planning::obstacles_array::Response msg);

    bool is_collision_free(geometry_msgs::Pose2D start_node, geometry_msgs::Pose2D end_node);

    bool pose_in_car(geometry_msgs::Pose2D node, geometry_msgs::Pose center_car, bool print);

    bool path_intersects_obstacle(geometry_msgs::Pose2D start_node,
      geometry_msgs::Pose2D end_node, geometry_msgs::Pose obstacle, int obstacle_type);

    void set_obstacles_radius(double in_obstacles_radius);

  protected:

    /**
  	 * @brief return the distance from one node to the other
  	 * @details return the distance from one node to the other
  	 */
    double distance(geometry_msgs::Pose2D p1, geometry_msgs::Pose2D p2);

    /**
     * @brief The tolerance radius to the goal
     */
    double obstacles_radius;

  private:

};

#endif
