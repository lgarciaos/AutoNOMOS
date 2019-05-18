/**
 * @file timer.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "utilities/ros_timer.hpp"

#include <cmath>
#include <cstddef>

sys_ros_timer_t::sys_ros_timer_t()
{
    reset();
    elapsed = ros::Duration(0);
}

sys_ros_timer_t::~sys_ros_timer_t()
{}

double sys_ros_timer_t::get_time_in_secs()
{  
    finish = ros::Time::now();
    double s = finish.toSec();
    s += ( 0.000000001 * finish.toNSec() );
    return s;
}

double sys_ros_timer_t::duration_to_double(ros::Duration duration)
{
    return duration.toSec();
}

void sys_ros_timer_t::reset()
{
    start = ros::Time::now();
}


double sys_ros_timer_t::measure()
{
    finish = ros::Time::now();
    elapsed = finish - start;
    // ROS_WARN_STREAM("Start: " << start << " Finish: " << finish << " Elapsed: " << elapsed << " el_d: " << duration_to_double(elapsed));
    return elapsed.toSec();
}


double sys_ros_timer_t::measure_reset()
{
    measure();
    reset();
    return duration_to_double(elapsed);
}

void sys_ros_timer_t::add_delay_user_clock( double delay )
{
    ros::Duration delay_aux(delay);
    start += delay_aux;
}