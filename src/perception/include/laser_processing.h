#ifndef _LASER_PROCESSING_HEADER_
#define _LASER_PROCESSING_HEADER_

// std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm> 
#include <vector>
#include <iterator>

// ros
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <perception/laser_processingConfig.h>


// ros messages
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

class laser_processing
{
	private:
		// the node handle
        ros::NodeHandle nh_;

        // Node handle in the private namespace
        ros::NodeHandle priv_nh_;

        // subscribers
        // ros::Subscriber sub_name;
        ros::Subscriber sub_laser_scan;

        // publishers
        // ros::Publisher pub_name;
        ros::Publisher pub_ocupancy_grid;

        float laser_range_min;        // minimum range value [m]
		float laser_range_max;        // maximum range value [m]
        float laser_ranges[360];

        nav_msgs::OccupancyGrid laser_grid;

     	float cell_resolution; //cell resolution in m/cell
		int grid_width; // 
		int grid_height;

		int central_grid_cell;

		

		void callback_laser_scan(const sensor_msgs::LaserScan& msg);

		void fill_occupancyGrid();


    public:
    	laser_processing(ros::NodeHandle nh);

    	virtual ~laser_processing();

    	void publish_topics();

    	void config_callback(perception::laser_processingConfig &config, uint32_t level);

};

#endif 
