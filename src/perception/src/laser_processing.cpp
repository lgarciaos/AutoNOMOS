#include "laser_processing.h"

#define RATE_HZ 5
using namespace std;



laser_processing::laser_processing(ros::NodeHandle nh)
    : nh_(nh), priv_nh_("~")
{
    std::string node_name = ros::this_node::getName();

    ROS_INFO("Node name: %s",node_name.c_str());

    // priv_nh_.param<std::string>(node_name+"/camera_name", camera_name, "/usb_cam/image_raw"); 
    // priv_nh_.param<int>(node_name+"/cam_w", cam_w, 640);
    priv_nh_.param<float>(node_name+"/laser_range_min", laser_range_min, .008);
    priv_nh_.param<float>(node_name+"/laser_range_max", laser_range_max, 6);
    priv_nh_.param<float>(node_name+"/cell_resolution", cell_resolution, 0.25);

    // priv_nh_.param<int>(node_name+"/grid_width",   grid_width, (int) 2 * laser_range_max / cell_resolution);
    // priv_nh_.param<int>(node_name+"/grid_height", grid_height, (int) 2 * laser_range_max / cell_resolution);
    grid_width = (int) 2 * laser_range_max / cell_resolution;
    grid_height = (int) 2 * laser_range_max / cell_resolution;

    ROS_INFO_STREAM("laser_range_max: " << laser_range_max);
    ROS_INFO_STREAM("cell_resolution: " << cell_resolution);
    ROS_INFO_STREAM("grid: " << grid_width << ", " << grid_height);

    sub_laser_scan = nh.subscribe("/scan", 1, &laser_processing::callback_laser_scan, this);
    pub_ocupancy_grid = nh.advertise<nav_msgs::OccupancyGrid>("/laser_occupancy_grid", 1);

    laser_grid.header.seq = 0;
    laser_grid.header.stamp = ros::Time::now();
    laser_grid.header.frame_id = "0";

    laser_grid.info.map_load_time = ros::Time::now();
    laser_grid.info.resolution = cell_resolution;
    laser_grid.info.width = grid_width;
    laser_grid.info.height = grid_height;


    int8_t grid[grid_width * grid_height];
    laser_grid.data.clear();
    vector<int> myvector (grid_width * grid_height);
    laser_grid.data.resize(grid_width * grid_height);
    fill(laser_grid.data.begin(), laser_grid.data.end(), -1);

    central_grid_cell = grid_width * grid_height + grid_width / 2;
}

void laser_processing::callback_laser_scan(const sensor_msgs::LaserScan& msg)
{
	// laser_ranges = msg.ranges;
	// double arr[100];
	copy(msg.ranges.begin(), msg.ranges.end(), laser_ranges);
}

void laser_processing::fill_occupancyGrid()
{
	// laser_grid
	// for (int i = 0; i < count; ++i)
	// {
	// 	/* code */
	// }

}

void laser_processing::publish_topics()
{	
	pub_ocupancy_grid.publish(laser_grid);
}

void laser_processing::config_callback(perception::laser_processingConfig &config, uint32_t level)
{

}

laser_processing::~laser_processing()
{	
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_processing");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE_HZ);

    laser_processing node = laser_processing(nh);

    dynamic_reconfigure::Server<perception::laser_processingConfig> server;
    dynamic_reconfigure::Server<perception::laser_processingConfig>::CallbackType f;
    f = boost::bind(&laser_processing::config_callback, &node, _1, _2);
    server.setCallback(f);

    // //ROS_INFO_STREAM("Before while");
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        node.publish_topics();
        // //ROS_INFO_STREAM("At while");
    }
    return 0;
}