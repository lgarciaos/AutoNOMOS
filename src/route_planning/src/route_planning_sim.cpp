#include <ros/ros.h>

#include <fstream>

#include "route_planning/route_state.h"

#define RANGE 0.5
#define SQUARE_RANGE 1.0
#define IN_VECINITY(value, center) (value <= center + RANGE && value >= center - RANGE)
#define BETWEEN(value, upper, bottom) (value <= upper && value >= bottom)
#define IN_SQUARE(x, y, x_square, y_square) ( x <= x_square + SQUARE_RANGE && x >= x_square - SQUARE_RANGE && y <= y_square + SQUARE_RANGE && y >= y_square - SQUARE_RANGE)


/////// VARIABLES ///////
int seq;
std::vector<std::pair<double, double>> points;

namespace params
{
    std::string txt_file_name;
}

/////// FUNCIONES ///////
int get_segment_num(double x, double y);
void fill_res(route_planning::route_state::Response &res, int segment, double distance, double x, double y);
// void set_next_state(route_planning::route_state::Request &req);
double get_max_segment_dist(int segment, double x, double y);


bool route_next_state(route_planning::route_state::Request &req,
                      route_planning::route_state::Response &res)
{

    // fill_res(res, seg, req.distance);
    double distance = 0, dist_aux, x, y;
    int seg;
    x = req.current_state.x;
    y = req.current_state.y;
    seg = get_segment_num(req.current_state.x, req.current_state.y);
    // set_next_state(req);

    res.next_state.x = -2;
    res.next_state.y = -6.35;
    res.next_state.theta = M_PI/2;
    return true;

    while (distance < req.distance)
    {
        dist_aux = get_max_segment_dist(seg, x, y);
        if ( dist_aux + distance >= req.distance)
        {
            fill_res(res, seg, req.distance - distance, x, y);
            ROS_WARN("Res: (%.3f, %.3f, %.3f)", res.next_state.x, res.next_state.y, res.next_state.theta);
            return true;
        }
        else
        {
            x = points[seg - 1].first;
            y = points[seg - 1].second;
            seg = (seg % 24 ) + 1;
            
        }
        distance += dist_aux;
        ROS_WARN_STREAM("DISTANCE: " << distance << "\tsegment: " << seg);
    }



    return false;
}

int get_segment_num(double x, double y)
{
    int segment = 0;

    if ( IN_VECINITY(x, -7.5) && BETWEEN(y, 6.5, -6.5) ) // segment 1
    {
        segment = 1;
    }
    else if ( IN_SQUARE(x, y, -7.5, -7.5 ) )
    {
        segment = 2;
    }
    else if ( BETWEEN(x, 6.5, -6.5) && IN_VECINITY(y, -7.5) )
    {
        segment = 3;
    }
    else if ( IN_SQUARE(x, y, 7.5, -7.5) )
    {
        segment = 4;
    }
    else if (IN_VECINITY(x, 7.5) && BETWEEN(y, -2.5, -6.5) )
    {
        segment = 5;
    }
    else if ( IN_SQUARE(x, y, 7.5, -1.5 ) )
    {
        segment = 6;
    }
    else if ( BETWEEN(x, 6.5, -3.5) && IN_VECINITY(y, -1.5) )
    {
        segment = 7;
    }
    else if ( IN_SQUARE(x, y, -4.5, -1.5) )
    {
        segment = 8;
    }
    else if ( IN_VECINITY(x, -4.5) && BETWEEN(y, -2.5, -4.5) )
    {
        segment = 9;
    }
    else if ( IN_SQUARE(x, y, -4.5, -4.5) )
    {
        segment = 10;
    }
    else if ( BETWEEN(x, 3.5, -3.5) && IN_VECINITY(y, -5.5) )
    {
        segment = 11;
    }
    else if ( IN_SQUARE(x, y, 4.5, -5.5) )
    {
        segment = 12;
    }
    else if ( IN_VECINITY(x, 4.5) && BETWEEN(y, 4.5, -4.5) )
    {
        segment = 13;
    }
    else if ( IN_SQUARE(x, y, 4.5, 5.5) )
    {
        segment = 14;
    }
    else if ( BETWEEN(x, 3.5, -3.5) && IN_VECINITY(y, 5.5) )
    {
        segment = 15;
    }
    else if ( IN_SQUARE(x, y, -4.5, 5.5) )
    {
        segment = 16;
    }
    else if ( IN_VECINITY(x, -4.5) && BETWEEN(y, 4.5, 2.5) )
    {
        segment = 17;
    }
    else if ( IN_SQUARE(x, y, -4.5, 1.5) )
    {
        segment = 18;
    }
    else if ( BETWEEN(x, 6.5, -3.5) && IN_VECINITY(y, 1.5) )
    {
        segment = 19;
    }
    else if ( IN_SQUARE(x, y, 7.5, 1.5) )
    {
        segment = 20;
    }
    else if ( IN_VECINITY(x, 7.5) && BETWEEN(y, 6.5, 2.5) )
    {
        segment = 21;
    }
    else if ( IN_SQUARE(x, y, 7.5, 7.5) )
    {
        segment = 22;
    }
    else if ( BETWEEN(x, 6.5, -6.5) && IN_VECINITY(y, 7.5) )
    {
        segment = 23;
    }
    else if ( IN_SQUARE(x, y, -7.5, 7.5) )
    {
        segment = 24;
    }

    return segment;
}

double get_max_segment_dist(int segment, double x, double y)
{
    // double ang, diam;
    // diam = 2;
    double res;
    switch ( segment )
    {
        case 1: 
            res = abs(y - -6.5);
            break;
        case 2: 
            res = sqrt( pow(x - -6.5, 2) + pow(y - -7.5, 2));
            break;
        case 3:
            res = abs(x - 6.5);
            break;
        case 4: 
            res = sqrt( pow(x - 7.5, 2) + pow(y - -6.5, 2));
            break;
        case 5:
            res = abs(y - -2.5);
            break;
        case 6: 
            res = sqrt( pow(x - 6.5, 2) + pow(y - -1.5, 2));
            break;
        case 7:
            res = abs(x - -3.5);
            break;
        case 8: 
            res = sqrt( pow(x - -4.5, 2) + pow(y - -2.5, 2));
            break;
        case 9:
            res = abs(y - -4.5);
            break;
        case 10: 
            res = sqrt( pow(x - -3.5, 2) + pow(y - -4.5, 2));
            break;
        case 11:
            res = abs(x - 3.5);
            break;
        case 12: 
            res = sqrt( pow(x - 4.5, 2) + pow(y - -4.5, 2));
            break;
        case 13:
            res = abs(y - 4.5);
            break;
        case 14: 
            res = sqrt( pow(x - 3.5, 2) + pow(y - 5.5, 2));
            break;
        case 15:
            res = abs(x - -3.5);
            break;
        case 16: 
            res = sqrt( pow(x - -4.5, 2) + pow(y - 4.5, 2));
            break;
        case 17:
            res = abs(y - 2.5);
            break;
        case 18: 
            res = sqrt( pow(x - -3.5, 2) + pow(y - 1.5, 2));
            break;
        case 19:
            res = abs(x - 6.5);
            break;
        case 20: 
            res = sqrt( pow(x - 7.5, 2) + pow(y - 2.5, 2));
            break;
        case 21:
            res = abs(y - 6.5);
            break;
        case 22: 
            res = sqrt( pow(x - 6.5, 2) + pow(y - 7.5, 2));
            break;
        case 23:
            res = abs(x - -6.5);
            break;
        case 24: 
            res = sqrt( pow(x - -7.5, 2) + pow(y - 6.5, 2));
            break;
        default:
            ROS_ERROR("INVALID SEGMENT");
            res = -1;
    }
    ROS_WARN_STREAM("get_max_segment_dist: " << res);
    return res;
}

void fill_res(route_planning::route_state::Response &res, int segment, double distance, double x, double y)
{
    
    switch ( segment )
    {
        case 1: 
            res.next_state.x = -7.5;
            res.next_state.y = y - distance;
            res.next_state.theta = 3 * M_PI / 2.0;
            ROS_WARN("CASE 1: (%.1f, %.2f, %.3f)", -7.5, y-distance, 3 * M_PI / 2.0);
            ROS_WARN("CASE 1: (%.1f, %.2f, %.3f)", res.next_state.x, res.next_state.y, res.next_state.theta);
            break;
        case 2: 
            res.next_state.x = x + distance / 2.0;
            res.next_state.y = y - distance / 2.0;
            res.next_state.theta = 7 * M_PI / 4.0;
            break;
        case 3:
            res.next_state.x = x + distance;
            res.next_state.y = -7.5;
            res.next_state.theta = 0;
            break;
        case 4: 
            res.next_state.x = x + distance / 2.0;
            res.next_state.y = y + distance / 2.0;
            res.next_state.theta = M_PI / 4.0;
            break;
        case 5:
            res.next_state.x = 7.5;
            res.next_state.y = y + distance;
            res.next_state.theta = M_PI / 2.0;
            break;
        case 6: 
            res.next_state.x = x - distance / 2.0;
            res.next_state.y = y + distance / 2.0;
            res.next_state.theta = 3 * M_PI / 4.0;
            break;
        case 7:
            res.next_state.x = x - distance;
            res.next_state.y = -1.5;
            res.next_state.theta = M_PI;
            break;
        case 8: 
            res.next_state.x = x - distance / 2.0;
            res.next_state.y = y - distance / 2.0;
            res.next_state.theta = 5 * M_PI / 4.0;
            break;
        case 9:
            res.next_state.x = -4.5;
            res.next_state.y = y - distance;
            res.next_state.theta = 3 * M_PI / 2.0;
            break;
        case 10: 
            res.next_state.x = x + distance / 2.0;
            res.next_state.y = y - distance / 2.0;
            res.next_state.theta = 7 * M_PI / 4.0;
            break;
        case 11:
            res.next_state.x = x + distance;
            res.next_state.y = -5.5;
            res.next_state.theta = 0;
            break;
        case 12: 
            res.next_state.x = x + distance / 2.0;
            res.next_state.y = y + distance / 2.0;
            res.next_state.theta = M_PI / 4.0;
            break;
        case 13:
            res.next_state.x = 4.5;
            res.next_state.y = y + distance;
            res.next_state.theta = M_PI / 2.0;
            break;
        case 14: 
            res.next_state.x = x - distance / 2.0;
            res.next_state.y = y + distance / 2.0;
            res.next_state.theta = 3 * M_PI / 4.0;
            break;
        case 15:
            res.next_state.x = x - distance;
            res.next_state.y = 5.5;
            res.next_state.theta = M_PI;
            break;
        case 16: 
            res.next_state.x = x - distance / 2.0;
            res.next_state.y = y - distance / 2.0;
            res.next_state.theta = 5 * M_PI / 4.0;
            break;
        case 17:
            res.next_state.x = -4.5;
            res.next_state.y = y - distance;
            res.next_state.theta = 3 * M_PI / 2.0;
            break;
        case 18: 
            res.next_state.x = x + distance / 2.0;
            res.next_state.y = y - distance / 2.0;
            res.next_state.theta = 7 * M_PI / 4.0;
            break;
        case 19:
            res.next_state.x = x + distance;
            res.next_state.y = 1.5;
            res.next_state.theta = 0;
            break;
        case 20: 
            res.next_state.x = x + distance / 2.0;
            res.next_state.y = y + distance / 2.0;
            res.next_state.theta = M_PI / 4.0;
            break;
        case 21:
            res.next_state.x = 7.5;
            res.next_state.y = y + distance;
            res.next_state.theta = M_PI / 2.0;
            break;
        case 22: 
            res.next_state.x = x - distance / 2.0;
            res.next_state.y = y + distance / 2.0;
            res.next_state.theta = 3 * M_PI / 4.0;
            break;
        case 23:
            res.next_state.x = x - distance;
            res.next_state.y = 7.5;
            res.next_state.theta = M_PI;
            break;
        case 24: 
            res.next_state.x = x - distance / 2.0;
            res.next_state.y = y - distance / 2.0;
            res.next_state.theta = 5 * M_PI / 4.0;
            break;
        default:
            ROS_ERROR("INVALID SEGMENT");
    }
    // res.next_state.theta = 0;
    ROS_WARN("Segment: %d\tdist: %.2f\t (%.2f,%.2f)", segment, distance, x, y);
    ROS_WARN("Next point: ( %.2f, %.2f, %.3f )", res.next_state.x, res.next_state.y, res.next_state.theta);
}

void fill_points()
{
    ROS_WARN_STREAM("File name is: " << params::txt_file_name);
    std::ifstream ifile(params::txt_file_name, std::ios::in);

    //check to see that the file was opened correctly:
    if (!ifile.is_open()) {
        ROS_ERROR("There was a problem opening the input file!");
        exit(1);//exit or do additional error checking
    }

    int i = 0;
    double num = 0.0;
    //keep storing values from the text file so long as data exists:
    std::pair<double, double> p;
    while (ifile >> num) {
        // scores.push_back(num);
        if (i % 2 == 0)
        {
            p.first = num;
        }
        else
        {
            p.second = num;
            points.push_back(p);
        }
        i++;
    }
    ROS_DEBUG_STREAM("Printing" << points.size() <<" points:");
    for (auto p : points )
    {
        ROS_DEBUG("( %.1f, %.1f ) ", p.first, p.second );
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_processing");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    // ros::Rate loop_rate(RATE_HZ);
    nh_priv.param<std::string>     ("points_file_name",  params::txt_file_name, "route_planning/input/points_1.txt");

    fill_points();

    ros::ServiceServer service_next_ctrl = nh.advertiseService("/route_planning/next_state", &route_next_state);

    seq = 0;
    // //ROS_INFO_STREAM("Before while");
    while(ros::ok())
    {
        ros::spin();
        // node.process();
        // node.publish_topics();
        // loop_rate.sleep();
        // //ROS_INFO_STREAM("At while");
    }
    return 0;
}