

#ifndef _referee_HEADER_
#define _referee_HEADER_

#include <iostream>

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>

// #include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int16.h>
// #include "msg/pts_array.msg"

using namespace std;

#define STATE_DONT_KNOW_LEFT 0
#define STATE_OUTSIDE_LEFT 1
#define STATE_LEFT_LEFT 2
#define STATE_LEFT_CENTER 3
#define STATE_CENTER_CENTER 4
#define STATE_RIGHT_CENTER 5
#define STATE_RIGHT_RIGHT 6
#define STATE_OUTSIDE_RIGHT 7
#define STATE_DONT_KNOW_RIGHT 8
// #define STATE_
//The states are:
//            |         |         |         NI -> No se  Izq
//            |                   |         AI -> Afuera Izq
//            |         |         |         LL -> Left Left
//            |                   |         LC -> Left Center
//            |         |         |         CC -> Center Center
//            |                   |         RC -> Right Center
//            |         |         |         RR -> Right Right
//   NI | AI  |LL| LC |CC |RC |RR | AD | ND     AD -> Afuera Derecha
//   0  | 1   |2 | 3  |4  |5  |6  | 7  | 8      ND -> No se Derecha

#define RATE_HZ 10



class referee
{
    private:

        // the node handle
        ros::NodeHandle nh_;

        // Node handle in the private namespace
        ros::NodeHandle priv_nh_;

        // subscribers
        // ros::Subscriber read_images_;

        // publishers
        //ros::Publisher publish_curvature;


        // double rate_hz = 30;

    public:
                
    	referee(ros::NodeHandle nh);

    	virtual ~referee();
        
};

#endif 
