#ifndef _OBSTACLE_T_HPP
#define _OBSTACLE_T_HPP

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// ros messages
#include <std_msgs/Header.h> 
#include <geometry_msgs/Polygon.h> 
#include <geometry_msgs/Pose.h> 
// ros
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
// #include <LinearMath/btMatrix3x3.h>

// #define ARE_NODES_PTR_EQUAL(A, B) (A -> point.x == B -> point.x & A -> point.y == B -> point.y)
using std::string;

class obstacle_t
{
  private:
    geometry_msgs::Polygon bounding_box;
    geometry_msgs::Pose pose;
    geometry_msgs::Point dimensions;
    string name;
    bool is_static;

  public:
    obstacle_t(){};
    obstacle_t(string obs_name, geometry_msgs::Pose pose_msg, geometry_msgs::Polygon polygon_msg, 
    bool is_static, geometry_msgs::Point dimensions)
    {
      // tf::poseMsgToTF(pose_msg, pose);
      pose = pose_msg;
      bounding_box = polygon_msg;
      name = obs_name;
      this -> is_static = is_static;
      this -> dimensions = dimensions;
    }

    void get_bounding_box_dimensions(double &x, double &y, double &z)
    {
      x = dimensions.x;
      y = dimensions.y;
      z = dimensions.z;
    }

    void set_pose2D(double x, double y, double theta)
    {
      pose.position.x = x;
      pose.position.y = y; 
      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);

      pose.orientation.x = q.x;
      pose.orientation.y = q.y;
      pose.orientation.z = q.z;
      pose.orientation.w = q.w;      
    } 

    void set_pose(double x, double y, double z, double qx, double qy, double qz, double qw)
    {
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;

      pose.orientation.x = qx;
      pose.orientation.y = qy;
      pose.orientation.z = qz;
      pose.orientation.w = qw;
    }

    void set_bounding_box_point(double x, double y, double z)
    {
      geometry_msgs::Point32 p;
      p.x = x;
      p.y = y;
      p.z = z;
      bounding_box.points.push_back(p);
    }

    void set_name(string obstacle_name)
    {
      name = obstacle_name;
    }

    void set_static(bool value)
    {
      is_static = value;
    }

    void get_xyz(double &x, double &y, double &z) const
    {
      x = pose.position.x;
      y = pose.position.y;
      z = pose.position.z;
    }

    void get_quat_xyzw(double &qx, double &qy, double &qz, double &qw)
    {
      qx = pose.orientation.x;
      qy = pose.orientation.y;
      qz = pose.orientation.z;
      qw = pose.orientation.w;
    }

    string get_name() const {return name;}

  friend bool operator< (const obstacle_t& o1, const obstacle_t &o2)
  {
    return o1.name < o2.name;
  }
  
  friend std::ostream& operator<<(std::ostream& os, const obstacle_t& n)
  {
    // double roll, pitch, yaw;
    // tf::Matrix3x3(n.pose.getRotation()).getRPY(roll, pitch, yaw);
    os << n.name << " - ( " << n.pose.position.x << ", " << n.pose.position.y 
      << " ). \t\nBB: - ( " << n.dimensions.x << ", " << n.dimensions.y << ", " << n.dimensions.z << " )";
      // << "\n\t1 - ( " 
      // << n.bounding_box.points[0].x << ", " << n.bounding_box.points[0].y << ", " << n.bounding_box.points[0].z 
      // << " )\n\t2 - ( "
      // << n.bounding_box.points[1].x << ", " << n.bounding_box.points[1].y << ", " << n.bounding_box.points[1].z
      // << " )\n\t3 - ( "
      // << n.bounding_box.points[2].x << ", " << n.bounding_box.points[2].y << ", " << n.bounding_box.points[2].z
      // << " )\n\t4 - ( "
      // << n.bounding_box.points[3].x << ", " << n.bounding_box.points[3].y << ", " << n.bounding_box.points[3].z
      // << " ).";

    return os;
  }

};

//Overload the < operator.
//Overload the > operator.
// bool operator> (const node_g& n1, const node_g &n2)
// {
//   return n1.cost > n2.cost;
// }

// bool operator==(const node_g& n1, const node_g& n2)
// {
//   return n1.point.x == n2.point.x && n1.point.y == n2.point.y;
// }

// bool operator!=(const node_g& n1, const node_g& n2)
// {
//   return !(n1 == n2);
// }

// struct node_g_ptrs_cmp
// {
//     bool operator()(const node_g* n1, const node_g* n2) const
//     {
//         return n1 -> cost < n2 -> cost;
//     }
// };

#endif
