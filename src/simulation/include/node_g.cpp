#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// ros messages
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
// #include <LinearMath/btMatrix3x3.h>

#define ARE_NODES_PTR_EQUAL(A, B) (A -> point.x == B -> point.x & A -> point.y == B -> point.y)

class node_g
{
  public:
    double cost;
    geometry_msgs::Point point;
    tf::Quaternion orientation;

    node_g *parent;

    void print_node_g(std::string s)
    {
      double roll, pitch, yaw;
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      // std::cout << "point: (" << point.x << ", " << point.y << ")" <<'\n';
      // std::cout << "parent: " << parent << '\n';
      // std::cout << "cost:" << cost << '\n';
      std::cout << "---" << s << ": cost = " << cost << " - (" << point.x << ", " << point.y << ") - " << roll <<". Parent " << parent << std::endl;

    }

    double get_roll()
    {
      double roll, pitch, yaw;
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      return roll;
    }

    void set_orientation(double roll, double pitch, double yaw)
    {
      orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);
    }

};
//Overload the < operator.
bool operator< (const node_g& n1, const node_g &n2)
{
  return n1.cost < n2.cost;
}
//Overload the > operator.
bool operator> (const node_g& n1, const node_g &n2)
{
  return n1.cost > n2.cost;
}

bool operator==(const node_g& n1, const node_g& n2)
{
  return n1.point.x == n2.point.x && n1.point.y == n2.point.y;
}

bool operator!=(const node_g& n1, const node_g& n2)
{
  return !(n1 == n2);
}

struct node_g_ptrs_cmp
{
    bool operator()(const node_g* n1, const node_g* n2) const
    {
        return n1 -> cost < n2 -> cost;
    }
};

std::ostream& operator<<(std::ostream& os, const node_g& n)
{
    os << "cost: " << n.cost << " - (" << n.point.x << ", " << n.point.y << "). Parent " << n.parent;
    return os;
}
