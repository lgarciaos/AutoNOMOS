#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// ros messages
#include <geometry_msgs/Point.h>

#define ARE_NODES_PTR_EQUAL(A, B) (A -> point.x == B -> point.x & A -> point.y == B -> point.y)

class node_g
{
  public:
    double cost;
    geometry_msgs::Point point;
    node_g *parent;

    void print_node_g(std::string s)
    {
      std::cout << "---" << s << ": cost = " << cost << " - (" << point.x << ", " << point.y << "). Parent " << parent << std::endl;

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
