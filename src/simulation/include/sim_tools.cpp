// std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include <regex>
#include <queue>
#include <cassert>

// ros
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ros messages
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>

// ignition
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>

// own libraries
#include "node_g.cpp"
#include "priority_queue_nodes.cpp"

#define RADIUS 0.8
#define ARE_POINTS_EQUAL(A, B) (A.x == B.x & A.y == B.y)

const double tol_dist = 0.1;
const int grid_init_x = -10;
const int grid_end_x = 0;
const int grid_init_y = -10;
const int grid_end_y = 10;

int markers_id = 0;
std::string autonomos_static_regex("AutoNOMOS_mini_static_[[:digit:]]+");
std::string lamp_post_regex("lamp_post_autonomos_[[:digit:]]+");
std::vector<geometry_msgs::Pose> autonomos_pose;
std::vector<geometry_msgs::Twist> autonomos_twist;
std::vector<geometry_msgs::Pose> lamp_pose;
std::vector<geometry_msgs::Twist> lamp_twist;

ignition::msgs::Marker markerMsg;
ignition::transport::Node node;
ignition::msgs::Material *matMsg;

///////////////////////////////////////////////////////////////////////////////
// TODO:
//       det if point is in car ==> trasformation
//////////////////////////////////////////////////////////////////////////////
void print_point(geometry_msgs::Point p);
void print_vector(std::vector<geometry_msgs::Point> p);

void get_obstacles_poses(gazebo_msgs::ModelStates msg)
{
  std::vector<int> obstacles_index;
  // ROS_INFO_STREAM("At: " << __PRETTY_FUNCTION__);
  int i = 0;
  autonomos_pose.clear();
  autonomos_twist.clear();
  lamp_pose.clear();
  lamp_twist.clear();
  for (auto element : msg.name) {
    // if (!element.compare(autonomos_static)) {
    if (std::regex_match (element, std::regex(autonomos_static_regex) ))
    {
      autonomos_pose.push_back(msg.pose[i]);
      autonomos_twist.push_back(msg.twist[i]);
    } else if(std::regex_match (element, std::regex(lamp_post_regex)) )
    {
      lamp_pose.push_back(msg.pose[i]);
      lamp_twist.push_back(msg.twist[i]);
    }
    i++;
    // ROS_INFO_STREAM(element);
  }

  matMsg = markerMsg.mutable_material();
}

bool point_in_car(geometry_msgs::Point point, geometry_msgs::Pose center_car, bool print = false)
{
  double x_dist = 0.125;
  double y_dist = 0.2;

  bool x_match = false;
  bool y_match = false;

  ignition::math::Quaterniond orient (center_car.orientation.x,
                                      center_car.orientation.y,
                                      center_car.orientation.z,
                                      center_car.orientation.w);
  // std::cout << "Pose: " << orient.Yaw() << ", " <<
  //   orient.Pitch() << ", " <<
  //   orient.Roll() << ", " <<
    // center_car.orientation.w <<
     // '\n';
  // std::cout << "twist: " << center_car.angular.x << ", " << center_car.angular.y << ", " << center_car.angular.z << '\n';
  double x_point_tf = point.x - center_car.position.x;
  double y_point_tf = point.y - center_car.position.y;
  double angle = orient.Roll();

  x_point_tf = x_point_tf * cos(angle) - y_point_tf * sin(angle);
  y_point_tf = y_point_tf * cos(angle) + x_point_tf * sin(angle);

  if (-x_dist <= x_point_tf && x_point_tf <= x_dist)
  {
    x_match = true;
  }
  if (-y_dist <= y_point_tf && y_point_tf <= y_dist )
  {
    y_match = true;
  }

  // if (print)
  // {
  //   std::cout <<
  //     "( " << point.x << ", " << point.y << " )\t\t" <<
  //     "( " << center_car.position.x << ", " << center_car.position.y << " )\t\t" <<
  //     "( " << x_point_tf << ", " << y_point_tf << " )" << '\n';
  // }

  return x_match & y_match;
}


std::vector<geometry_msgs::Point> remove_obst_points(std::vector<geometry_msgs::Point> points_in)
{
  std::vector<geometry_msgs::Point> points_out;
  std::vector<int> points_in_obstacles;
  bool obst_found = false;
  int car_num = 0;
  int point_num = 0;
  for(auto point : points_in)
  {
    car_num = 0;
    for(auto pose : autonomos_pose)
    {
      if (point_in_car(point, pose))
      {
        points_in_obstacles.push_back(point_num);
      }
      // if( car_num == 67)
      // {
      //   std::cout << "is 67 in car? " << point_in_car(point, pose, true) << std::endl;
      // }
      car_num++;
    }
    point_num++;
  }

  std::unique(points_in_obstacles.begin(), points_in_obstacles.end());

  // for ( auto p : points_in_obstacles ) {
  //   std::cout << "index: " << p <<
  //     "\t( " << points_in[p].x << ", " << points_in[p].y << " )" << std::endl;
  // }

  int index_point;
  if(points_in_obstacles.size() > 0)
  {
    index_point = points_in_obstacles.front();
    points_in_obstacles.erase(points_in_obstacles.begin());
  }
  for (size_t i = 0; i < points_in.size(); i++) {
    // std::cout << "i: " << i << "\tindex_point: " << index_point << '\n';
    if (i == index_point)
    {
      if(points_in_obstacles.size() > 0)
      {
        index_point = points_in_obstacles.front();
        points_in_obstacles.erase(points_in_obstacles.begin());
      } else {
        index_point = -1;
      }
    } else {
      points_out.push_back(points_in[i]);
    }
  }

  // for ( auto p : points_out ) {
  //   std::cout << "( " << p.x << ", " << p.y << " )" << '\n';
  // }

  return points_out;
}


std::vector<geometry_msgs::Point> generate_grid(double x_inc, double y_inc)
{
  std::vector<geometry_msgs::Point> v;
  geometry_msgs::Point aux_pt;
  for (double x = grid_init_x; x < grid_end_x; x+=x_inc) {
    for (double y = grid_init_y; y < grid_end_y; y+=y_inc) {
      aux_pt.x = x;
      aux_pt.y = y;
      aux_pt.z = 0;

      v.push_back(aux_pt);
    }
  }
  return v;
}
void delete_markers()
{
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  markers_id = 0;
  node.Request("/marker", markerMsg);
}
void paint_points(std::vector<geometry_msgs::Point> points)
{
  // std::cout << "At paint_points" << '\n';
  matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/Green");

  markerMsg.set_id(markers_id);
  markers_id++;
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::POINTS);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_scale(),
                      ignition::math::Vector3d(1, 1, 1));
  for (auto element : points)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(element.x, element.y, element.z));
  }
  node.Request("/marker", markerMsg);
}

void paint_point(geometry_msgs::Point p, int id)
{
  ignition::msgs::Marker markerMsg_2;
  ignition::msgs::Material *matMsg = markerMsg_2.mutable_material();
  std::cout << "printing point:";
  print_point(p);
  std::cout << std::endl;

  matMsg->mutable_script()->set_name("Gazebo/Red");
  // markerMsg.set_ns("default");
  markerMsg_2.set_id(markers_id);
  markers_id++;
  markerMsg_2.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg_2.set_type(ignition::msgs::Marker::SPHERE);
  // markerMsg.clear_point();
  ignition::msgs::Set(markerMsg_2.mutable_scale(),
                      ignition::math::Vector3d(0.2, 0.2, 0.2));
  ignition::msgs::Set(markerMsg_2.mutable_pose(),
    ignition::math::Pose3d(p.x, p.y, p.z, 0, 0, 0));

  node.Request("/marker", markerMsg_2);
}

double distance(geometry_msgs::Point ini, geometry_msgs::Point p)
{
  return sqrt( (p.x - ini.x) * (p.x - ini.x) + (p.y - ini.y) * (p.y - ini.y) );
}

double h(node_g* n, geometry_msgs::Point goal)
{
  return distance(n -> point, goal);
}

double actual_cost(node_g *last, node_g *next)
{
  return distance(last -> point, next -> point) + next -> cost;
}

double movement_cost(node_g* current, node_g* neighbor)
{

}

void print_point(geometry_msgs::Point p)
{
  printf("(%.1f, %.1f)", p.x, p.y);
}


// void print_queue(std::priority_queue<node_g*, std::vector<node_g*>, std::greater<node_g*>> q) {
//     while(!q.empty()) {
//         std::cout << q.top() ->cost << " - ";
//         print_point(q.top() -> point);
//         std::cout << std::endl;
//         q.pop();
//     }
//     std::cout << '\n';
// }

void print_vector(std::vector<node_g*> vec)
{
  // std::cout << __PRETTY_FUNCTION__ << '\n';
  for (auto e : vec) {
    print_point(e->point);
    std::cout << ", ";
  }
  std::cout << '\n';
}

std::vector<node_g*> get_adj_points(geometry_msgs::Point pt_ini, std::vector<node_g*> nodes)
{
  std::vector<node_g*> out;
  for(auto n : nodes)
  {
    // if (distance(pt_ini, pt) < RADIUS && !(pt_ini.x == pt.x && pt_ini.y == pt.y) ) // TODO: change != to macro??
    if (distance(pt_ini, n -> point) < RADIUS && !ARE_POINTS_EQUAL(pt_ini, n -> point) ) // TODO: change != to macro??
    {
      out.push_back(n);
    }
  }
  // TODO: determine if the point can be reach by the car.
  return out;
}

bool point_reached(geometry_msgs::Point curr_point, geometry_msgs::Point end_point)
{
  return distance(curr_point, end_point) <= tol_dist;
}

// std::priority_queue<node_g*, std::vector<node_g*>, std::greater<node_g*>> remove_node_from_open(
//   std::priority_queue<node_g*, std::vector<node_g*>, std::greater<node_g*>> open, node_g* node)

// {
//   std::priority_queue<node_g*, std::vector<node_g*>, std::greater<node_g*>> res;
//   while(0 < open.size())
//   {
//     node_g* n = open.top();
//     if ( n == node)
//     {
//       // open.pop();
//     }
//     else
//     {
//       res.push(n);
//     }
//     open.pop();
//   }
//   return res;
// }

std::vector<node_g*> remove_node_from_closed(std::vector<node_g*> closed, node_g *node)
{
  std::vector<node_g*> res;
  for(auto n : closed)
  {
    if ( !( n == node ))
    {
       res.push_back(n);
    }
  }
  return res;
}

// bool is_element_in_queue(std::priority_queue<node_g*, std::vector<node_g*>, std::greater<node_g*> > queue, node_g* element)
// {
//   while (0 < queue.size())
//   {
//     node_g *n = queue.top();
//     queue.pop();
//     if ( n -> point.x == element -> point.x && n -> point.y == element -> point.y )
//     {
//       return true;
//     }
//   }
//   return false;
// }

bool is_element_in_vector(std::vector<node_g*> vector, node_g* element)
{
  for(auto e : vector)
  {
    if (e == element) {
      assert(e -> point.x == element -> point.x && e -> point.y == element -> point.y);
      return true;
    }
  }
  return false;
}

void print_path(node_g* end_node)
{
  // std::cout << __PRETTY_FUNCTION__ << '\n';
  node_g *n = end_node;
  int cont_end = 10, cont = 0, continue_var = 1;
  while (n->parent != NULL)
  {
    std::cout << "cost:" << n -> cost;
    // std::cout << " parent: " << n -> parent;
    print_point(n->point);
    std::cout << " <== ";
    n = n->parent;
    if (cont == cont_end)
    {
      std::cout << "continue?" << '\n';
      std::cin >> continue_var;
      if (continue_var)
      {
        continue_var = 0;
        cont = 0;
      }
      else
      {
        break;
      }
    }
    cont++;
  }
  std::cout << "cost:" << n -> cost << " parent: " << n -> parent;
  print_point(n->point);
  std::cout << " <== ";
  std::cout << "NULL" << '\n';
}

void draw_lines(std::vector<geometry_msgs::Point> vec)
{
  std::cout << __PRETTY_FUNCTION__ << '\n';
  ignition::msgs::Marker markerMsg_2;
  ignition::msgs::Material *matMsg = markerMsg_2.mutable_material();

  matMsg->mutable_script()->set_name("Gazebo/Blue");
  markerMsg_2.set_id(markers_id);
  markers_id++;
  markerMsg_2.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg_2.set_type(ignition::msgs::Marker::LINE_STRIP);
  // ignition::msgs::Set(markerMsg_2.add_point(),
  //     ignition::math::Vector3d(pt_from.x, pt_from.y, pt_from.z));
  // ignition::msgs::Set(markerMsg_2.add_point(),
  //     ignition::math::Vector3d(pt_to.x, pt_to.y, pt_to.z));

  for (auto pt : vec)
  {
    ignition::msgs::Set(markerMsg_2.add_point(),
       ignition::math::Vector3d(pt.x, pt.y, pt.z));
  }


  node.Request("/marker", markerMsg_2);
}

void draw_path(node_g* end_node)
{
  node_g *n = end_node;
  int cont_end = 10, cont = 0;
  geometry_msgs::Point last_pt;
  std::vector<geometry_msgs::Point> vec;
  // last_pt = n -> point;
  // n = n -> parent;
  std::cout << __LINE__ << '\n';

  while (n->parent != NULL)
  {
    vec.push_back(n -> point);
    // draw_line(n -> point, last_pt);

    last_pt = n -> point;
    n = n->parent;
  }
  std::cout << __LINE__ << '\n';
  draw_lines(vec);
}

std::vector<node_g*> init_nodes_g(std::vector<geometry_msgs::Point> points)
{
  std::vector<node_g*> res;
  for (auto pt : points)
  {
    node_g *n;
    n = (node_g*) malloc(sizeof(node_g));
    n -> cost = 0;
    n -> point = pt;
    n -> parent = NULL;
    res.push_back(n);
  }
  return res;
}

void a_star(std::vector<geometry_msgs::Point> points, geometry_msgs::Point init_point, geometry_msgs::Point fin_point)
{
  // std::cout << __PRETTY_FUNCTION__ << '\n';
  // std::priority_queue<node_g*, std::vector<node_g*>, std::greater<node_g*> > open;
  priority_queue_nodes open;
  std::vector<node_g*> closed;
  std::vector<node_g*> neighbors;
  std::vector<node_g*> nodes;
  node_g* current;
  current = (node_g*) malloc(sizeof(node_g));
  double cost, neig_actual_cost;
  bool neig_in_open = false;
  bool neig_in_closed = false;

  double dummy;
  std::vector<int> v_dummy = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  /////////////////////////////////////////////////////////////////////////////
  //  TODO: check that priority_queue is working well
  /////////////////////////////////////////////////////////////////////////////
  // srand(231192);
  // for (size_t i = 0; i < 10; i++) {
  //   node_g* aux;
  //   aux = (node_g*) malloc(sizeof(node_g));
  //   int i_dummy = rand() % v_dummy.size();
  //   aux -> cost = v_dummy[i_dummy];
  //   v_dummy.erase(v_dummy.begin() + i_dummy);
  //   aux -> point.x = 10 - i;
  //   aux -> point.y = 10 - i;
  //   aux -> parent = NULL;
  //   open.push(aux);
  // }
  // open.print_vector();
  // std::cin >> dummy;

  current -> cost = 0;
  current -> point.x = init_point.x;
  current -> point.y = init_point.y;
  current -> parent = NULL;
  open.push(current);

  nodes = init_nodes_g(points);
  // std::cout << __LINE__ << '\n';
  while (!point_reached(open.top() -> point, fin_point)) {
      current = open.top();
      open.pop();
      closed.push_back(current);
      neighbors = get_adj_points(current -> point, nodes);
      std::cout << "current point: ";
      print_point(current -> point);
      std::cout << '\n';
      std::cout << "Neighbors: ";
      print_vector(neighbors);
      std::cout << '\n';
      // draw_line(last);
      std::cout << "Current: " << current << '\n';
      // std::cin >> dummy;

      for(auto neig : neighbors)
      {
        // std::cout << __LINE__ << '\n';
        neig_actual_cost = actual_cost(current, neig);
        cost = current -> cost ; // + movement_cost(current, node_neig); // TODO: implement actual_cost & movementcost
        // neig_in_open = is_element_in_queue(open, neig);
        // std::cout << __LINE__ << '\n';
        neig_in_open = open.node_in_queue(neig);
        neig_in_closed = is_element_in_vector(closed, neig);


        // std::cout << __LINE__ << '\n';
        if (neig_in_open && cost < neig_actual_cost )
        {
          // open = remove_node_from_open(open, neig);// Remove neig from open
          // std::cout << __LINE__ << '\n';
          open.remove_node_from_queue(neig);
        }
        // else if (neig_in_closed && cost < neig_actual_cost )
        // {
        //   // std::cout << __LINE__ << '\n';
        //   closed = remove_node_from_closed(closed, neig);
        // }
        else if (!neig_in_open && !neig_in_closed)
        {
          // if (current -> point.x == -8 && current -> point.y == -5.5)
          // {
          //   open.print_vector();
          //   current -> print_node_g(std::string("current"));
          //   neig -> print_node_g(std::string("neig"));
          // }
          // if (current -> point.x == -8.5 && current -> point.y == -5)
          // {
          //   open.print_vector();
          //   current -> print_node_g(std::string("current"));
          //   neig -> print_node_g(std::string("neig"));
          // }

          if (current -> parent != neig)
          {


            neig -> parent = current;
            neig -> cost = neig_actual_cost + h(neig, fin_point);
            std::cout << "--- neighbor";
            print_point(neig -> point);
            std::cout << "\tcost: " << neig -> cost;
            std::cout << "\tthis:"  << neig;
            std::cout << '\n';
            // std::cout << __LINE__ << '\n';
            open.push(neig);
          }
          else
          {
            std::cout << "problem..." << '\n';
            current -> print_node_g("CURRENT");
            neig -> print_node_g("NEIG");
          }
          // std::cout << __LINE__ << '\n';
          // std::cout << "current: " << 1current << '\n';
          // std::cout << "neighbor: " << node_neig << '\n';
        }
      }
      // std::cout << "-------------   PATH   ------------- " << '\n';
      // print_path(open.top());
      // std::cout << "------------- END PATH -------------" << '\n';
      // delete_markers();
      // paint_points(points);
      // paint_point(init_point, 1);
      // paint_point(fin_point, 2);
      // draw_path(open.top());
  }
  ROS_INFO_STREAM("While ended...");
  delete_markers();
  paint_points(points);
  paint_point(init_point, 1);
  paint_point(fin_point, 2);

  open.print_vector();
  std::cout << __LINE__ << '\n';
  // print_path(open.top());

  draw_path(open.top());
  std::cout << __LINE__ << '\n';

}
