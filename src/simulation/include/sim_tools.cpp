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
#include "priority_queue_nodes.cpp"


#define RADIUS 0.8
#define GRID "GRID"
#define CTRL_6 "CTRL_6"
#define RRT "RRT"
#define RECTANGLE 0
#define CIRCLE 1
#define CAR_SIZE_X 0.2
#define CAR_SIZE_Y 0.125
#define ITERATIONS_LIMIT 10000
#define ARE_POINTS_EQUAL(A, B) (A.x == B.x & A.y == B.y)

// #define POS_X_BOUND +10.0
// #define NEG_X_BOUND -01.0
// #define POS_Y_BOUND +07.0
// #define NEG_Y_BOUND -07.0
const double tol_dist = 0.1;
const int grid_init_x = 10;
const int grid_end_x = -1;
const int grid_init_y = 7;
const int grid_end_y = -7;

int markers_id = 0;
std::string autonomos_static_regex("AutoNOMOS_mini_static[_]*\\d*");
std::string lamp_post_regex("lamp_post_autonomos[_]*\\d*");
std::vector<geometry_msgs::Pose> autonomos_pose;
std::vector<geometry_msgs::Twist> autonomos_twist;
std::vector<geometry_msgs::Pose> lamp_pose;
std::vector<geometry_msgs::Twist> lamp_twist;
std::vector<geometry_msgs::Point> points_without_obs;


ignition::msgs::Marker markerMsg;
ignition::transport::Node node;
ignition::msgs::Material *matMsg;

void print_point(geometry_msgs::Point p);
void print_vector(std::vector<geometry_msgs::Point> p);

void get_obstacles_poses(gazebo_msgs::ModelStates msg)
{
  // std::cout << __PRETTY_FUNCTION__ << '\n';
  std::vector<int> obstacles_index;
  int i = 0;
  autonomos_pose.clear();
  autonomos_twist.clear();
  lamp_pose.clear();
  lamp_twist.clear();
  for (auto element : msg.name)
  {
    // std::cout << "model: " << element << '\n';
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
  // double x_dist = 0.125;
  // double y_dist = 0.2;

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

  if (-CAR_SIZE_X <= x_point_tf && x_point_tf <= CAR_SIZE_X)
  {
    x_match = true;
  }
  if (-CAR_SIZE_Y <= y_point_tf && y_point_tf <= CAR_SIZE_Y )
  {
    y_match = true;
  }

  if (print)
  {
    std::cout <<
      "( " << point.x << ", " << point.y << " )\t\t" <<
      "( " << center_car.position.x << ", " << center_car.position.y << " )\t\t" <<
      "( " << x_point_tf << ", " << y_point_tf << " )" << '\n';
  }

  return x_match & y_match;
}

void set_points_without_obstacles(std::vector<geometry_msgs::Point> points_w_obs)
{
  points_without_obs = points_w_obs;
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
    }
    else
    {
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
  // std::cout << "printing point:";
  // print_point(p);
  // std::cout << std::endl;

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
  ROS_FATAL_STREAM("NOT IMPLEMENTED YET!");
  return -1;
}

void print_point(geometry_msgs::Point p)
{
  printf("(%.1f, %.1f)", p.x, p.y);
}

void print_vector(std::vector<node_g*> vec)
{
  for (auto e : vec) {
    print_point(e->point);
    std::cout << ", ";
  }
  std::cout << '\n';
}

bool path_intersects_obstacle(geometry_msgs::Point start, double roll_start,
                              geometry_msgs::Point end,   double roll_end,
                              geometry_msgs::Pose obstacle, int obstacle_type)
{
  double s, t, d;
  double x00, x01, x10, x11;
  double y00, y01, y10, y11;
  double x_aux, y_aux;
  std::vector<double> x_obs_coords;
  std::vector<double> y_obs_coords;
  std::vector<double> x_sta_coords;
  std::vector<double> y_sta_coords;
  std::vector<double> x_end_coords;
  std::vector<double> y_end_coords;

  bool res = false;

  // u0 = (x00, y00)
  // u1 = (x10, y10)
  // v0 = (x01, y01)
  // v1 = (x11, y11)

  // elements that would form a parametric line between the start and end points

  // elements that would form a parametric line between two points of the obstacles_pos

  if (obstacle_type == RECTANGLE)
  {
    // x00 = start.x;
    // y00 = start.y;
    // x01 = (start.x - end.x);
    // y01 = (start.y - end.y);

    x_obs_coords.push_back(obstacle.position.x + CAR_SIZE_X);
    x_obs_coords.push_back(obstacle.position.x + CAR_SIZE_X);
    x_obs_coords.push_back(obstacle.position.x - CAR_SIZE_X);
    x_obs_coords.push_back(obstacle.position.x - CAR_SIZE_X);

    y_obs_coords.push_back(obstacle.position.y + CAR_SIZE_Y);
    y_obs_coords.push_back(obstacle.position.y - CAR_SIZE_Y);
    y_obs_coords.push_back(obstacle.position.y + CAR_SIZE_Y);
    y_obs_coords.push_back(obstacle.position.y - CAR_SIZE_Y);

    x_sta_coords.push_back((+CAR_SIZE_X) * cos(roll_start) - (+CAR_SIZE_Y) * sin(roll_start) + start.x );
    x_sta_coords.push_back((+CAR_SIZE_X) * cos(roll_start) - (-CAR_SIZE_Y) * sin(roll_start) + start.x );
    x_sta_coords.push_back((-CAR_SIZE_X) * cos(roll_start) - (+CAR_SIZE_Y) * sin(roll_start) + start.x );
    x_sta_coords.push_back((-CAR_SIZE_X) * cos(roll_start) - (-CAR_SIZE_Y) * sin(roll_start) + start.x );

    y_sta_coords.push_back((+CAR_SIZE_Y) * cos(roll_start) + (+ CAR_SIZE_X) * sin(roll_start) + start.y);
    y_sta_coords.push_back((-CAR_SIZE_Y) * cos(roll_start) + (+ CAR_SIZE_X) * sin(roll_start) + start.y);
    y_sta_coords.push_back((+CAR_SIZE_Y) * cos(roll_start) + (- CAR_SIZE_X) * sin(roll_start) + start.y);
    y_sta_coords.push_back((-CAR_SIZE_Y) * cos(roll_start) + (- CAR_SIZE_X) * sin(roll_start) + start.y);

    x_end_coords.push_back((+ CAR_SIZE_X) * cos(roll_end) - (+ CAR_SIZE_Y) * sin(roll_end) + end.x);
    x_end_coords.push_back((+ CAR_SIZE_X) * cos(roll_end) - (- CAR_SIZE_Y) * sin(roll_end) + end.x);
    x_end_coords.push_back((- CAR_SIZE_X) * cos(roll_end) - (+ CAR_SIZE_Y) * sin(roll_end) + end.x);
    x_end_coords.push_back((- CAR_SIZE_X) * cos(roll_end) - (- CAR_SIZE_Y) * sin(roll_end) + end.x);

    y_end_coords.push_back((+ CAR_SIZE_Y) * cos(roll_end) + (+ CAR_SIZE_X) * sin(roll_end) + end.y);
    y_end_coords.push_back((- CAR_SIZE_Y) * cos(roll_end) + (+ CAR_SIZE_X) * sin(roll_end) + end.y);
    y_end_coords.push_back((+ CAR_SIZE_Y) * cos(roll_end) + (- CAR_SIZE_X) * sin(roll_end) + end.y);
    y_end_coords.push_back((- CAR_SIZE_Y) * cos(roll_end) + (- CAR_SIZE_X) * sin(roll_end) + end.y);


    for (size_t j = 0; j < 4; j++)
    {
      // x00 = start.x * cos(roll_start) - start.y * sin(roll_start);
      // y00 = start.y * cos(roll_start) + start.x * sin(roll_start);
      // x_aux = end.x * cos(roll_end) - end.y * sin(roll_end);
      // y_aux = end.y * cos(roll_end) + end.x * sin(roll_end);

      x00 = x_sta_coords[j];
      y00 = y_sta_coords[j];
      x01 = x_end_coords[j] - x00;
      y01 = y_end_coords[j] - y00;

      for (size_t i = 0; i < 4; i++)
      {
        // printf("Init_PT\t\tEnd_PT\t\tOBS_1\t\tOBST_2\n");
        x10 = x_obs_coords[i];
        y10 = y_obs_coords[i];
        x11 = x_obs_coords[((i + 1) % 4)] - x_obs_coords[i];
        y11 = y_obs_coords[((i + 1) % 4)] - y_obs_coords[i];
        d = x11 * y01 - x01 * y11;
        if (d == 0)
        {
          // lines are parallel
        }
        else
        {
          s = (1/d) *  ( (x00 - x10) * y01 - (y00 - y10) * x01);
          t = (1/d) * -(-(x00 - x10) * y11 + (y00 - y10) * x11);
          if (0 <= s && s <= 1 && 0 <= t && t <= 1 )
          {
            //   x00    y00     x01    y01      x10    y10      x11    y11
            // printf("s = %.1f, t = %.1f, d = %.1f, roll_start = %.1f, roll_end = %.1f\n", s, t, d, roll_start, roll_end);
            // printf("sta (%.2f, %.2f):\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\n", start.x, start.y, x_sta_coords[0], y_sta_coords[0], x_sta_coords[1], y_sta_coords[1], x_sta_coords[2], y_sta_coords[2], x_sta_coords[3], y_sta_coords[3]);
            // printf("end (%.2f, %.2f):\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\n", end.x, end.y, x_end_coords[0], y_end_coords[0], x_end_coords[1], y_end_coords[1], x_end_coords[2], y_end_coords[2], x_end_coords[3], y_end_coords[3]);
            // printf("obs (%.2f, %.2f):\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\n", obstacle.position.x, obstacle.position.y, x_obs_coords[0], y_obs_coords[0], x_obs_coords[1], y_obs_coords[1], x_obs_coords[2], y_obs_coords[2], x_obs_coords[3], y_obs_coords[3]);
            // printf("x(t) = %.2f + %.2ft\ty(t) = %.2f + %.2ft\n", x00, x01, y00, y01);
            // printf("x(t) = %.1f + %.1ft\ty(t) = %.1f + %.1ft\n", x_sta_coords[0], x_sta_coords[0] - x_end_coords[0], y_sta_coords[0], y_sta_coords[0] - y_end_coords[0]);
            // printf("x(t) = %.1f + %.1ft\ty(t) = %.1f + %.1ft\n", x_sta_coords[1], x_sta_coords[1] - x_end_coords[1], y_sta_coords[1], y_sta_coords[1] - y_end_coords[1]);
            // printf("x(t) = %.1f + %.1ft\ty(t) = %.1f + %.1ft\n", x_sta_coords[2], x_sta_coords[2] - x_end_coords[2], y_sta_coords[2], y_sta_coords[2] - y_end_coords[2]);
            // printf("x(t) = %.1f + %.1ft\ty(t) = %.1f + %.1ft\n", x_sta_coords[3], x_sta_coords[3] - x_end_coords[3], y_sta_coords[3], y_sta_coords[3] - y_end_coords[3]);
            // printf("(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f). s = %.1f, t = %.1f, d = %.1f\n", start.x, start.y, end.x, end.y, obstacle.position.x, obstacle.position.y, s, t, d);
            res = true;
          }
        }
      }
    }
  }

  return res;

}

double get_distance(int vel)
{
  double adj_fac = -15.0 / 31.0 * 1.0 / 12.0;
  double rad = .03;
  double time_s = 1;
  // std::cout << "vel: " << vel << ", adj_fac: " << adj_fac << ", rad: " << rad << ", ret: " << vel * rad * adj_fac * time_s << '\n';
  return vel * rad * adj_fac * time_s;
}

bool is_collision_free(geometry_msgs::Point start_pt, double roll_start, geometry_msgs::Point end_pt, double roll_end)
{
  // geometry_msgs::Point point;
  // point.x = x_point_tf;
  // point.y = y_point_tf;
  int aut_count = 0;
  // std::cout << "CAR \t\t POINT" << '\n';
  // printf("(x00, y00)\t(x01, y01)\t(x10, y10)\t(x11, y11).\n");

  for(auto pose : autonomos_pose)
  {
    // std::cout << "(" << pose.position.x << ", " << pose.position.y << ")\t(" << end_pt.x << ", " << end_pt.y << ")" << std::endl;
    if (point_in_car(end_pt, pose, false))
    {
      return false;
    //   points_in_obstacles.push_back(point_num);
    }
    if (path_intersects_obstacle(start_pt, roll_start, end_pt, roll_end, pose, RECTANGLE))
    {
      return false;
    }
    aut_count++;
  }
  return true;
}

std::vector<node_g*> get_adj_points(geometry_msgs::Point pt_ini, double roll,
                                    std::vector<node_g*> nodes, std::string points_creation)
{
  std::vector<node_g*> out;

  if (points_creation == GRID)
  {
    for(auto n : nodes)
    {
      if (distance(pt_ini, n -> point) < RADIUS && !ARE_POINTS_EQUAL(pt_ini, n -> point) )
      {
        out.push_back(n);
      }
    }
  }
  else if (points_creation == CTRL_6)
  {
    std::vector<double> steering_vec = {-.5, 0, .5};
    std::vector<int> speed_vec = {-100, -150, -200};
    for (auto steering : steering_vec)
    {
      double angle = roll + steering;
      for (auto speed : speed_vec )
      {
        double dist = get_distance(speed);
        double x_point_tf = pt_ini.x + dist * sin(angle);
        double y_point_tf = pt_ini.y + dist * cos(angle);
        geometry_msgs::Point end_pt;
        end_pt.x = x_point_tf;
        end_pt.y = y_point_tf;
        // printf("Dist: %.2f. Start: (%.2f, %.2f). End: (%.2f, %.2f)\n", dist, pt_ini.x, pt_ini.y, x_point_tf, y_point_tf);
        if(is_collision_free(pt_ini, roll, end_pt, angle))
        {
          node_g* n;
          n = (node_g*) malloc(sizeof(node_g));
          n -> point.x = end_pt.x;
          n -> point.y = end_pt.y;
          n -> set_orientation(angle, 0, 0);
          n -> parent = NULL;
          // n -> print_node_g("FREE");
          out.push_back(n);
        }

      }
    }
  }
  // TODO: determine if the point can be reach by the car.
  return out;
}

bool point_reached(geometry_msgs::Point curr_point, geometry_msgs::Point end_point)
{
  return distance(curr_point, end_point) <= tol_dist;
}

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

bool is_element_in_vector(std::vector<node_g*> vector, node_g* element)
{
  for(auto e : vector)
  {
    double dx = fabs(e -> point.x - element -> point.x);
    double dy = fabs(e -> point.y - element -> point.y);
    double dc = fabs(e -> cost    - element -> cost);
    // if (e == element) {
    if (dx <= TOLERANCE_RAD && dy <= TOLERANCE_RAD && dc <= TOLERANCE_COST)
    {
      // assert(e -> point.x == element -> point.x && e -> point.y == element -> point.y);
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
       ignition::math::Vector3d(pt.x, pt.y, .01));
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
  // std::cout << __LINE__ << '\n';

  while (n->parent != NULL)
  {
    vec.push_back(n -> point);
    // draw_line(n -> point, last_pt);

    last_pt = n -> point;
    n = n->parent;
  }
  draw_lines(vec);
}

void draw_line(geometry_msgs::Point ini_pt, geometry_msgs::Point end_pt, std::string color)
{
  // std::cout << __PRETTY_FUNCTION__ << '\n';
  // ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ":" << __LINE__);
  ignition::msgs::Marker markerMsg_2;
  ignition::msgs::Material *matMsg ;//= markerMsg_2.mutable_material();
  matMsg = (ignition::msgs::Material*) malloc(sizeof(ignition::msgs::Material*));
  matMsg = markerMsg_2.mutable_material();

  matMsg->mutable_script()->set_name(color);
  markerMsg_2.set_id(markers_id);
  markers_id++;
  markerMsg_2.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg_2.set_type(ignition::msgs::Marker::LINE_LIST);


  for (double i = -10; i < 0; i+=0.5)
  {
    ignition::msgs::Set(markerMsg_2.add_point(),
      ignition::math::Vector3d(i, i, 0));
    ignition::msgs::Set(markerMsg_2.add_point(),
      ignition::math::Vector3d(i+.5, i+.5, 0));
  }
  // ignition::msgs::Set(markerMsg_2.add_point(),
    // ignition::math::Vector3d(ini_pt.x, ini_pt.y, ini_pt.z));
  // ignition::msgs::Set(markerMsg_2.add_point(),
    // ignition::math::Vector3d(end_pt.x, end_pt.y, end_pt.z));


  node.Request("/marker", markerMsg_2);
}

void draw_all_nodes(std::vector<node_g*> open, std::vector<node_g*> closed)
{
  ignition::msgs::Marker markerMsg_2;
  ignition::msgs::Material *matMsg ;//= markerMsg_2.mutable_material();
  matMsg = (ignition::msgs::Material*) malloc(sizeof(ignition::msgs::Material*));
  matMsg = markerMsg_2.mutable_material();

  matMsg->mutable_script()->set_name("Gazebo/Purple");
  markerMsg_2.set_id(markers_id);
  markers_id++;
  markerMsg_2.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg_2.set_type(ignition::msgs::Marker::LINE_LIST);

  for (auto n : open)
  {
    // n->print_node_g(std::string("DRAW"));
    // draw_line(n -> point, n -> parent -> point, std::string("Gazebo/Purple"));
    ignition::msgs::Set(markerMsg_2.add_point(),
      ignition::math::Vector3d(n -> point.x, n -> point.y, 0));
    ignition::msgs::Set(markerMsg_2.add_point(),
      ignition::math::Vector3d(n -> parent -> point.x, n -> parent -> point.y, 0));
  }
  node.Request("/marker", markerMsg_2);
  ignition::msgs::Marker markerMsg_3;
  // ignition::msgs::Material *matMsg ;//= markerMsg_2.mutable_material();
  // matMsg = (ignition::msgs::Material*) malloc(sizeof(ignition::msgs::Material*));
  // matMsg = markerMsg_3.mutable_material();

  matMsg->mutable_script()->set_name("Gazebo/White");
  markerMsg_3.set_id(markers_id);
  markers_id++;
  markerMsg_3.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg_3.set_type(ignition::msgs::Marker::LINE_LIST);
  for (auto n : closed)
  {
    if (n -> parent != NULL)
    {
      ignition::msgs::Set(markerMsg_3.add_point(),
        ignition::math::Vector3d(n -> point.x, n -> point.y, 0));
      ignition::msgs::Set(markerMsg_3.add_point(),
        ignition::math::Vector3d(n -> parent -> point.x, n -> parent -> point.y, 0));
      // draw_line(n -> point, n -> parent -> point, std::string("Gazebo/FlatBlack"));
    }
  }
  node.Request("/marker", markerMsg_3);

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

void a_star(geometry_msgs::Point init_point, geometry_msgs::Point fin_point, std::string points_creation)
{
  priority_queue_nodes open;
  std::vector<node_g*> closed;
  std::vector<node_g*> neighbors;
  std::vector<node_g*> nodes;
  node_g* current;
  current = (node_g*) malloc(sizeof(node_g));
  double cost, neig_actual_cost;
  bool neig_in_open = false;
  bool neig_in_closed = false;
  int iterations = 0;

  double dummy;

  current -> cost = 0;
  current -> point.x = init_point.x;
  current -> point.y = init_point.y;
  current -> set_orientation(0, 0, 0);
  current -> parent = NULL;
  open.push(current);

  nodes = init_nodes_g(points_without_obs);
  while (!point_reached(open.top() -> point, fin_point))
  {
    iterations++;
    current = open.top();
    open.pop();
    closed.push_back(current);
    neighbors = get_adj_points(current -> point, current -> get_roll(), nodes, points_creation);

    // std::cin >> dummy;

    for(auto neig : neighbors)
    {
      neig_actual_cost = actual_cost(current, neig);
      cost = current -> cost ;
      neig_in_open = open.node_in_queue(neig);
      neig_in_closed = is_element_in_vector(closed, neig);

      if (neig_in_open && cost < neig_actual_cost )
      {
        open.remove_node_from_queue(neig);
      }
      else if (!neig_in_open && !neig_in_closed)
      {
        if (current -> parent != neig)
        {
          neig -> parent = current;
          neig -> cost = neig_actual_cost + h(neig, fin_point);
          open.push(neig);
        }
        else
        {
          std::cout << "problem..." << '\n';
          current -> print_node_g("CURRENT");
          neig -> print_node_g("NEIG");
        }
      }
    }

      if (iterations % ITERATIONS_LIMIT == 0 )
      {
        std::cout << "iterations: " << iterations << "... continue?" << '\n';
        std::cin >> dummy;
        if (dummy == 0)
        {
          break;
        }
        else
        {
          delete_markers();
          paint_points(points_without_obs);
          paint_point(init_point, 1);
          paint_point(fin_point, 2);
          draw_all_nodes(open.get_vector(), closed);

          draw_path(open.top());
        }
      }
  }
  delete_markers();
  paint_points(points_without_obs);
  paint_point(init_point, 1);
  paint_point(fin_point, 2);
  draw_all_nodes(open.get_vector(), closed);

  draw_path(open.top());

}
