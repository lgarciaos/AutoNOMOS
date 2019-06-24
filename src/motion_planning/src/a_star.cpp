#include "a_star.h"


a_star_t::~a_star_t()
{

}

void a_star_t::setup_planning()
{
  goal = (node_g*) malloc(sizeof(node_g));
  collision_detector = new collision_detector_t();

}

void a_star_t::set_obstacles(std::vector<geometry_msgs::Pose> vec_obstacles_poses,
  std::vector<int> vec_obstacles_type, double in_obstacles_radius)
{
  // collision_detector -> set_obstacles(vec_obstacles_poses, vec_obstacles_type);
  collision_detector -> set_obstacles_radius(in_obstacles_radius);
  // obstacles_poses = vec_obstacles_poses;
  // obstacles_type = vec_obstacles_type;
}

void a_star_t::set_start_state(geometry_msgs::Pose2D in_start)
{
  node_g* current;
  current = (node_g*) malloc(sizeof(node_g));
  current -> cost = 0;
  current -> point.x = in_start.x;
  current -> point.y = in_start.y;
  current -> set_orientation(in_start.theta, 0, 0);
  current -> parent = NULL;
  open.push(current);

}

std::vector<geometry_msgs::Point> a_star_t::generate_grid(double x_inc,
  double y_inc, double grid_init_x, double grid_init_y, double grid_end_x,
  double grid_end_y )
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

void a_star_t::remove_obst_points(
  std::vector<geometry_msgs::Point> points_in)
{
  std::vector<int> points_in_obstacles;
  // bool obst_found = false;
  int car_num = 0;
  int point_num = 0;
  node_g* aux;
  aux = (node_g*) malloc(sizeof(node_g));

  for(auto point : points_in)
  {
    car_num = 0;
    aux -> point = point;
    // printf("aux:            (%.2f, %.2f, 0.00)\n", aux -> point.x, aux -> point.y );
    // printf("aux_get_pose2d: (%.2f, %.2f, %.2f)\n", aux -> get_pose2d().x, aux -> get_pose2d().y, aux -> get_pose2d().theta );
    for(auto pose : obstacles_poses)
    {
      if (collision_detector -> pose_in_car(aux -> get_pose2d(), pose, false))
      {
        points_in_obstacles.push_back(point_num);
      }
      car_num++;
    }
    point_num++;
  }

  std::unique(points_in_obstacles.begin(), points_in_obstacles.end());

  long unsigned int index_point;
  if(points_in_obstacles.size() > 0)
  {
    index_point = points_in_obstacles.front();
    points_in_obstacles.erase(points_in_obstacles.begin());
  }
  for (long unsigned int i = 0; i < points_in.size(); i++) {
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
      grid_points.push_back(points_in[i]);
    }
  }
}

void a_star_t::set_type(std::string in_type)
{
  // std::cout << "type: " << in_type << '\n';
  type = in_type;
}

void a_star_t::set_goal_state(geometry_msgs::Pose2D in_goal, double in_radius)
{
  goal -> cost = 0;
  goal -> point.x = in_goal.x;
  goal -> point.y = in_goal.y;
  goal -> set_orientation(in_goal.theta, 0, 0);
  goal -> parent = NULL;

  goal_radius = in_radius;
}

void a_star_t::step()
{
  std::vector<node_g*> neighbors;
  // node_g* current;
  // current = (node_g*) malloc(sizeof(node_g));
  double cost, neig_actual_cost;
  bool neig_in_open = false;
  bool neig_in_closed = false;

  current = open.top();
  open.pop();
  closed.push_back(current);
  neighbors = get_adj_points(current, nodes_grid, type);

  for(auto neig : neighbors)
  {
    neig_actual_cost = actual_cost(current, neig);
    cost = current -> cost ;
    neig_in_open = open.node_in_queue(neig);
    neig_in_closed = is_element_in_vector(closed, neig);

    // printf("(%.2f, %.2f, %.2f)\tactual_c: %.2f\tcost:%.2f\n", neig -> point.x,
      // neig -> point.y, neig -> get_roll(), neig_actual_cost, neig_actual_cost + h(neig, goal));

    if (neig_in_open && cost < neig_actual_cost )
    {
      open.remove_node_from_queue(neig);
    }
    else if (!neig_in_open && !neig_in_closed)
    {
      neig -> parent = current;
      neig -> cost = neig_actual_cost + h(neig, goal);
      open.push(neig);
    }
  }

}

double a_star_t::distance(node_g* n1, node_g* n2)
{
  double d_x = n1 -> point.x - n2 -> point.x;
  double d_y = n1 -> point.y - n2 -> point.y;
  // double d_theta = n1 -> get_roll() - n2 -> get_roll();
  // return sqrt( d_x * d_x + d_y * d_y + d_theta * d_theta);
  return sqrt( d_x * d_x + d_y * d_y);
}

double a_star_t::distance(node_g* n1, geometry_msgs::Pose2D pose)
{
  node_g* n_aux;
  n_aux = (node_g*) malloc(sizeof(node_g));

  n_aux -> point.x = pose.x;
  n_aux -> point.y = pose.y;
  n_aux -> set_orientation(pose.theta, 0, 0);
  // return sqrt( d_x * d_x + d_y * d_y + d_theta * d_theta);
  // return sqrt( d_x * d_x + d_y * d_y);// + d_theta * d_theta);
  return distance(n1, n_aux);
}

double a_star_t::h(node_g* now, node_g* goal)
{
  return distance(now, goal);
}

double a_star_t::get_distance_driven(int vel)
{
  double adj_fac = -15.0 / 31.0 * 1.0 / 12.0;
  double rad = .03;
  double time_s = 1;
  // std::cout << "vel: " << vel << ", adj_fac: " << adj_fac << ", rad: " << rad << ", ret: " << vel * rad * adj_fac * time_s << '\n';
  return vel * rad * adj_fac * time_s;
}

// bool a_star_t::node_in_car(node_g* node, geometry_msgs::Pose center_car, bool print = false)
// {
//
//   bool x_match = false;
//   bool y_match = false;
//
//   tf::Quaternion orient (center_car.orientation.x,
//                          center_car.orientation.y,
//                          center_car.orientation.z,
//                          center_car.orientation.w);
//
//   tf::Matrix3x3 m(orient);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);
//   double x_point_tf = node -> point.x - center_car.position.x;
//   double y_point_tf = node -> point.y - center_car.position.y;
//   double angle = roll;
//
//   x_point_tf = x_point_tf * cos(angle) - y_point_tf * sin(angle);
//   y_point_tf = y_point_tf * cos(angle) + x_point_tf * sin(angle);
//
//   if (-CAR_SIZE_X <= x_point_tf && x_point_tf <= CAR_SIZE_X)
//   {
//     x_match = true;
//   }
//   if (-CAR_SIZE_Y <= y_point_tf && y_point_tf <= CAR_SIZE_Y )
//   {
//     y_match = true;
//   }
//
//   if (print)
//   {
//     std::cout <<
//       "( " << node -> point.x << ", " << node -> point.y << " )\t\t" <<
//       "( " << center_car.position.x << ", " << center_car.position.y << " )\t\t" <<
//       "( " << x_point_tf << ", " << y_point_tf << " )" << '\n';
//   }
//
//   return x_match & y_match;
// }


// bool a_star_t::is_collision_free(node_g* start_node, node_g* end_node)
// {
//   int aut_count = 0;
//
//   // for(auto pose : obstacles_poses)
//   for (size_t i = 0; i < obstacles_poses.size(); i++)
//   {
//     // if (obstacles_type[i] == RECTANGLE && node_in_car(end_node, obstacles_poses[i], false))
//     if (obstacles_type[i] == RECTANGLE && pose_in_car(end_node, obstacles_poses[i], false))
//     {
//       return false;
//     }
//     if (collision_detector -> path_intersects_obstacle(start_node-> get_pose2d(),
//       end_node -> get_pose2d(), obstacles_poses[i], obstacles_type[i]))
//     {
//       return false;
//     }
//     aut_count++;
//   }
//   return true;
// }



// bool a_star_t::path_intersects_obstacle(node_g* start_node, node_g* end_node,
//                               geometry_msgs::Pose obstacle, int obstacle_type)
// {
//   double s, t, d;
//   double x00, x01, x10, x11;
//   double y00, y01, y10, y11;
//   double x_aux, y_aux;
//   double roll_start = start_node -> get_roll();
//   double roll_end = end_node -> get_roll();
//   std::vector<double> x_obs_coords;
//   std::vector<double> y_obs_coords;
//   std::vector<double> x_sta_coords;
//   std::vector<double> y_sta_coords;
//   std::vector<double> x_end_coords;
//   std::vector<double> y_end_coords;
//
//   bool res = false;
//
//   // u0 = (x00, y00)
//   // u1 = (x10, y10)
//   // v0 = (x01, y01)
//   // v1 = (x11, y11)
//
//   if (obstacle_type == RECTANGLE)
//   {
//
//     x_obs_coords.push_back(obstacle.position.x + CAR_SIZE_X);
//     x_obs_coords.push_back(obstacle.position.x + CAR_SIZE_X);
//     x_obs_coords.push_back(obstacle.position.x - CAR_SIZE_X);
//     x_obs_coords.push_back(obstacle.position.x - CAR_SIZE_X);
//
//     y_obs_coords.push_back(obstacle.position.y + CAR_SIZE_Y);
//     y_obs_coords.push_back(obstacle.position.y - CAR_SIZE_Y);
//     y_obs_coords.push_back(obstacle.position.y + CAR_SIZE_Y);
//     y_obs_coords.push_back(obstacle.position.y - CAR_SIZE_Y);
//
//     x_sta_coords.push_back((+CAR_SIZE_X) * cos(roll_start) - (+CAR_SIZE_Y) * sin(roll_start) + start_node -> point.x );
//     x_sta_coords.push_back((+CAR_SIZE_X) * cos(roll_start) - (-CAR_SIZE_Y) * sin(roll_start) + start_node -> point.x );
//     x_sta_coords.push_back((-CAR_SIZE_X) * cos(roll_start) - (+CAR_SIZE_Y) * sin(roll_start) + start_node -> point.x );
//     x_sta_coords.push_back((-CAR_SIZE_X) * cos(roll_start) - (-CAR_SIZE_Y) * sin(roll_start) + start_node -> point.x );
//
//     y_sta_coords.push_back((+CAR_SIZE_Y) * cos(roll_start) + (+ CAR_SIZE_X) * sin(roll_start) + start_node -> point.y);
//     y_sta_coords.push_back((-CAR_SIZE_Y) * cos(roll_start) + (+ CAR_SIZE_X) * sin(roll_start) + start_node -> point.y);
//     y_sta_coords.push_back((+CAR_SIZE_Y) * cos(roll_start) + (- CAR_SIZE_X) * sin(roll_start) + start_node -> point.y);
//     y_sta_coords.push_back((-CAR_SIZE_Y) * cos(roll_start) + (- CAR_SIZE_X) * sin(roll_start) + start_node -> point.y);
//
//     x_end_coords.push_back((+ CAR_SIZE_X) * cos(roll_end) - (+ CAR_SIZE_Y) * sin(roll_end) + end_node -> point.x);
//     x_end_coords.push_back((+ CAR_SIZE_X) * cos(roll_end) - (- CAR_SIZE_Y) * sin(roll_end) + end_node -> point.x);
//     x_end_coords.push_back((- CAR_SIZE_X) * cos(roll_end) - (+ CAR_SIZE_Y) * sin(roll_end) + end_node -> point.x);
//     x_end_coords.push_back((- CAR_SIZE_X) * cos(roll_end) - (- CAR_SIZE_Y) * sin(roll_end) + end_node -> point.x);
//
//     y_end_coords.push_back((+ CAR_SIZE_Y) * cos(roll_end) + (+ CAR_SIZE_X) * sin(roll_end) + end_node -> point.y);
//     y_end_coords.push_back((- CAR_SIZE_Y) * cos(roll_end) + (+ CAR_SIZE_X) * sin(roll_end) + end_node -> point.y);
//     y_end_coords.push_back((+ CAR_SIZE_Y) * cos(roll_end) + (- CAR_SIZE_X) * sin(roll_end) + end_node -> point.y);
//     y_end_coords.push_back((- CAR_SIZE_Y) * cos(roll_end) + (- CAR_SIZE_X) * sin(roll_end) + end_node -> point.y);
//
//
//     for (size_t j = 0; j < 4; j++)
//     {
//
//       x00 = x_sta_coords[j];
//       y00 = y_sta_coords[j];
//       x01 = x_end_coords[j] - x00;
//       y01 = y_end_coords[j] - y00;
//
//       for (size_t i = 0; i < 4; i++)
//       {
//         x10 = x_obs_coords[i];
//         y10 = y_obs_coords[i];
//         x11 = x_obs_coords[((i + 1) % 4)] - x_obs_coords[i];
//         y11 = y_obs_coords[((i + 1) % 4)] - y_obs_coords[i];
//         d = x11 * y01 - x01 * y11;
//         if (d == 0)
//         {
//           // lines are parallel
//         }
//         else
//         {
//           s = (1/d) *  ( (x00 - x10) * y01 - (y00 - y10) * x01);
//           t = (1/d) * -(-(x00 - x10) * y11 + (y00 - y10) * x11);
//           if (0 <= s && s <= 1 && 0 <= t && t <= 1 )
//           {
//             //   x00    y00     x01    y01      x10    y10      x11    y11
//             // printf("s = %.1f, t = %.1f, d = %.1f, roll_start = %.1f, roll_end = %.1f\n", s, t, d, roll_start, roll_end);
//             // printf("sta (%.2f, %.2f):\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\n", start.x, start.y, x_sta_coords[0], y_sta_coords[0], x_sta_coords[1], y_sta_coords[1], x_sta_coords[2], y_sta_coords[2], x_sta_coords[3], y_sta_coords[3]);
//             // printf("end (%.2f, %.2f):\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\n", end.x, end.y, x_end_coords[0], y_end_coords[0], x_end_coords[1], y_end_coords[1], x_end_coords[2], y_end_coords[2], x_end_coords[3], y_end_coords[3]);
//             // printf("obs (%.2f, %.2f):\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f)\n", obstacle.position.x, obstacle.position.y, x_obs_coords[0], y_obs_coords[0], x_obs_coords[1], y_obs_coords[1], x_obs_coords[2], y_obs_coords[2], x_obs_coords[3], y_obs_coords[3]);
//             // printf("x(t) = %.2f + %.2ft\ty(t) = %.2f + %.2ft\n", x00, x01, y00, y01);
//             // printf("x(t) = %.1f + %.1ft\ty(t) = %.1f + %.1ft\n", x_sta_coords[0], x_sta_coords[0] - x_end_coords[0], y_sta_coords[0], y_sta_coords[0] - y_end_coords[0]);
//             // printf("x(t) = %.1f + %.1ft\ty(t) = %.1f + %.1ft\n", x_sta_coords[1], x_sta_coords[1] - x_end_coords[1], y_sta_coords[1], y_sta_coords[1] - y_end_coords[1]);
//             // printf("x(t) = %.1f + %.1ft\ty(t) = %.1f + %.1ft\n", x_sta_coords[2], x_sta_coords[2] - x_end_coords[2], y_sta_coords[2], y_sta_coords[2] - y_end_coords[2]);
//             // printf("x(t) = %.1f + %.1ft\ty(t) = %.1f + %.1ft\n", x_sta_coords[3], x_sta_coords[3] - x_end_coords[3], y_sta_coords[3], y_sta_coords[3] - y_end_coords[3]);
//             // printf("(%.2f, %.2f)\t(%.2f, %.2f)\t(%.2f, %.2f). s = %.1f, t = %.1f, d = %.1f\n", start.x, start.y, end.x, end.y, obstacle.position.x, obstacle.position.y, s, t, d);
//             res = true;
//           }
//         }
//       }
//     }
//   }
//
//   return res;
//
// }

std::vector<node_g*> a_star_t::get_adj_points(node_g* node_ini,
                  std::vector<node_g*> nodes, std::string points_creation)
{
  std::vector<node_g*> out;

  if (points_creation == GRID)
  {
    for(auto n : nodes)
    {
      if (distance(node_ini, n) < goal_radius &&
          !ARE_NODES_PTR_EQUAL(node_ini, n) )
      {
        out.push_back(n);
      }
    }
  }
  else if (points_creation == CTRL)
  {
    std::vector<double> steering_vec = {-.5, 0, .5};
    std::vector<int> speed_vec = {-100, -150, -200};
    for (auto steering : steering_vec)
    {
      double angle = node_ini -> get_roll() + steering;
      for (auto speed : speed_vec )
      {
        double dist = get_distance_driven(speed);
        double x_point_tf = node_ini -> point.x + dist * sin(angle);
        double y_point_tf = node_ini -> point.y + dist * cos(angle);
        node_g* end_node;
        end_node = (node_g*) malloc(sizeof(node_g));
        end_node -> point.x = x_point_tf;
        end_node -> point.y = y_point_tf;
        end_node -> set_orientation(angle, 0, 0);
        // printf("aux:            (%.2f, %.2f, %.2f)\n", node_ini -> point.x, node_ini -> point.y, node_ini -> get_roll() );
        // printf("aux_get_pose2d: (%.2f, %.2f, %.2f)\n", node_ini -> get_pose2d().x, node_ini -> get_pose2d().y, node_ini -> get_pose2d().theta );
        if(collision_detector -> is_collision_free(node_ini -> get_pose2d(), end_node -> get_pose2d()))
        {
          // node_g* n;
          // n = (node_g*) malloc(sizeof(node_g));
          // n -> point.x = end_node -> point.x;
          // n -> point.y = end_node -> point.y;
          // n -> set_orientation(angle, 0, 0);
          // n -> parent = NULL;
          // out.push_back(n);
          end_node -> parent = NULL;
          out.push_back(end_node);
        }

      }
    }
  }
  // TODO: determine if the point can be reach by the car.
  return out;
}

double a_star_t::actual_cost(node_g *last, node_g *next)
{
  return distance(last, next) + next -> cost;
}

bool a_star_t::is_element_in_vector(std::vector<node_g*> vector, node_g* element)
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

bool a_star_t::pose_reached()
{
  // std::cout << "dist: " << distance(open.top(), goal) << "\t rad: " << goal_radius << '\n';
  return distance(open.top(), goal) <= goal_radius;
}

std_msgs::Float64MultiArray a_star_t::get_closed_lines()
{
  std_msgs::Float64MultiArray res;
  std_msgs::MultiArrayDimension layout_aux;

  for (auto e : closed)
  {
    if (e -> parent != NULL)
    {
      res.data.push_back(e -> point.x);
      res.data.push_back(e -> point.y);
      res.data.push_back(e -> parent -> point.x);
      res.data.push_back(e -> parent -> point.y);
    }
  }
  res.layout.data_offset = 0;
  layout_aux.label = "x_i";
  layout_aux.size = closed.size()*4;
  layout_aux.stride = 4*4*4*4;
  res.layout.dim.push_back(layout_aux);
  layout_aux.label = "y_i";
  layout_aux.size = closed.size()*4;
  layout_aux.stride = 4*4*4;
  res.layout.dim.push_back(layout_aux);
  layout_aux.label = "x_i + 1";
  layout_aux.size = closed.size()*4;
  layout_aux.stride = 4*4;
  res.layout.dim.push_back(layout_aux);
  layout_aux.label = "y_i + 1";
  layout_aux.size = closed.size()*4;
  layout_aux.stride = 4;
  res.layout.dim.push_back(layout_aux);
  return res;
}

std_msgs::Float64MultiArray a_star_t::get_opened_lines()
{
  std_msgs::Float64MultiArray res;
  std_msgs::MultiArrayDimension layout_aux;

  for (auto e : open.get_vector())
  {
    if (e -> parent != NULL)
    {
      res.data.push_back(e -> point.x);
      res.data.push_back(e -> point.y);
      res.data.push_back(e -> parent -> point.x);
      res.data.push_back(e -> parent -> point.y);
    }
  }
  res.layout.data_offset = 0;
  layout_aux.label = "x_i";
  layout_aux.size = open.get_vector().size() * 4;
  layout_aux.stride = 4*4*4*4;
  res.layout.dim.push_back(layout_aux);
  layout_aux.label = "y_i";
  layout_aux.size = open.get_vector().size() * 4;
  layout_aux.stride = 4*4*4;
  res.layout.dim.push_back(layout_aux);
  layout_aux.label = "x_i + 1";
  layout_aux.size = open.get_vector().size() * 4;
  layout_aux.stride = 4*4;
  res.layout.dim.push_back(layout_aux);
  layout_aux.label = "y_i + 1";
  layout_aux.size = open.get_vector().size() * 4;
  layout_aux.stride = 4;
  res.layout.dim.push_back(layout_aux);
  return res;
}

std_msgs::Float64MultiArray a_star_t::get_path_lines()
{
  std_msgs::Float64MultiArray res;
  std_msgs::MultiArrayDimension layout_aux;
  node_g* aux = open.top();
  int tot_nodes = 0;
  while (aux -> parent != NULL)
  {
    tot_nodes++;
    res.data.push_back(aux -> point.x);
    res.data.push_back(aux -> point.y);
    res.data.push_back(aux -> parent -> point.x);
    res.data.push_back(aux -> parent -> point.y);
    aux = aux -> parent;
  }
  res.layout.data_offset = 0;

  layout_aux.label = "x_i";
  layout_aux.size = tot_nodes * 4;
  layout_aux.stride = 4*4*4*4;
  res.layout.dim.push_back(layout_aux);
  layout_aux.label = "y_i";
  layout_aux.size = tot_nodes * 4;
  layout_aux.stride = 4*4*4;
  res.layout.dim.push_back(layout_aux);
  layout_aux.label = "x_i + 1";
  layout_aux.size = tot_nodes * 4;
  layout_aux.stride = 4*4;
  res.layout.dim.push_back(layout_aux);
  layout_aux.label = "y_i + 1";
  layout_aux.size = tot_nodes * 4;
  layout_aux.stride = 4;
  res.layout.dim.push_back(layout_aux);
  return res;
}

int a_star_t::get_total_nodes()
{
  return closed.size(); // add the opened nodes?
}

void a_star_t::reset_nodes()
{
  closed.clear();
  open.reset_queue();
}

void a_star_t::get_solution(std::vector<std::pair<double*,double> >& controls)
{
  // std::cout << "TODO" << '\n';
}
