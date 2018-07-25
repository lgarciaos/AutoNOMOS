#include "collision_detector.h"

void collision_detector_t::set_obstacles(std::vector<geometry_msgs::Pose> in_obstacles_poses,
  std::vector<int> in_obstacles_types)
{
  obstacles_poses = in_obstacles_poses;
  obstacles_types = in_obstacles_types;
}

bool collision_detector_t::is_collision_free(geometry_msgs::Pose2D start_pose,
  geometry_msgs::Pose2D end_pose)
{
  for (size_t i = 0; i < obstacles_poses.size(); i++)
  {
    if (obstacles_types[i] == RECTANGLE && pose_in_car(end_pose, obstacles_poses[i], false))
    {
      return false;
    }
    if (path_intersects_obstacle(start_pose, end_pose, obstacles_poses[i], obstacles_types[i]))
    {
      return false;
    }
  }
  return true;
}

bool collision_detector_t::pose_in_car(geometry_msgs::Pose2D pose,
  geometry_msgs::Pose center_car, bool print)
{
    bool x_match = false;
    bool y_match = false;

    tf::Quaternion orient (center_car.orientation.x,
                           center_car.orientation.y,
                           center_car.orientation.z,
                           center_car.orientation.w);

    tf::Matrix3x3 m(orient);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double x_point_tf = pose.x - center_car.position.x;
    double y_point_tf = pose.y - center_car.position.y;
    double angle = roll;

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
        "( " << pose.x << ", " << pose.y << " )\t\t" <<
        "( " << center_car.position.x << ", " << center_car.position.y << " )\t\t" <<
        "( " << x_point_tf << ", " << y_point_tf << " )" << '\n';
    }

    return x_match & y_match;
}

bool collision_detector_t::path_intersects_obstacle(geometry_msgs::Pose2D start_pose,
  geometry_msgs::Pose2D end_pose, geometry_msgs::Pose obstacle, int obstacle_type)
{
  double s, t, d;
  double x00, x01, x10, x11;
  double y00, y01, y10, y11;
  double x_aux, y_aux;
  double roll_start = start_pose.theta;
  double roll_end = end_pose.theta;
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

  if (obstacle_type == RECTANGLE)
  {

    x_obs_coords.push_back(obstacle.position.x + CAR_SIZE_X);
    x_obs_coords.push_back(obstacle.position.x + CAR_SIZE_X);
    x_obs_coords.push_back(obstacle.position.x - CAR_SIZE_X);
    x_obs_coords.push_back(obstacle.position.x - CAR_SIZE_X);

    y_obs_coords.push_back(obstacle.position.y + CAR_SIZE_Y);
    y_obs_coords.push_back(obstacle.position.y - CAR_SIZE_Y);
    y_obs_coords.push_back(obstacle.position.y + CAR_SIZE_Y);
    y_obs_coords.push_back(obstacle.position.y - CAR_SIZE_Y);

    x_sta_coords.push_back((+CAR_SIZE_X) * cos(roll_start) - (+CAR_SIZE_Y) * sin(roll_start) + start_pose.x );
    x_sta_coords.push_back((+CAR_SIZE_X) * cos(roll_start) - (-CAR_SIZE_Y) * sin(roll_start) + start_pose.x );
    x_sta_coords.push_back((-CAR_SIZE_X) * cos(roll_start) - (+CAR_SIZE_Y) * sin(roll_start) + start_pose.x );
    x_sta_coords.push_back((-CAR_SIZE_X) * cos(roll_start) - (-CAR_SIZE_Y) * sin(roll_start) + start_pose.x );

    y_sta_coords.push_back((+CAR_SIZE_Y) * cos(roll_start) + (+ CAR_SIZE_X) * sin(roll_start) + start_pose.y);
    y_sta_coords.push_back((-CAR_SIZE_Y) * cos(roll_start) + (+ CAR_SIZE_X) * sin(roll_start) + start_pose.y);
    y_sta_coords.push_back((+CAR_SIZE_Y) * cos(roll_start) + (- CAR_SIZE_X) * sin(roll_start) + start_pose.y);
    y_sta_coords.push_back((-CAR_SIZE_Y) * cos(roll_start) + (- CAR_SIZE_X) * sin(roll_start) + start_pose.y);

    x_end_coords.push_back((+ CAR_SIZE_X) * cos(roll_end) - (+ CAR_SIZE_Y) * sin(roll_end) + end_pose.x);
    x_end_coords.push_back((+ CAR_SIZE_X) * cos(roll_end) - (- CAR_SIZE_Y) * sin(roll_end) + end_pose.x);
    x_end_coords.push_back((- CAR_SIZE_X) * cos(roll_end) - (+ CAR_SIZE_Y) * sin(roll_end) + end_pose.x);
    x_end_coords.push_back((- CAR_SIZE_X) * cos(roll_end) - (- CAR_SIZE_Y) * sin(roll_end) + end_pose.x);

    y_end_coords.push_back((+ CAR_SIZE_Y) * cos(roll_end) + (+ CAR_SIZE_X) * sin(roll_end) + end_pose.y);
    y_end_coords.push_back((- CAR_SIZE_Y) * cos(roll_end) + (+ CAR_SIZE_X) * sin(roll_end) + end_pose.y);
    y_end_coords.push_back((+ CAR_SIZE_Y) * cos(roll_end) + (- CAR_SIZE_X) * sin(roll_end) + end_pose.y);
    y_end_coords.push_back((- CAR_SIZE_Y) * cos(roll_end) + (- CAR_SIZE_X) * sin(roll_end) + end_pose.y);


    for (size_t j = 0; j < 4; j++)
    {
      x00 = x_sta_coords[j];
      y00 = y_sta_coords[j];
      x01 = x_end_coords[j] - x00;
      y01 = y_end_coords[j] - y00;

      for (size_t i = 0; i < 4; i++)
      {
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

void collision_detector_t::set_obstacles_radius(double in_obstacles_radius)
{
  obstacles_radius = in_obstacles_radius;
}

double collision_detector_t::distance(geometry_msgs::Pose2D p1, geometry_msgs::Pose2D p2)
{
  double d_x = p1.x - p2.x;
  double d_y = p1.y - p2.y;
  return sqrt( d_x * d_x + d_y * d_y);
}