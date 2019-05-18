#include "autonomos.hpp"
#include "utilities/random.hpp"


#define _USE_MATH_DEFINES

#define MAX_SPEED -200.0

#define POS_X_BOUND -0.0 //+10.0
#define NEG_X_BOUND -10.0 //-01.0
#define POS_Y_BOUND +10.0 //+07.0
#define NEG_Y_BOUND -10.0 //-07.0

#include <cmath>

autonomos_t::autonomos_t(std::string ctrl_to_use_in, bool global_planning)
{
  state_dimension = 3;
  control_dimension = 2;
  temp_state = new double[state_dimension];
  collision_detector = new collision_detector_t();

  this -> global = global_planning;

  if (ctrl_to_use_in == RANDOM_CTRL ||
      ctrl_to_use_in == BANG_BANG)
  {
    ctrl_to_use = ctrl_to_use_in;
  }
  else
  {
    ROS_FATAL_STREAM("Not a valid controller: " << ctrl_to_use_in);

  }

  set_bounds(POS_X_BOUND, NEG_X_BOUND, POS_Y_BOUND, NEG_Y_BOUND);

}

autonomos_t::~autonomos_t()
{
  free(temp_state);
}

void autonomos_t::set_bounds( double pos_x_bound, double neg_x_bound, double pos_y_bound, double neg_y_bound )
{
  this -> pos_x_bound = pos_x_bound;
  this -> neg_x_bound = neg_x_bound;
  this -> pos_y_bound = pos_y_bound;
  this -> neg_y_bound = neg_y_bound;

  this -> planning_rad_range = -1;
  
  ROS_WARN_STREAM("Bounding rectangle: ( " << pos_x_bound << " , " << pos_y_bound << ") , ( "
                  << neg_x_bound << " , " << neg_y_bound << " )." );
}

void autonomos_t::set_current_loc(double x, double y, double theta)
{
  current_loc.x = x;
  current_loc.y = y;
  current_loc.theta = theta;
}


void autonomos_t::set_bounds( double planning_rad_range )
{
  ROS_ASSERT_MSG(planning_rad_range >= 0, "Local planning range radius must be greater than 0.");
  this -> planning_rad_range = planning_rad_range;

  this -> pos_x_bound = 0;
  this -> neg_x_bound = 0;
  this -> pos_y_bound = 0;
  this -> neg_y_bound = 0;
}

double autonomos_t::distance(double* point1, double* point2)
{
	double val = fabs(point1[2]-point2[2]);
	if(val > M_PI)
		val = 2*M_PI-val;
	return std::sqrt( val * val + (point1[1]-point2[1]) * (point1[1]-point2[1])+(point1[0]-point2[0]) * (point1[0]-point2[0]) );
}

void autonomos_t::random_state(double* state)
{
	state[0] = current_loc.x + uniform_random(-planning_rad_range, planning_rad_range);
	state[1] = current_loc.y + uniform_random(-planning_rad_range, planning_rad_range);
	state[2] = uniform_random(-M_PI,M_PI);
}

void autonomos_t::random_control(double* control)
{
  // control[0] -> SPEED
  // control[1] -> STEERING
  if (ctrl_to_use == RANDOM_CTRL)
  {
    control[0] = uniform_random(-1,1);// * MAX_SPEED;
    control[1] = uniform_random(-.5,.5);
  }
  else if (ctrl_to_use == BANG_BANG)
  {
    bang_bang_ctrl(control);
  }

}

void autonomos_t::bang_bang_ctrl(double* control)
{
  int speed_aux = uniform_random(0, 5);
  int steer_aux = uniform_random(0, 3);
  switch (speed_aux) {
    case 0:  control[0] = 1.0 / 3.0;
    break;
    case 1:  control[0] = 2.0 / 3.0;
    break;
    case 2:  control[0] = -1.0 / 3.0;
    break;
    case 3:  control[0] = -2.0 / 3.0;
    break;
    default: control[0] = 1;
    break;
  }

  switch (steer_aux) {
    case 0:  control[1] = -.5;
    break;
    case 1:  control[1] = 0.0;
    break;
    default: control[1] = 0.5;
    break;

  }
}

bool autonomos_t::propagate( double* start_state, double* control, int min_step,
  int max_step, double* result_state, double& duration )
{
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];temp_state[2] = start_state[2];

	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{
		double temp0 = temp_state[0];
		double temp1 = temp_state[1];
		double temp2 = temp_state[2];
    temp_state[0] += params::integration_step*cos(temp2)*control[0];
		temp_state[1] += params::integration_step*sin(temp2)*control[0];
    // temp_state[2] += params::integration_step*control[1];
		temp_state[2] += params::integration_step * tan(control[1]) * control[0] / 0.25;

    // temp_state[0] += params::integration_step*cos(control[1])*control[0];
    // temp_state[1] += params::integration_step*sin(control[1])*control[0];
    // temp_state[2] += params::integration_step*tan(control[1]) * control[0];
		enforce_bounds();
		validity = validity && valid_state();
	}

	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];
	result_state[2] = temp_state[2];
	duration = num_steps*params::integration_step;
	return validity;
}

void autonomos_t::enforce_bounds()
{
  in_bounds = true;
  if (global)
  {
    global_rectangular_bound();
  }
  else
  {
    local_circular_bound();
  }
}

void autonomos_t::local_circular_bound()
{
  ROS_ASSERT_MSG(planning_rad_range >= 0, "Local planning range radius must be greater than 0.");
  
  double dist;

  dist = sqrt( pow(current_loc.x - temp_state[0], 2 ) +  pow(current_loc.y - temp_state[1], 2 ) );
  // ROS_WARN_STREAM( "X: " << current_loc.x << "\tX_P: " << temp_state[0] << "\tY: " << current_loc.y << "\tY_P: " << temp_state[1] <<  "\tdist: " << dist );

  // if (dist > planning_range )
    // in_bounds = false;

  in_bounds = dist < planning_rad_range;

  if (temp_state[2]<-M_PI)
    temp_state[2]+=2*M_PI;
  else if(temp_state[2]>M_PI)
    temp_state[2]-=2*M_PI;
}

void autonomos_t::global_rectangular_bound()
{

  if(temp_state[0] < NEG_X_BOUND)
    in_bounds = false;
  else if(temp_state[0] > POS_X_BOUND)
    in_bounds = false;

  if(temp_state[1] < NEG_Y_BOUND)
    in_bounds = false;
  else if(temp_state[1] > POS_Y_BOUND)
    in_bounds = false;

  if(temp_state[2]<-M_PI)
    temp_state[2]+=2*M_PI;
  else if(temp_state[2]>M_PI)
    temp_state[2]-=2*M_PI;
}

bool autonomos_t::valid_state()
{

  geometry_msgs::Pose2D p1, p2;
  p1.x = temp_state[0];
  p1.y = temp_state[1];
  p1.theta = temp_state[2];
  p2.x = temp_state[0];
  p2.y = temp_state[1];
  p2.theta = temp_state[2];
  bool aux_collision = true;
  aux_collision =  collision_detector -> is_collision_free(p1, p2);
  // std::string aux_str = aux_collision?"true":"false";
  // std::cout << "is_collision_free: " << aux_str ;
  // if (!aux_collision)
  // {
  //   std::cout << "Collision detected!";
  //   std::cout << std::endl;
  // }
	return  aux_collision && in_bounds;

}

svg::Point autonomos_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]+10)/(20) * dims.width;
	double y = (state[1]+10)/(20) * dims.height;
	return svg::Point(x,y);
}

void autonomos_t::set_obstacles(std::vector<geometry_msgs::Pose> vec_obstacles_poses,
  std::vector<int> vec_obstacles_type, double in_obstacles_radius)
{
  collision_detector -> set_obstacles(vec_obstacles_poses, vec_obstacles_type);
  collision_detector -> set_obstacles_radius(in_obstacles_radius);
}
