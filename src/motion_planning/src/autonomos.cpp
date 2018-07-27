#include "autonomos.hpp"
#include "utilities/random.hpp"


#define _USE_MATH_DEFINES

#define MAX_SPEED -200.0

#define POS_X_BOUND +15.0
#define NEG_X_BOUND -15.0
#define POS_Y_BOUND +15.0
#define NEG_Y_BOUND -15.0

#include <cmath>

autonomos_t::autonomos_t()
{
  state_dimension = 3;
  control_dimension = 2;
  temp_state = new double[state_dimension];
  collision_detector = new collision_detector_t();
}

autonomos_t::~autonomos_t()
{

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
	state[0] = uniform_random(NEG_X_BOUND, POS_X_BOUND);
	state[1] = uniform_random(NEG_Y_BOUND, POS_Y_BOUND);
	state[2] = uniform_random(-M_PI,M_PI);
}

void autonomos_t::random_control(double* control)
{
	control[0] = uniform_random(0,1);// * MAX_SPEED;
	control[1] = uniform_random(-.5,.5);
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
		temp_state[2] += params::integration_step*control[1];
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
	if(temp_state[0] < NEG_X_BOUND)
		temp_state[0] = NEG_X_BOUND;
	else if(temp_state[0] > POS_X_BOUND)
		temp_state[0] = POS_X_BOUND;

	if(temp_state[1] < NEG_Y_BOUND)
		temp_state[1] = NEG_Y_BOUND;
	else if(temp_state[1] > POS_Y_BOUND)
		temp_state[1] = POS_Y_BOUND;

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
	return  aux_collision &&
      (temp_state[0] != NEG_X_BOUND) &&
			(temp_state[0] != POS_X_BOUND) &&
			(temp_state[1] != NEG_Y_BOUND) &&
			(temp_state[1] != POS_Y_BOUND);
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
