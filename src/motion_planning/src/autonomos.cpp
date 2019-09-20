#include "autonomos.hpp"
#include "utilities/random.hpp"


#define _USE_MATH_DEFINES

#define MAX_SPEED -200.0

#define POS_X_BOUND -0.0 //+10.0
#define NEG_X_BOUND -10.0 //-01.0
#define POS_Y_BOUND +10.0 //+07.0
#define NEG_Y_BOUND -10.0 //-07.0

#include <cmath>

using std::string;

autonomos_t::autonomos_t(std::string ctrl_to_use_in, bool global_planning, string name)
{
  state_dimension = 3;
  control_dimension = 2;
  temp_state = new double[state_dimension];
  // collision_detector = new collision_detector_t();

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

  CCD_INIT(&ccd); // initialize ccd_t struct

      // set up ccd_t struct
  ccd.support1       = autonomos_t::support; // support function for first object
  ccd.support2       = autonomos_t::support; // support function for second object
  ccd.max_iterations = 100; 
  autonomos_name = name;
  // allow_reverse = false;
}

void autonomos_t::support(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    // assume that obj_t is user-defined structure that holds info about
    // object (in this case box: x, y, z, pos, quat - dimensions of box,
    // position and rotation)
    obstacle_t *obj = (obstacle_t *)_obj;
    ccd_vec3_t dir, pos;
    ccd_quat_t qinv, q_aux;

    // apply rotation on direction vector
    // ccdVec3Copy(&dir, _dir);
    double x, y, z;
    double dim_x, dim_y, dim_z;
    double qx, qy, qz, qw;
    obj -> get_xyz(x, y, z);
    obj -> get_quat_xyzw(qx, qy, qz, qw);
    obj -> get_bounding_box_dimensions(dim_x, dim_y, dim_z);
    ccdVec3Set(&pos, x, y, z);
    ccdVec3Copy(&dir, _dir);
    ccdQuatSet(&q_aux, qx, qy, qz, qw); 
    ccdQuatInvert2(&qinv, &q_aux);
    ccdQuatRotVec(&dir, &qinv);

    // // compute support point in specified direction
    ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * dim_x * CCD_REAL(0.5),
                  ccdSign(ccdVec3Y(&dir)) * dim_y * CCD_REAL(0.5),
                  ccdSign(ccdVec3Z(&dir)) * dim_z * CCD_REAL(0.5));

    // // transform support point according to position and rotation of object
    ccdQuatRotVec(v, &q_aux);
    ccdVec3Add(v, &pos);
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
	double val = fabs(point1[2]-point2[2]), res;
	if(val > M_PI)
		val = 2*M_PI-val;
  // only_geometric = true;
  // if (only_geometric)
  // {
    res = std::sqrt( pow( point1[1] - point2[1], 2) + pow( point1[0] - point2[0], 2));
  // } 
  // else
  // {
	  // res = std::sqrt( val * val + 
      // (point1[1]-point2[1]) * (point1[1]-point2[1])+(point1[0]-point2[0]) * (point1[0]-point2[0]) );
  // }
  return res;
}

void autonomos_t::random_state(double* state)
{
	if( global)
	{
		state[0] = uniform_random(this -> neg_x_bound, this -> pos_x_bound);
		state[1] = uniform_random(this -> neg_y_bound, this -> pos_y_bound);
		state[2] = uniform_random(-M_PI,M_PI);	
	
	}
	else
	{
		state[0] = current_loc.x + uniform_random(-planning_rad_range, planning_rad_range);
		state[1] = current_loc.y + uniform_random(-planning_rad_range, planning_rad_range);
		state[2] = uniform_random(-M_PI,M_PI);
	}

}

void autonomos_t::random_control(double* control, bool allow_reverse)
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

  // if (!allow_reverse)
  // {
  //   control[0] = abs(control[0]);
  // }

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
		// double temp0 = temp_state[0];
		// double temp1 = temp_state[1];
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

  if(temp_state[0] < this -> neg_x_bound )
    in_bounds = false;
  else if(temp_state[0] > this -> pos_x_bound )
    in_bounds = false;

  if(temp_state[1] < this -> neg_y_bound  )
    in_bounds = false;
  else if(temp_state[1] > this -> pos_y_bound )
    in_bounds = false;
	
	/*
  std::cout << "-x = " << this -> neg_x_bound <<
  	  " +x = " << this -> pos_x_bound <<
  	  " -y = " << this -> neg_y_bound <<
  	  " +y = " << this -> pos_y_bound <<
	  " tmp_state: ( " << temp_state[0] << ", " <<
	  temp_state[1] << ") in_bounds: " << in_bounds << std::endl;
	*/

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
  bool collision = false;

  obstacle_t robot_temp = robot_obj;
  robot_temp.set_pose2D(temp_state[0], temp_state[1], temp_state[2]);
  // std::cout << "original: " << robot_obj << std::endl;
  // std::cout << "new: " << robot_temp << std::endl;
  for( auto obs : static_obstacles)
  {
    collision = ccdGJKIntersect(&robot_temp, &obs, &ccd);
    if (collision)
    {
      // ROS_WARN_STREAM("robot: " << robot_temp << "\nobs: " << obs << "\nintersection: " << collision);
      break;
    }
  }
  // ROS_WARN_STREAM("Collision: " << collision);
	return  !collision && in_bounds;

}

svg::Point autonomos_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = ( state[0] + 10 ) / (20) * dims.width;
	double y = ( state[1] + 10 ) / (20) * dims.height;
	return svg::Point(x,y);
}

void autonomos_t::set_obstacles(motion_planning::obstacles_array::Response msg)
{
  dynamic_obstacles.clear();
  for (long unsigned int i = 0; i < msg.names.size(); ++i)
  {
    // obstacle_t obs(msg.names[i], msg.poses[i], msg.bounding_boxes[i], msg.is_static[i]);
    obstacle_t obs(msg.names[i], msg.poses[i], msg.bounding_boxes[i], 
        msg.is_static[i], msg.bounding_boxes_dimensions[i]);
    // std::cout << "obs " << obs << std::endl;
    //std::cout << "name: " << msg.names[i] << std::endl;
    if (msg.names[i] == autonomos_name)
    {
      robot_obj = obs;
    }
    else if (msg.is_static[i])
    {
      //printf("static: %s", msg.names[i]);
	    static_obstacles.insert(obs);
    } 
    else
    {
      //printf("dynamic: %s", msg.names[i]);
      dynamic_obstacles.insert(obs);
    }
  }
}

// std::set<obstacle_t> autonomos_t::get_dynamic_obstacles()
bool autonomos_t::get_next_dynamic_state(double* state, int i)
{
  
  if ( dynamic_obstacles.size() > 0  )
  {
    auto it_aux = dynamic_obstacles.begin();//
    //std::next(it_aux, i);
    (*it_aux).get_xyz(state[0], state[1], state[2]);
    //printf("%d: %s", i, (*it_aux).get_name());
    //std::cout << i << ": " << (*it_aux).get_name() << std::endl;
    (*it_aux).get_name();
    dynamic_obstacles.erase(it_aux);
    return true;
  }
  else
  {
    return false;
  }
}
