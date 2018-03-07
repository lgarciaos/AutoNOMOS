#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

#define RATE_HZ 100

// This project standarizes how calls are made from every node in the proyect to the car
// making transparent the selection if one looks to use simulation or the real car

static const uint32_t MY_ROS_QUEUE_SIZE = 1;

class TranslationSimReal
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! publicar a un topico intermedio estandarizado, para hacer transparene cambio entre simulacion o carro real 
  ros::Publisher _pub_actions_standarized;
  // topicos que finalmente publican al carro
  ros::Publisher _pub_steering;
  ros::Publisher _pub_velocity;

  // Suscribers to the topics
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber steering_sub;
  // suscribirse tambien al topico estandarizado para de ahi publicar al carro real
  ros::Subscriber actions_standarized_sub;

  // standarized value
  geometry_msgs::Twist standarized_values;

  // double rate_hz;

  std::string topico_estandarizado;

  std::string topico_steering_sim; // std_msgs::Float64
  std::string topico_velocidad_sim; // geometry_msgs::Twist
  std::string topico_steering_real; // std_msgs::Int16
  std::string topico_velocidad_real; // std_msgs::Int16

  std::string seleccion_real_simulacion;

  // values simulation
  geometry_msgs::Twist value_velocity_sim;
  std_msgs::Float64 value_steering_sim;

  // values real car
  std_msgs::Int16 value_steering_real;
  std_msgs::Int16 value_velocity_real;

public:
  //! ROS node initialization
  TranslationSimReal(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    
    std::string node_name = ros::this_node::getName();

    
    nh_.param<std::string>(node_name+"/topico_steering_sim", topico_steering_sim, "/autonomos/steer/steer_position_controller/command");
    nh_.param<std::string>(node_name+"/topico_velocidad_sim", topico_velocidad_sim, "/cmd_vel");

    nh_.param<std::string>(node_name+"/topico_steering_real", topico_steering_real, "/manual_control/steering");
    nh_.param<std::string>(node_name+"/topico_velocidad_real", topico_velocidad_real, "/manual_control/speed");

    nh_.param<std::string>(node_name+"/topico_estandarizado", topico_estandarizado, "/standarized_vel_ste");
    nh_.param<std::string>(node_name+"/seleccion_real_simulacion", seleccion_real_simulacion, "simulacion");

    
    if (seleccion_real_simulacion == "simulacion") {
      cmd_vel_sub = nh_.subscribe(topico_velocidad_sim, MY_ROS_QUEUE_SIZE, &TranslationSimReal::get_velocity_sim, this);
      steering_sub = nh_.subscribe(topico_steering_sim, MY_ROS_QUEUE_SIZE, &TranslationSimReal::get_steering_sim, this);
      _pub_velocity = nh_.advertise<geometry_msgs::Twist>(topico_velocidad_sim, MY_ROS_QUEUE_SIZE);
      _pub_steering = nh_.advertise<std_msgs::Float64>(topico_steering_sim, MY_ROS_QUEUE_SIZE);
    } else {
      /* TODO - real */
      cmd_vel_sub = nh_.subscribe(topico_velocidad_real, MY_ROS_QUEUE_SIZE, &TranslationSimReal::get_velocity_real, this);
      steering_sub = nh_.subscribe(topico_steering_real, MY_ROS_QUEUE_SIZE, &TranslationSimReal::get_steering_real, this);
      _pub_velocity = nh_.advertise<std_msgs::Int16>(topico_velocidad_real, MY_ROS_QUEUE_SIZE);
      _pub_steering = nh_.advertise<std_msgs::Int16>(topico_steering_real, MY_ROS_QUEUE_SIZE);
    }
    
    // suscriber estandarizado
    actions_standarized_sub = nh_.subscribe(topico_estandarizado, MY_ROS_QUEUE_SIZE, &TranslationSimReal::get_standarized_actions, this);

    // publisher estandarizado
    _pub_actions_standarized = nh_.advertise<geometry_msgs::Twist>(topico_estandarizado, MY_ROS_QUEUE_SIZE);
    
    // initialize vars
    // base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    // value_steering.data = 0.0;

  }



  // SIMULATION
  //transforms the motion into values for shift >> used before but maybe not useful anymore (290317)
  void get_steering_sim(const std_msgs::Float64& val)
  {
    standarized_values.angular.z = val.data;
    _pub_actions_standarized.publish(standarized_values);
  }

  void get_velocity_sim(const geometry_msgs::Twist& val)
  {
    standarized_values.linear.x = val.linear.x;
    _pub_actions_standarized.publish(standarized_values);
  }

  void get_standarized_actions(const geometry_msgs::Twist& val)
  {
    if (seleccion_real_simulacion == "simulacion") {
      if (value_velocity_sim.linear.x != val.linear.x) {
        value_velocity_sim.linear.x = val.linear.x;
        _pub_velocity.publish(value_velocity_sim);
      }

      if (value_steering_sim.data != val.angular.z) {
        value_steering_sim.data = val.angular.z;
        _pub_steering.publish(value_steering_sim);
      }
    } else {
      if (value_velocity_real.data != val.linear.x) {
        value_velocity_real.data = val.linear.x;
        _pub_velocity.publish(value_velocity_real);
      }

      if (value_steering_real.data != val.angular.z) {
        // transformar radianes de regreso a grados
        value_steering_real.data = val.angular.z;
        _pub_steering.publish(value_steering_real);
      }
    }
  }

  // REAL CAR
  void get_steering_real(const std_msgs::Int16& val)
  {
    // transformar grados a radianes internamente para estandarizar
    standarized_values.angular.z = val.data; // REQUIERE conversion a radianes
    _pub_actions_standarized.publish(standarized_values);
  }

  void get_velocity_real(const std_msgs::Int16& val)
  {
    standarized_values.linear.x = val.data; // hace falta convertir a m/s
    _pub_actions_standarized.publish(standarized_values);
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "translate_sim_real");
  ros::NodeHandle nh;
  ros::Rate loop_rate(RATE_HZ);

  TranslationSimReal driver(nh);

  // driver.translate_instructions();

  //! Loop forever while translating the instructions from simulation or real car to a standarized topic
  while(nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;

}
