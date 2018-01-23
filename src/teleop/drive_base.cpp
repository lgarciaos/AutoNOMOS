#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  ros::Publisher steering_pub_;
  // speed
  geometry_msgs::Twist base_cmd;
  //steer
  std_msgs::Float64 value_steering;
  //rate
  double rate_hz;
  std::string topico_steering;
  std::string topico_velocidad;

  

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    rate_hz = 1;
    std::string node_name = ros::this_node::getName();

    nh_.param<std::string>(node_name+"/topico_steering", topico_steering, "/autonomos/steer/steer_position_controller/command");
    nh_.param<std::string>(node_name+"/topico_velocidad", topico_velocidad, "/cmd_vel");

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(topico_velocidad, rate_hz);
    steering_pub_ = nh.advertise<std_msgs::Float64>(topico_steering, rate_hz);

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    value_steering.data = 0.0;

  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    std::cout << "\n" << "Type a command and then press enter.  " << "\n"
      << "Use 'w' to increase speed, 's' to decrease it " << "\n" 
      << "'a' to turn left, 'd' to turn right, '.' to exit.\n";

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    char cmd;
    while(nh_.ok()){

      //std::cin.get(cmd);
      std::cin.get(cmd);
      if(cmd!='w' && cmd!='s' && cmd!='a' && cmd!='d' && cmd!='.')
      {
        std::cout << "\n" << "unknown command:" << cmd ;
        continue;
      }
      
      //move forward
      if(cmd=='w'){
        base_cmd.linear.x -= 0.05;
      }
      else if(cmd=='s'){
        base_cmd.linear.x += 0.05;
      }
      //turn left (yaw) and drive forward at the same time
      else if(cmd=='a'){
        if (value_steering.data < 0.45) {
          value_steering.data += 0.05;
        }
      }
      //turn right (yaw) and drive forward at the same time
      else if(cmd=='d'){
        if (value_steering.data > -0.45) {
          value_steering.data -= 0.05;
        }
      }
      //quit
      else if(cmd=='.'){
        
        break;
      }

      printf("\n Velocity: %.2f, Steering: %.2f", base_cmd.linear.x, value_steering.data);

      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);
      steering_pub_.publish(value_steering);
    }
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "drive_base");
  ros::NodeHandle nh;

  // this struct are to prevent users to press enter on every char
  static struct termios oldt, newt;

  /*tcgetattr gets the parameters of the current terminal
    STDIN_FILENO will tell tcgetattr that it should write the settings
    of stdin to oldt*/
    tcgetattr( STDIN_FILENO, &oldt);
    /*now the settings will be copied*/
    newt = oldt;

    /*ICANON normally takes care that one line at a time will be processed
    that means it will return if it sees a "\n" or an EOF or an EOL*/
    newt.c_lflag &= ~(ICANON);          

    /*Those new settings will be set to STDIN
    TCSANOW tells tcsetattr to change attributes immediately. */
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);

  RobotDriver driver(nh);
  driver.driveKeyboard();

  /*restore the old settings*/
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}
