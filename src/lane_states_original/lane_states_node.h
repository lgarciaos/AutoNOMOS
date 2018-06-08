#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/Header.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <string>
#include <sstream>
#include <cmath>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "tf/tf.h"

#include <sensor_msgs/Imu.h>
#include "tools/ackerman.h"

#include <nav_msgs/GridCells.h>
// #include <nav_msgs/Odometry.h>

#include <geometry_msgs/Pose2D.h>


#include "../fu_line_detection/src/tools/NewtonPolynomial.h"

static const uint32_t MY_ROS_QUEUE_SIZE = 100;

#define RATE_HZ 5

#define NUM_STATES 7
#define STATE_WIDTH 20

#define RADIO 15
#define PI 3.14159265

#define HISTORY_IMU 20
#define HISTORY_POS 20



class clane_states {

private:

    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    ros::Publisher pub_lidar;

    ros::Subscriber sub_ransac_left ;
    ros::Subscriber sub_ransac_center ;
    ros::Subscriber sub_ransac_right ;

    ros::Subscriber sub_pts_left;
    ros::Subscriber sub_pts_center;
    ros::Subscriber sub_pts_right;

    ros::Subscriber sub_des_state ;
    // no es confiable velocidad y steering
    ros::Subscriber sub_mov ;
    // probar con imu
    ros::Subscriber sub_imu ;
    // ros::Subscriber sub_orientation = nh.subscribe("/car_orientation", MY_ROS_QUEUE_SIZE, &get_car_orientation);
    ros::Subscriber sub_robot_pos ;



    geometry_msgs::Twist destiny_position;

    std::string nombre;



    NewtonPolynomial poly_left;
    NewtonPolynomial poly_center;
    NewtonPolynomial poly_right;

    bool polyDetectedLeft;
    bool polyDetectedCenter;
    bool polyDetectedRight;

    geometry_msgs::Twist car_global_pose;

    const float T_imu = 0.1; // rate of imu topic

    // sensor
    int car_center;
    float car_orientation ;
    int des_state;
    float ctrl_estado;
    double dist_x_steering;
    double pix_x_prob;
    int pixeles_cambio_estado;
    double odom_x;
    double odom_y;
    double odom_theta;

    double odom_pos_y_old;

    int state_width_pix;
    int model_num_gazebo;

    // imu vars

    double accx [HISTORY_IMU], accy [HISTORY_IMU];
    double velocityx [HISTORY_IMU], velocityy [HISTORY_IMU];
    double positionx [HISTORY_IMU], positiony [HISTORY_IMU];

    double global_position_x [HISTORY_POS];
    double global_position_y [HISTORY_POS];
    double global_orientation [HISTORY_POS];

    void get_ransac_left(const nav_msgs::GridCells& poly);

    void get_ransac_center(const nav_msgs::GridCells& poly);

    void get_ransac_right(const nav_msgs::GridCells& poly);

    void get_pts_left(const nav_msgs::GridCells& array);

    void get_pts_center(const nav_msgs::GridCells& array);

    void get_pts_right(const nav_msgs::GridCells& array);

    void get_ctrl_action(const geometry_msgs::Twist& val);

    void get_ctrl_desired_state(const std_msgs::Int16& val);

    void get_car_orientation(const std_msgs::Float32& val);


    void global_pose_callback(const geometry_msgs::Pose2D& global_pose);


    void get_imu(const sensor_msgs::Imu& val);

    float horizontal_dist(geometry_msgs::Point p1, geometry_msgs::Point p2);

    int det_hit (int position, int lanes_detected, bool ll, bool cc, bool rr, float dist_sensado_ll, float dist_sensado_cc, float dist_sensado_rr);

    int actual_state(std_msgs::Float32MultiArray locArray);

    int image_height;

    double cte(double global_pose_x, double global_pose_y);
public:

    ros::Publisher pub_loc;
    ros::Publisher pub_image;
    cv::Mat img_hist;
    std::string nombre_estado [NUM_STATES] = { "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR"};
    float actual_speed_rpm;
    float actual_steering;

    int R;
    int C;
    int L;
    nav_msgs::GridCells arr_left;
    nav_msgs::GridCells arr_center;
    nav_msgs::GridCells arr_right;


    clane_states(ros::NodeHandle nh);

    float* det_hits(float *dist_sensado_ll, float *dist_sensado_cc, float *dist_sensado_rr, int *lanes_detected);

    std_msgs::Float32MultiArray sense(std_msgs::Float32MultiArray p, float* hits);

    std_msgs::Float32MultiArray move(std_msgs::Float32MultiArray prob, double delta_pos_y, int *U);

    void write_to_file_headers();

    void write_to_file(std_msgs::Float32MultiArray m_array, float* p, int values);

    void write_to_image( cv::Mat& imagen, float* hits, std_msgs::Float32MultiArray sense, std_msgs::Float32MultiArray move, int values, int borrarSenseImagen, int *inc_color );

    float* datos_para_debug(int* num_datos, std_msgs::Float32MultiArray locArray, float dist_sensado_ll, float dist_sensado_cc, float dist_sensado_rr, int U, state_car delta_odom, state_car delta_predict, state_car pos_odom, state_car pos_predict, double delta_t, bool bandera_primer);

};
