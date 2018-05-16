#ifndef ACKERMAN_H
#define ACKERMAN_H

#include <ros/ros.h>
#include <math.h>
#define PI 3.14159265

class ackerman
{

private:
    double wheelBase;
    double wheelRadius;
    double v_x;
    double v_y;
    double v_theta;

    double v_x_old;
    double v_y_old;
    double v_theta_old;

public:
    double pos_dx;
    double pos_dy;
    double pos_dtheta;

    double pos_x;
    double pos_y;
    double pos_theta;

    ackerman(float wheelBase, float wheelRadius);

    void UpdateParameters(float vel_rad, float steering, float delta_time);

};

#endif // ACKERMAN_H


