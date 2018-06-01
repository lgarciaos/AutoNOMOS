#ifndef ACKERMAN_H
#define ACKERMAN_H

#include <ros/ros.h>
#include <math.h>
#define PI 3.14159265

struct state_car {
    double x;
    double y;
    double theta;
    state_car(): x(0), y(0), theta(0) {}
};

class ackerman
{

private:
    double wheel_base;
    double wheel_radius;

    state_car vel;
    state_car vel_old;

public:

    state_car pos_odom;
    state_car pos_predict;

    ackerman(float wheel_base, float wheel_radius);

    state_car update_odometry(float vel_rad_seg, float steering, float delta_time);

    state_car predict_deltas(float vel_rad_seg, float steering, float delta_time);
};

#endif // ACKERMAN_H


