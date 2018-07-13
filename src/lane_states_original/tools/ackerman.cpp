#include "ackerman.h"

ackerman::ackerman(float wheel_base, float wheel_radius) {

    this->wheel_base = wheel_base;
    this->wheel_radius = wheel_radius;

}

// odometry from velocities with respect to global frame (using global theta)
state_car ackerman::update_odometry(float vel_rad_seg, float steering, float delta_time) {

    state_car delta;

    double vel_mts_seg = vel_rad_seg * wheel_radius; // rad / s * Perimeter  (mts) / 2 * PI (rad)

    vel_old.x = vel.x;
    vel_old.y = vel.y;
    vel_old.theta = vel.theta;

    vel.x = vel_mts_seg * cos(pos_odom.theta);
    vel.y = vel_mts_seg * sin(pos_odom.theta);
    vel.theta = (vel_mts_seg / wheel_base) * tan(steering); // 0 a 2PI

    // calculate deltas
    delta.x = vel.x * delta_time; // (vel.x + (vel.x - vel_old.x) / 2.0) * delta_time;
    delta.y = vel.y * delta_time; // (vel.y + (vel.y - vel_old.y) / 2.0) * delta_time;
    delta.theta = vel.theta * delta_time;

    // update odometry based on global deltas
    pos_odom.x += delta.x;
    pos_odom.y += delta.y;
    pos_odom.theta += delta.theta;

    return delta;
}

// odometry from velocities with respect to local frame (using local theta)
state_car ackerman::predict_deltas(float vel_rad_seg, float steering, float delta_time) {

    double vel_mts_seg = vel_rad_seg * wheel_radius; // (rad / s) * (Perimeter / 2 * PI * rad)

    state_car delta;

    double vel_theta = (vel_mts_seg / wheel_base) * tan(steering); // 0 a 2PI
    delta.theta = vel_theta * delta_time;

    double vel_x = vel_mts_seg * cos(delta.theta);
    double vel_y = vel_mts_seg * sin(delta.theta);

    delta.x = vel_x * delta_time;
    delta.y = vel_y * delta_time;

    // update odometry based on local deltas
    double motion_magnitude = sqrt(pow(delta.x,2) + pow(delta.y, 2));
    pos_predict.x += delta.x * cos(pos_predict.theta) - delta.y * tan(pos_predict.theta) * cos(pos_predict.theta);
    pos_predict.y += motion_magnitude * sin(pos_predict.theta + atan2(delta.y, delta.x));
    pos_predict.theta += delta.theta;

    return delta;

}
