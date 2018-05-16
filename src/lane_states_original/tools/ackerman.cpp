#include "ackerman.h"

ackerman::ackerman(float wheelBase, float wheelRadius) {
    this->wheelBase = wheelBase;
    this->wheelRadius = wheelRadius;
    this->v_x = 0.0;
    this->v_y = 0.0;
    this->v_theta = 0.0;
    this->pos_dx = 0.0;
    this->pos_dy = 0.0;
    this->pos_dtheta = 0.0;
}

void ackerman::UpdateParameters(float vel_rad, float steering, float delta_time) {
    double vel_mts_seg = vel_rad * this->wheelRadius; // rad / s * Perimeter / 2 * PI * rad

    this->v_x_old = this->v_x;
    this->v_y_old = this->v_y;
    this->v_theta_old = this->v_theta;

    this->v_x = vel_mts_seg * cos(this->pos_theta);
    this->v_y = vel_mts_seg * sin(this->pos_theta);
    this->v_theta = (vel_mts_seg / this->wheelBase) * tan(steering);

    // integracion trapezoidal
    // velocityy [1] = velocityy [0] + (accy [1] + (accy [1] - accy [0]) / 2) * T_imu;
    // para estimacion global sumar posicion anterior
    // pos_x = (v_x + (v_x - v_x_old) / 2) * delta_time;

    // integracion rectangular
    this->pos_dx = (this->v_x + (this->v_x - this->v_x_old) / 2.0) * delta_time;
    this->pos_dy = (this->v_y + (this->v_y - this->v_y_old) / 2.0) * delta_time;
    this->pos_dtheta = (this->v_theta + (this->v_theta - this->v_theta_old) / 2.0) * delta_time;

    this->pos_x += this->pos_dx;
    this->pos_y += this->pos_dy;
    this->pos_theta += this->pos_dtheta;
}
