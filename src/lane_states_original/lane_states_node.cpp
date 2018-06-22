#include "lane_states_node.h"

#define PAINT_OUTPUT
#define PUBLISH_DEBUG_OUTPUT

int num_execution = 0;
double pix_width_mts = 0;

clane_states::clane_states(ros::NodeHandle nh)
    : nh_(nh), priv_nh_("~")
{

    std::string node_name = ros::this_node::getName();
    priv_nh_.param<int>(node_name+"/car_center", car_center, 80);
    priv_nh_.param<int>(node_name+"/image_height", image_height, 160);
    priv_nh_.param<int>(node_name+"/state_width_pixels", state_width_pix, 16);
    priv_nh_.param<int>(node_name+"/model_num_gazebo", model_num_gazebo, 1);
    priv_nh_.param<double>(node_name+"/pix_width_mts", pix_width_mts, 0.00051);
    priv_nh_.param<int>(node_name+"/num_execution", num_execution, 0);

    const char * format = "P(x)=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n";

    // iniciar variables de integracion IMU
    for(int i = 0; i < HISTORY_IMU; i++) {
        velocityy[i] = 0.0;
        positiony[i] = 0.0;
        accy[i] = 0.0;

        velocityx[i] = 0.0;
        positionx[i] = 0.0;
        accx[i] = 0.0;
    }

    for(int i = HISTORY_POS; i >= 0; i--){
        global_position_x [i] = 0;
        global_position_y [i] = 0;
        global_orientation [i] = 0;
    }

    pub_loc = nh_.advertise<std_msgs::Float32MultiArray>("/localization_array", MY_ROS_QUEUE_SIZE);
    pub_image = nh_.advertise<sensor_msgs::Image>("/histogram_states", MY_ROS_QUEUE_SIZE);

    sub_ransac_left = nh_.subscribe("/points/ransac_left", MY_ROS_QUEUE_SIZE, &clane_states::get_ransac_left, this);
    sub_ransac_center = nh_.subscribe("/points/ransac_center", MY_ROS_QUEUE_SIZE, &clane_states::get_ransac_center, this);
    sub_ransac_right = nh_.subscribe("/points/ransac_right", MY_ROS_QUEUE_SIZE, &clane_states::get_ransac_right, this);

    sub_pts_left = nh_.subscribe("/points/left", MY_ROS_QUEUE_SIZE, &clane_states::get_pts_left, this);
    sub_pts_center = nh_.subscribe("/points/center", MY_ROS_QUEUE_SIZE, &clane_states::get_pts_center, this);
    sub_pts_right = nh_.subscribe("/points/right", MY_ROS_QUEUE_SIZE, &clane_states::get_pts_right, this);

    sub_des_state = nh_.subscribe("/desired_state", MY_ROS_QUEUE_SIZE, &clane_states::get_ctrl_desired_state, this);
    // no es confiable velocidad y steering
    sub_mov = nh_.subscribe("/standarized_vel_ste", MY_ROS_QUEUE_SIZE, &clane_states::get_ctrl_action, this);
    // probar con imu
    sub_imu = nh_.subscribe("/AutoNOMOS_mini/imu", MY_ROS_QUEUE_SIZE, &clane_states::get_imu, this);
    // ros::Subscriber sub_orientation = nh_.subscribe("/car_orientation", MY_ROS_QUEUE_SIZE, &get_car_orientation);      

    sub_robot_pos = nh_.subscribe("/AutoNOMOS_mini/real_pose_from_gazebo", MY_ROS_QUEUE_SIZE, &clane_states::global_pose_callback, this);



    // publicar imagen con la distribucion de sense, hits y move para debug
#ifdef PAINT_OUTPUT

    img_hist = cv::Mat(260, STATE_WIDTH * NUM_STATES, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int i = 0; i < NUM_STATES; i++) {
            cv::line(img_hist, cv::Point(i*STATE_WIDTH, 0), cv::Point(i*STATE_WIDTH, 260), cv::Scalar(255, 0, 0), 1, CV_AA);
            cv::putText(img_hist, nombre_estado[i], cv::Point(i*STATE_WIDTH + STATE_WIDTH/4, 10), 0, .35, cv::Scalar(0,255,237));
    }

    cv::putText(img_hist, "hits", cv::Point(0, 160), 0, .35, cv::Scalar(0,221,237));
    cv::putText(img_hist, "sense", cv::Point(0, 138), 0, .35, cv::Scalar(0,221,237));
    cv::putText(img_hist, "move", cv::Point(0, 260), 0, .35, cv::Scalar(0,221,237));

#endif

#ifdef PUBLISH_DEBUG_OUTPUT
    write_to_file_headers();
#endif



    // ackerman model_ack(0.32, 0.02);
    des_state = 5;

    ctrl_estado = 0;
    dist_x_steering = 0;
    pix_x_prob = 0;
    pixeles_cambio_estado=0;
    odom_x = 0;
    odom_y = 0;
    odom_theta = 0;

    odom_pos_y_old = 0;

    actual_speed_rpm = 0;
    actual_steering = 0;

    R = 0;
    C = 0;
    L = 0;

}

//gets the left points
void clane_states::get_pts_left(const nav_msgs::GridCells& array) {
    arr_left.cells = array.cells;
    if (array.cell_width > 5 && array.cells[0].x > 0) {
            L = array.cell_width;
    }
    else {
            L=0;
    }
}

//gets the center points
void clane_states::get_pts_center(const nav_msgs::GridCells& array) {
    arr_center.cells = array.cells;
    if (array.cell_width > 5 && array.cells[0].x > 0) {
            C = array.cell_width;
    }
    else {
            C=0;
    }
}

//gets the right points
void clane_states::get_pts_right(const nav_msgs::GridCells& array) {
    arr_right.cells = array.cells;
    if (array.cell_width > 5 && array.cells[0].x > 0) {
            R = array.cell_width;
    }
    else {
            R=0;
    }
}

//gets the left points
// 3 because that's the supporters for a newton polynomial
void clane_states::get_ransac_left(const nav_msgs::GridCells& poly) {
    poly_left.clear();
    if (poly.cell_width == 3 && poly.cells[0].x > 0) {
        polyDetectedLeft = true;
        for(int i = 0; i < poly.cell_width; i++) {
            poly_left.addData(poly.cells[i].x, poly.cells[i].y);
        }
        // printf("left detectado");
    }
    else {
        // printf("\n left no detectado, cells: %.2f", poly.cell_width);
        polyDetectedLeft = false;
    }
}

//gets the center points
// 3 because that's the supporters for a newton polynomial
void clane_states::get_ransac_center(const nav_msgs::GridCells& poly) {
    poly_center.clear();
    if (poly.cell_width == 3 && poly.cells[0].x > 0) {
        polyDetectedCenter = true;
        for(int i = 0; i < poly.cell_width; i++) {
            poly_center.addData(poly.cells[i].x, poly.cells[i].y);
        }
        // printf("center detectado");
    }
    else {
        // printf("\n center no detectado, cells: %.2f", poly.cell_width);
        polyDetectedCenter = false;
    }
}

//gets the right points
// 3 because that's the supporters for a newton polynomial
void clane_states::get_ransac_right(const nav_msgs::GridCells& poly) {
    poly_right.clear();
    if (poly.cell_width == 3 && poly.cells[0].x > 0) {
        polyDetectedRight = true;
        for(int i = 0; i < poly.cell_width; i++) {
            poly_right.addData(poly.cells[i].x, poly.cells[i].y);
        }
        // printf("right detectado");
    }
    else {
        // printf("\n right no detectado, cells: %.2f", poly.cell_width);
        polyDetectedRight = false;
    }
}

/* reads actual_speed and steering from standarized topic */
void clane_states::get_ctrl_action(const geometry_msgs::Twist& val) {
    // negative actual_speed is forward
    // it is not required to know if the car is moving forward or backward, positive for simplification
    actual_steering = val.angular.z;
    actual_speed_rpm = val.linear.x;
}

void clane_states::get_ctrl_desired_state(const std_msgs::Int16& val) {
    ctrl_estado = val.data;
}

// get global orientation
void clane_states::get_car_orientation(const std_msgs::Float32& val) {
    car_orientation = val.data;
}

// get global pose of car, Gazebo ONLY

void clane_states::global_pose_callback(const geometry_msgs::Pose2D& global_pose){
    // msg.name
    // TODO search by car name

    global_position_x [0] = global_pose.x;
    global_position_y [0] = global_pose.y;
    global_orientation [0] = global_pose.theta;
}


// TODO: Dead reckogning
void clane_states::get_imu(const sensor_msgs::Imu& val){

    /*
    std_msgs/Header header
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance
    */

    // linear_acc_x = val.linear_acceleration.x;
    // linear_acc_y = val.linear_acceleration.y;
    // linear_acc_z = val.linear_acceleration.z;

    accy [0] = accy [1];
    accy [1] = val.linear_acceleration.y;

    velocityy [0] = velocityy [1];
    velocityy [1] += (accy [1] + (accy [1] - accy [0]) / 2) * T_imu;
    // second integration
    // pos_x.append(pos_x[i - 1] + ( vel_x[i] + (vel_x[i] - vel_x[i - 1]) / 2 ) * T)
    positiony [0] = positiony [1];
    positiony [1] += (velocityy [1] + (velocityy [1] - velocityy [0]) / 2) * T_imu;

    accx [0] = accx [1];
    accx [1] = val.linear_acceleration.x;

    velocityx [0] = velocityx [1];
    velocityx [1] += (accx [1] + (accx [1] - accx [0]) / 2) * T_imu;
    // second int pos
    positionx [0] = positionx [1];
    positionx [1] += (velocityx [1] + (velocityx [1] - velocityx [0]) / 2) * T_imu;

}


// Calculates the distance in pixels. NOTE: only using th x component because Y is asumed constant, 
// maybe isnt the best way to have it.
// If y is asumed constant ==> using abs() instead of sqrt might be more efficient
float clane_states::horizontal_dist(geometry_msgs::Point p1, geometry_msgs::Point p2) {
    return abs(p1.x - p2.x);
}

//determines a hit based on the number of lines detected and the distance from the car center to the lines
//to determine lines detected, the following table is used:
//   | 	L  |  C  |  R
//===================== 
// 0 |  0  |  0  |  0
// 1 |  0  |  0  |  1
// 2 |  0  |  1  |  0
// 3 |  0  |  1  |  1
// 4 |  1  |  0  |  0
// 5 |  1  |  0  |  1
// 6 |  1  |  1  |  0
// 7 |  1  |  1  |  1
//
//The states are:
//		 |     |     |			
//		 |           |			OL -> Out left
//		 |     |     |			LL -> Left Left
//		 |           |			LC -> Left Center
//		 |	   |     |			CC -> Center Center
//		 |           |			RC -> Right Center
//		 |     |     |			RR -> Right Right
//   OL LL LC CC RC RR OR		OR -> Out Right
//   0	1  2  3  4  5  6 
//	
//
//Determine the probable position of the car according to what is seen
int clane_states::det_hit (int position, int lanes_detected, bool ll, bool cc, bool rr, float dist_sensado_ll, float dist_sensado_cc, float dist_sensado_rr) {
    int state = (int) floor(position / STATE_WIDTH);
    int hit = 0;
    switch (state)
    {
        // con el cambio a dbscan aumenta el sensado y disminuyen el numero de
        // combinaciones de deteccion a:
        // con el problema de que R no siempre estará a la derecha del carro

        //   | 	L  |  C  |  R
        //=====================
        // 0 |  0  |  0  |  0
        // 1 |  0  |  0  |  1
        // 3 |  0  |  1  |  1
        // 7 |  1  |  1  |  1


        case 0: //OL
                // posibles combinaciones:
                // no esta cerca de ninguna linea
                // 1L y el carro está a la izquierda de R
                // 2L y el carro está a la izquierda de C
                // 3L y el carro está a la izquierda de L

                hit = !(ll || cc || rr ) && ((R > 0 && lanes_detected == 1 && car_center < arr_right.cells[0].x) ||
                                             (C > 0 && lanes_detected == 3 && car_center < arr_center.cells[0].x) ||
                                             (L > 0 && lanes_detected == 7 && car_center < arr_left.cells[0].x ));

            break;
        case 1: // LL
                // posibles combinaciones:
                // cerca de L C ó R
                // 1L carro sobre R
                // 2L carro sobre C
                // 3L carro sobre L

                hit = (ll || cc || rr ) && ((lanes_detected == 1 && rr) ||
                                            (lanes_detected == 3 && cc) ||
                                            (lanes_detected == 7 && ll));
		/*
                if (hit) {
                    double state_center = (state * STATE_WIDTH + (state+1) * STATE_WIDTH) / 2;
                    // el supuesto es que solo ve una linea: right or center
                    double dist_car_in_state = dist_sensado_rr < dist_sensado_cc ? dist_sensado_rr : dist_sensado_cc;
                    dist_car_in_state = dist_car_in_state < dist_sensado_ll ? dist_car_in_state : dist_sensado_ll;

                    if ((position < state_center - dist_car_in_state - RADIO / 2 || position >  state_center - dist_car_in_state + RADIO / 2) && (position < state_center + dist_car_in_state - RADIO / 2 || position >  state_center + dist_car_in_state + RADIO / 2))
                        hit = !hit;
                }
		*/
                break;

                // FALTA APROVECHAR INFORMACION DE SENSADO

        case 2: //LC
                // posibles combinaciones:
                // no esta cerca de ninguna linea
                // 2L carro entre C y R
                // 3L carro entre L y C

                // hit = !(rr || cc ) && (lanes_detected == 3);

                 hit = !(ll || cc || rr) && ((R > 0 && lanes_detected == 1 && car_center > arr_right.cells[0].x) ||
                                             (C > 0 && car_center < arr_center.cells[0].x && lanes_detected == 3) ||
                                             (C > 0 && lanes_detected == 3 && car_center > arr_center.cells[0].x && car_center < arr_right.cells[0].x) ||
                                             (C > 0 && lanes_detected == 7 && car_center > arr_left.cells[0].x && car_center < arr_center.cells[0].x));

		/*
                 if (lanes_detected > 1) {
                     if (hit) {
                        double min_coord = state * STATE_WIDTH;
                        double max_coord = (state + 1) * STATE_WIDTH;
                        double temp_min, temp_max;


                        if ( car_center > arr_center.cells[0].x ) {
                            // pt_c debe estar a la izquierda, en este caso se cumple
                            if (dist_sensado_cc > 0 && dist_sensado_cc < 1000)
                                temp_min = min_coord + dist_sensado_cc - STATE_WIDTH / 2 - RADIO;
                            // pt_r debe estar a la derecha, en este caso se cumple
                            if (dist_sensado_rr > 0 && dist_sensado_rr < 1000)
                                temp_max = max_coord + -dist_sensado_rr + STATE_WIDTH / 2 + RADIO;
                        } else {
                            // pt_c debe estar a la izquierda, en este caso se cumple
                            if (dist_sensado_cc > 0 && dist_sensado_cc < 1000)
                                temp_min = min_coord + dist_sensado_ll - STATE_WIDTH / 2 - RADIO;
                            // pt_r debe estar a la derecha, en este caso se cumple
                            if (dist_sensado_rr > 0 && dist_sensado_rr < 1000)
                                temp_max = max_coord + -dist_sensado_cc + STATE_WIDTH / 2 + RADIO;
                        }
                        if (temp_min > min_coord) {
                            if (temp_min > max_coord - RADIO)
                                min_coord =  max_coord - RADIO;
                            else
                                min_coord = temp_min;
                        }
                        if (temp_max < max_coord) {
                            if (temp_max < min_coord + RADIO)
                                max_coord = min_coord + RADIO;
                            else
                                max_coord = temp_max;
                        }


                        if (position < min_coord || position > max_coord)
                                hit = !hit;
                    }
                }
		*/

                break;
        case 3: //CC
                // posibles combinaciones:
                // cerca de C R
                // 2L carro sobre C o R
                // 3L carro sobre C
                hit = ( cc || rr ) && ((lanes_detected == 3) ||
                                       (lanes_detected == 7 && cc));
		/*
                if (hit) {
                    double state_center = (state*STATE_WIDTH + (state+1)*STATE_WIDTH) / 2;
                    // el supuesto es que solo ve una linea: right or center
                    double dist_car_in_state = dist_sensado_rr < dist_sensado_cc ? dist_sensado_rr : dist_sensado_cc;

                    if ((position < state_center - dist_car_in_state - RADIO / 2 ||
                        position >  state_center - dist_car_in_state + RADIO / 2) &&
                        (position < state_center + dist_car_in_state - RADIO / 2 ||
                        position >  state_center + dist_car_in_state + RADIO / 2))
                                hit = !hit;
                }
		*/
                break;
        case 4: // RC
                // posibles combinaciones:
                // no esta cerca de ninguna linea
                // 2L carro entre C y R
                // 3L carro entre C y R

                hit = !(ll || cc || rr) &&
                            ((R > 0 && car_center < arr_right.cells[0].x && lanes_detected == 1) ||
                            (C > 0 && car_center > arr_right.cells[0].x && lanes_detected == 3) ||
                            (C > 0 && car_center > arr_center.cells[0].x && car_center < arr_right.cells[0].x && (lanes_detected == 3 || lanes_detected == 7)));
		/*
                if (lanes_detected > 1) {
                    if (hit) {
                        double min_coord = state * STATE_WIDTH;
                        double max_coord = (state + 1) * STATE_WIDTH;
                        double temp;

                        // pt_c debe estar a la izquierda, en este caso se cumple
                        if (dist_sensado_cc > 0 && dist_sensado_cc < 1000) {
                            temp = min_coord + dist_sensado_cc - STATE_WIDTH / 2 - RADIO;
                            if (temp > min_coord) {
                                if (temp > max_coord - RADIO)
                                    min_coord =  max_coord - RADIO;
                                else
                                    min_coord = temp;
                            }
                        }

                        // pt_r debe estar a la derecha, en este caso se cumple
                        if (dist_sensado_rr > 0 && dist_sensado_rr < 1000) {
                            temp = max_coord + -dist_sensado_rr + STATE_WIDTH / 2 + RADIO;
                            if (temp < max_coord) {
                                if (temp < min_coord + RADIO)
                                    max_coord = min_coord + RADIO;
                                else
                                    max_coord = temp;
                            }
                        }

                        if (position < min_coord || position > max_coord)
                                hit = !hit;
                    }
                }
		*/
                break;
        case 5: //RR
                // posibles combinaciones:
                // cerca de R
                // 1L carro sobre R
                // 2L carro sobre R
                // 3L carro sobre R

                hit = rr &&
                        (lanes_detected == 1 || lanes_detected == 3 || lanes_detected == 7);
		/*
                if (hit) {
                    if (hit) {
                        double state_center = (state * STATE_WIDTH + (state + 1) * STATE_WIDTH) / 2;
                        // el supuesto es que solo ve una linea: right or center
                        double dist_car_in_state = dist_sensado_rr;

                    if ((position < state_center - dist_car_in_state - RADIO / 2 || position >  state_center - dist_car_in_state + RADIO / 2) && (position < state_center + dist_car_in_state - RADIO / 2 || position >  state_center + dist_car_in_state + RADIO / 2))
                        hit = !hit;
                    }
                }
		*/
                break;
        case 6: //OR
                // posibles combinaciones:
                // no esta cerca de ninguna linea
                // 1L carro a la derecha de R
                // 2L carro a la derecha de R
                // 3L carro a la derecha de R

                hit = !(ll || cc || rr ) &&
                   R > 0 && car_center > arr_right.cells[0].x &&
                        (lanes_detected == 1 || lanes_detected == 3 || lanes_detected == 7 );

                break;
    }

    return hit;
}

float* clane_states::det_hits(float *dist_sensado_ll, float *dist_sensado_cc, float *dist_sensado_rr, int *lanes_detected) {
    // TODO: Hay una idea pendiente, la cual es que el sensado nos puede dar información de los estados
    // aunque no estemos en ese estado, si podemos saber que nos estamos acercado
    // p.ej. estamos en RC y nos movemos a la izquierda sabemos que estamos en los limites de CC

    // sensor
    float p_hit = 0.99;
    float p_miss = 0.01;

    *lanes_detected = (L > 0);
    *lanes_detected = *lanes_detected << 1;
    *lanes_detected = *lanes_detected | C > 0;
    *lanes_detected = *lanes_detected << 1;
    *lanes_detected = *lanes_detected | R > 0;

    geometry_msgs::Point pt_r;
    geometry_msgs::Point pt_c;
    geometry_msgs::Point pt_l;
    geometry_msgs::Point pt_car;

    //define the static point (center) of the car
    pt_car.x = car_center;
    pt_car.y = 0;
    pt_car.z = 0;

    // first points are closer to the car using ransac
    if ( R > 0 ) pt_r = arr_right.cells[0];
    if ( C > 0 ) pt_c = arr_center.cells[0];
    if ( L > 0 ) pt_l = arr_left.cells[0];

    double px_sobre_cm = 0.65;
    // if there are points in the lines, get the distance between the car and the closest point if not, assign a BIG number
    *dist_sensado_rr = R > 0 ? horizontal_dist(pt_r, pt_car) / px_sobre_cm : 1000;
    *dist_sensado_cc = C > 0 ? horizontal_dist(pt_c, pt_car) / px_sobre_cm : 1000;
    *dist_sensado_ll = L > 0 ? horizontal_dist(pt_l, pt_car) / px_sobre_cm : 1000; // / px_sobre_cm

    // define if each distance is smaller than dist_lines
    bool sense_rr = *dist_sensado_rr < STATE_WIDTH / 2;
    bool sense_cc = *dist_sensado_cc < STATE_WIDTH / 2;
    bool sense_ll = *dist_sensado_ll < STATE_WIDTH / 2;

    // based on sensing update probabilities
    float* hits = new float[NUM_STATES*STATE_WIDTH];
    for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {
            unsigned int hit = det_hit(i, *lanes_detected, sense_ll, sense_cc, sense_rr, *dist_sensado_ll, *dist_sensado_cc, *dist_sensado_rr);
            hits[i] = (hit * p_hit + (1-hit) * p_miss);
    }

    return hits;
}

std_msgs::Float32MultiArray clane_states::sense(std_msgs::Float32MultiArray p, float* hits) {

    std_msgs::Float32MultiArray q;
    for (int i = 0; i < NUM_STATES * STATE_WIDTH; ++i) {
            q.data.push_back(p.data[i] * hits[i]);
    }

    // normalizacion
    float sum = 0;
    for (int i = 0; i < NUM_STATES * STATE_WIDTH; ++i) {
            sum += q.data[i];
    }
    for (int i = 0; i < NUM_STATES * STATE_WIDTH; ++i) {
            q.data[i] /= sum;
    }
    return q;
}

std_msgs::Float32MultiArray clane_states::move(std_msgs::Float32MultiArray prob, double delta_pos_y, int *U) {

    // movement
    float p_exact = 0.7;
    float p_undershoot = 0.1;
    float p_undershoot_2 = 0.05;
    float p_overshoot = 0.1;
    float p_overshoot_2 = 0.05;

    // tamaño de celda en pixeles: 0.006879221 revisar por actualización de camino
    // negativo por la rotacion de cuadro de referencia con respecto al carro
        // utilizando posicion global
        // dist_x_steering = (global_position_x[1] - global_position_x[0]); // cuantos pixeles se desplaza por m/s
        // utilizando posicion derivada de modelo de ackerman

    double cm_x_prob = delta_pos_y / pix_width_mts; // * 100; // * 100; // para cambiar de metros a centimetros

    // int
    *U = round( cm_x_prob ); // pixels

    std_msgs::Float32MultiArray q;
    // ROS_INFO_STREAM( "Orientation: " << car_orientation << ", Control: " << actual_steering <<  ", rate_velocidad: " << dist_x_steering << ", U: " << U);

    for (int i = 0; i < NUM_STATES*STATE_WIDTH; i++) {
        double s = 0.0;

        //HISTOGRAMA CICLICO
        //EXACT
        int mov = i + *U;
        int mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
        if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
        s = p_exact * prob.data[mod2];

        //HISTOGRAMA CICLICLO
        //UNDERSHOOT
        mov = i + *U - 1;
        mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
        if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
        s += p_undershoot * prob.data[mod2];

        // UNDERSHOOT 2
        mov = i + *U - 2;
        mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
        if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
        s += p_undershoot_2 * prob.data[mod2];

        //HISTOGRAMA CICLICLO
        //OVERSHOOT
        mov = i + *U + 1;
        mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
        if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
        s += p_overshoot * prob.data[mod2];

        //OVERSHOOT 2
        mov = i + *U + 2;
        mod2 = (mov) % (NUM_STATES*STATE_WIDTH);
        if (mod2<0) mod2 = NUM_STATES*STATE_WIDTH+mod2;
        s += p_overshoot_2 * prob.data[mod2];

        q.data.push_back(s);
    }
    return q;
}

int clane_states::actual_state(std_msgs::Float32MultiArray locArray) {
    float max=0;
    for (int i=0;i<NUM_STATES*STATE_WIDTH;i++) {
            if(locArray.data[i]>max){
                    max=locArray.data[i];
            }
    }

    int countStates=0;
    int state = -1;
    for (int i=NUM_STATES*STATE_WIDTH-1;i>=0;i--) {
        if(locArray.data[i]==max){
            int temp_state = (int)floor(i/STATE_WIDTH);
            if (temp_state != state){
                state = temp_state;
                countStates++;
            }
        }
    }

    if (countStates==1)
    	return state;
    else
    	return -1; // no se pudo determinar el estado, ya que hay mas de uno posible
}



void clane_states::write_to_file(std_msgs::Float32MultiArray m_array, float* p, int values) {
    // debug behavior of probabilities using file
    // FILE *f = fopen("~/git/AutoNOMOS/src/histogramfilter.txt", "a");

    printf("%d", num_execution);

    for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i) {
            printf("\t%.2f ", m_array.data[i]);
    }
    for (int i = 0; i < values; ++i) {
            printf("\t%.5f ", p[i]);
    }

    printf("\n");

    // fclose(f);
}

void clane_states::write_to_image( cv::Mat& imagen, float* hits, std_msgs::Float32MultiArray sense, std_msgs::Float32MultiArray move, int values, int borrarSenseImagen, int *inc_color ) {
    // debug behavior of probabilities using rqt
    int hist_height = 100;

    for (int i = 0; i < values; ++i) {
            // SENSE
            int y_height = 130;
            if (borrarSenseImagen == 0 && i < values - 1)
                    cv::line(imagen, cv::Point(i, y_height), cv::Point(i + 1, y_height - hist_height), cv::Scalar(0, 0, 0), 1, CV_AA);
            cv::line(imagen, cv::Point(i, y_height), cv::Point(i, y_height - sense.data[i] * hist_height), cv::Scalar(0, *inc_color, 255 - *inc_color), 1, CV_AA);

            // HITS
            y_height = 150;
            cv::line(imagen, cv::Point(i, y_height), cv::Point(i, y_height - hits[i] * 10), cv::Scalar(0, 100, 255), 1, CV_AA);
            // borrar hit posicion i + 1 en imagen
            if (i < values - 1)
                    cv::line(imagen, cv::Point(i + 1, y_height), cv::Point(i + 1, y_height - 10), cv::Scalar(0, 0, 0), 1, CV_AA);

            // MOVE
            y_height = 250;
            cv::line(imagen, cv::Point(i, y_height - sense.data[i] * hist_height), cv::Point(i, y_height - move.data[i] * hist_height), cv::Scalar(0, 200, 255), 1, CV_AA);
            // borrar movimiento posicion i + 1 en imagen
            if (i < values - 1)
                    cv::line(imagen, cv::Point(i + 1, y_height), cv::Point(i + 1, y_height - hist_height + 15), cv::Scalar(0, 0, 0), 1, CV_AA);
    }

    *inc_color = (*inc_color + 7) % 255;
}

double clane_states::cte(double global_pose_x, double global_pose_y){
    // calcular cross track error en circuito de curva
    double cte = 0;
    // izquierda arriba
    double robot_x = global_pose_x;
    double robot_y = global_pose_y;
    double radius = 1.3;
    double pista_x;
    double pista_y;
    double alpha;

    if (robot_y > 4 && robot_x < 2) {
        // angulo al origen
        alpha = atan2(robot_y - 4, robot_x - 2);
        pista_x = radius * cos(alpha);
        pista_y = radius * sin(alpha);

        // printf ("\n 1. alpha {} ({}, {}) ".format(alpha, pista_x, pista_y))

        // con traslacion
        pista_x += 2;
        pista_y += 4;

        cte = sqrt(pow(pista_x - robot_x, 2) + pow(pista_y - robot_y, 2));

        if (robot_x > pista_x)
            cte = -cte;
    }
    // derecha arriba
    else if (robot_y > 4 && robot_x > 7) {
        alpha = atan2(robot_y - 4, robot_x - 7);
        pista_x = radius * cos(alpha);
        pista_y = radius * sin(alpha);

        // printf ("\n 2. alpha {} ({}, {}) ".format(alpha, pista_x, pista_y))

        pista_x += 7;
        pista_y += 4;

        cte = sqrt(pow(pista_x - robot_x, 2) + pow(pista_y - robot_y, 2));

        if (robot_x < pista_x)
            cte = -cte;
    }
    // izquierda abajo
    else if (robot_y < -4 && robot_x < 2){
        alpha = atan2(robot_y + 4, robot_x - 2);
        pista_x = radius * cos(alpha);
        pista_y = radius * sin(alpha);

        pista_x += 2;
        pista_y += -4;

        cte = sqrt(pow(pista_x - robot_x, 2) + pow(pista_y - robot_y, 2));

        if (robot_x > pista_x)
            cte = -cte;
    }
    // derecha abajo
    else if (robot_y < -4 && robot_x > 7) {
        alpha = atan2(robot_y + 4, robot_x - 7);
        pista_x = radius * cos(alpha);
        pista_y = radius * sin(alpha);

        pista_x += 7;
        pista_y += -4;

        cte = sqrt(pow(pista_x - robot_x, 2) + pow(pista_y - robot_y, 2));

        if (robot_x < pista_x)
            cte = -cte;
    }
    // recta izquierda
    else if (robot_y >= -4 && robot_y <= 4 && robot_x < 2) {
        cte = robot_x - .7;
        // printf ("\n 3. {}".format(cte))
    }
    // recta derecha
    else if (robot_y >= -4 && robot_y <= 4 && robot_x > 7) {
        cte = robot_x - 8.3;
        //printf ("\n 3. {}".format(cte))
    }
    // recta arriba
    else if (robot_y >= 4 && robot_x >= 2 && robot_x <= 7) {
        cte = robot_y - 5.3;
        // printf ("\n 3. {}".format(cte))
    }
    // recta abajo
    else {
        cte = robot_y + 5.3;
        // printf ("\n 4. {}".format(cte))
    }

    return cte;

}

void clane_states::write_to_file_headers() {
    // debug behavior of probabilities using file
    // FILE *f = fopen("~/git/AutoNOMOS/images/histogramfilter.txt", "w");

    printf("num_execution");

    for (int i = 0; i < NUM_STATES * STATE_WIDTH; ++i) {
            printf("\tEst_%d ", i);
    }

    printf("\tL"); // 0
    printf("\tC");
    printf("\tR");
    printf("\td_ll");
    printf("\td_cc");
    printf("\td_rr");
    printf("\tactual_speed");
    printf("\tactual_steering"); // 7

    printf("\testimated_state");
    printf("\tU"); // 9
    printf("\tdesired_state");

    printf("\tglobal_x");
    printf("\tglobal_y"); // 12
    printf("\tglobal_theta");

    printf("\tpredicted_x");
    printf("\tpredicted_y");
    printf("\tpredicted_theta");

    printf("\tdelta_predicted_x");
    printf("\tdelta_predicted_y");
    printf("\tdelta_predicted_theta");

    printf("\tpos_odom_x"); // 20
    printf("\tpos_odom_y");
    printf("\tpos_odom_theta");

    printf("\tdelta_odom_x");
    printf("\tdelta_odom_y");
    printf("\tdelta_odom_theta");

    printf("\tdelta_time");
    printf("\tcte");

    printf("\n");
    // fclose(f);
}

float* clane_states::datos_para_debug(int* num_datos, std_msgs::Float32MultiArray locArray, float dist_sensado_ll, float dist_sensado_cc, float dist_sensado_rr, int U, state_car delta_odom, state_car delta_predict, state_car pos_odom, state_car pos_predict, double delta_t, bool bandera_primer) {

    int estadoEstimado = actual_state(locArray);

    *num_datos = 28;
    float *datos = new float[*num_datos];

    datos[0] = (float) polyDetectedLeft;
    datos[1] = (float) polyDetectedCenter;
    datos[2] = (float) polyDetectedRight;
    datos[3] = dist_sensado_ll;
    datos[4] = dist_sensado_cc;
    datos[5] = dist_sensado_rr;
    datos[6] = actual_speed_rpm;
    datos[7] = actual_steering;

    datos[8] = (float) estadoEstimado;
    datos[9] = U;
    datos[10] = estadoEstimado + ctrl_estado;

    datos[11] = global_position_x [0];
    datos[12] = global_position_y [0];
    datos[13] = global_orientation [0];

    datos[14] = pos_predict.x;
    datos[15] = pos_predict.y; // direccion
    datos[16] = pos_predict.theta;

    datos[17] = delta_predict.x;
    datos[18] = delta_predict.y;
    datos[19] = delta_predict.theta;

    datos[20] = pos_odom.x;
    datos[21] = pos_odom.y;
    datos[22] = pos_odom.theta;

    datos[23] = delta_odom.x;
    datos[24] = delta_odom.y;
    datos[25] = delta_odom.theta;

    datos[26] = delta_t;
    datos[27] = cte(global_position_x [0], global_position_y [0]);

    return datos;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "lane_states_node");
    // ROS_INFO_STREAM("lane_states_node initialized");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE_HZ);

    sensor_msgs::ImagePtr imgmsg;

    double delta_t_ = 0;
    double prev_ = 0;
    int U;
    float* z_t;

    int num_datos = 0;
    float* datos ;

    float dist_sensado_ll;
    float dist_sensado_cc;
    float dist_sensado_rr;

    int lanes_detected;
    int inc_color = 0;

    int borrarSenseImagen = 0;
    bool bandera_primer = true;

    clane_states node = clane_states(nh);
    ackerman model_ref = ackerman(0.32, 0.025);

    std_msgs::Float32MultiArray belief;


    // Iniciar con distribucion uniforme
    for (int i = 0; i < NUM_STATES*STATE_WIDTH; ++i)
    {
        belief.data.push_back((float)(1/(float)(NUM_STATES*STATE_WIDTH)));
    }

    state_car delta_odom, delta_predict;
    // conversión en el plugin
    double factor_conversion_simulador = -(15.0 / 31.0) * (1.0 / 12.0);

    while(ros::ok()) {

        ros::spinOnce();
        if (prev_ == 0)
            delta_t_ = 0.2;
          else
            delta_t_ = ros::Time::now().toSec() - prev_;
        prev_ = ros::Time::now().toSec();

        double vel_rad_seg = node.actual_speed_rpm * factor_conversion_simulador;  //* (PI / 30);  // 1 revolution/ min = 2π / 60 seconds = π/ 30  radians per sec
        // printf("\n vel: %.6f, vel_rad: %.6f, factor: %.6f", node.actual_speed_rpm, vel_rad_seg, factor_conversion_simulador);

        delta_odom = model_ref.update_odometry(vel_rad_seg, node.actual_steering, delta_t_);
        delta_predict = model_ref.predict_deltas(vel_rad_seg, node.actual_steering, delta_t_);

        std_msgs::Float32MultiArray belief_hat = node.move(belief, delta_predict.y, &U);

        z_t = node.det_hits(&dist_sensado_ll, &dist_sensado_cc, &dist_sensado_rr, &lanes_detected);

        // if(lanes_detected > 0) {
            belief = node.sense(belief_hat, z_t);
        // } else {
            // mantain previous probabilities if no lines detected
        //    belief = belief_hat;
        // }

        node.pub_loc.publish(belief);
#ifdef PUBLISH_DEBUG_OUTPUT
        datos = node.datos_para_debug(&num_datos, belief, dist_sensado_ll, dist_sensado_cc, dist_sensado_rr, U, delta_odom, delta_predict, model_ref.pos_odom, model_ref.pos_predict, delta_t_, bandera_primer);
        node.write_to_file(belief, datos, num_datos);
#endif

#ifdef PAINT_OUTPUT
        node.write_to_image(node.img_hist, z_t, belief, belief_hat, STATE_WIDTH * NUM_STATES, borrarSenseImagen, &inc_color);
        // borrarSenseImagen = ++borrarSenseImagen % STATE_WIDTH; // borrar cada cierto numero de iteraciones
//	 cv::imshow("Localization results", img_hist);
//       cv::waitKey(1);
        imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", node.img_hist).toImageMsg();
        node.pub_image.publish(imgmsg);
#endif

        loop_rate.sleep();

    }

    free(z_t);
    free(datos);

    return 0;
}
