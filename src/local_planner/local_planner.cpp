// --PID pseudocode--
// previouserror = 0
// integral = 0 
// start:
// error = pE - measuredvalue
// integral = integral + error*dt
// derivative = (error - previouserror)/dt
// output = Kp*error + Ki*integral + Kd*derivative
// previouserror = error
// wait(dt)
// goto start
// -------------

// --codeImplementation--
// https://gist.github.com/bradley219/5373998

#include "local_planner.h"

local_planner::local_planner(ros::NodeHandle nh)
    : nh_(nh), priv_nh_("~")
{
    std::string node_name = ros::this_node::getName();
    std::string topico_estandarizado;

    // ROS_INFO_STREAM("Parametros obtenidos");

    priv_nh_.param<double>(node_name+"/Kp", Kp, 0.5);
    priv_nh_.param<double>(node_name+"/Ki", Ki, 0.01);
    priv_nh_.param<double>(node_name+"/Kd", Kd, 0.0);

    priv_nh_.param<int>(node_name+"/car_center", car_center, 80.0);
    priv_nh_.param<int>(node_name+"/car_speed", car_speed, 50.0);
    priv_nh_.param<double>(node_name+"/min_steering", min_steering, -1.5707);
    priv_nh_.param<double>(node_name+"/max_steering", max_steering, 1.5707);

    priv_nh_.param<std::string>(node_name+"/topico_estandarizado", topico_estandarizado, "/standarized_vel_ste");

    priv_nh_.param<int>(node_name+"/image_height", image_height, 160);

    priv_nh_.param<int>(node_name+"/estado_deseado", estado_deseado, 4); // RC

    priv_nh_.param<int>(node_name+"/state_width_pixels", state_width_pix, 16);

    // priv_nh_.param<double>(node_name+"/kalpha", kalpha, 1);
    // priv_nh_.param<double>(node_name+"/kbeta", kbeta, 1);

    // publicar acciones de control a topico estandarizado
    pub_speed_sta = nh_.advertise<geometry_msgs::Twist>(topico_estandarizado, MY_ROS_QUEUE_SIZE);

    pub_image = nh_.advertise<sensor_msgs::Image>("/local_planner", MY_ROS_QUEUE_SIZE);

    // suscribirse localizacion
    // suscribirse ransac
    sub_localization = nh_.subscribe("/localization_array", MY_ROS_QUEUE_SIZE, &local_planner::get_localization, this);
    sub_des_state = nh_.subscribe("/desired_state", MY_ROS_QUEUE_SIZE, &local_planner::get_ctrl_desired_state, this);

    sub_ransac_left = nh_.subscribe("/points/ransac_left", MY_ROS_QUEUE_SIZE, &local_planner::get_ransac_left, this);
    sub_ransac_center = nh_.subscribe("/points/ransac_center", MY_ROS_QUEUE_SIZE, &local_planner::get_ransac_center, this);
    sub_ransac_right = nh_.subscribe("/points/ransac_right", MY_ROS_QUEUE_SIZE, &local_planner::get_ransac_right, this);

    sub_pts_left = nh_.subscribe("/points/left", MY_ROS_QUEUE_SIZE, &local_planner::get_pts_left, this);
    sub_pts_center = nh_.subscribe("/points/center", MY_ROS_QUEUE_SIZE, &local_planner::get_pts_center, this);
    sub_pts_right = nh_.subscribe("/points/right", MY_ROS_QUEUE_SIZE, &local_planner::get_pts_right, this);

    // ros::Subscriber sub_lidar = nh.subscribe("/target_pose", MY_ROS_QUEUE_SIZE, &get_vel_vec);

    poly_left = NewtonPolynomial();
    poly_center = NewtonPolynomial();
    poly_right = NewtonPolynomial();

    integralPID = 0.0;
    prevErrorPID = 0.0;
    estado_actual = -1;

    car_text_position = 150;

    L = 0;
    C = 0;
    R = 0;

// PASAR POLINOMIOS DE RANSAC POR PUNTOS

}

//gets the left points
void local_planner::get_pts_left(const nav_msgs::GridCells& array) {
    arr_left.cells = array.cells;
    arr_left.cell_width = array.cell_width;
    if (array.cell_width > 5 && array.cells[0].x > 0) {
            L = array.cell_width;
    }
    else {
            L=0;
    }
}

//gets the center points
void local_planner::get_pts_center(const nav_msgs::GridCells& array) {
    arr_center.cells = array.cells;
    arr_center.cell_width = array.cell_width;
    if (array.cell_width > 5 && array.cells[0].x > 0) {
            C = array.cell_width;
    }
    else {
            C=0;
    }
}

//gets the right points
void local_planner::get_pts_right(const nav_msgs::GridCells& array) {
    arr_right.cells = array.cells;
    arr_right.cell_width = array.cell_width;
    if (array.cell_width > 5 && array.cells[0].x > 0) {
            R = array.cell_width;
    }
    else {
            R=0;
    }
}

void local_planner::get_ransac_left(const nav_msgs::GridCells& poly){
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

void local_planner::get_ransac_center(const nav_msgs::GridCells& poly){
    poly_center.clear();
    if (poly.cell_width == 3 && poly.cells[0].x > 0) {
        polyDetectedCenter = true;
        for(int i = 0; i < poly.cell_width; i++) {
            poly_center.addData(poly.cells[i].x, poly.cells[i].y);
        }
    }
    else {
        polyDetectedCenter = false;
    }
}

void local_planner::get_ransac_right(const nav_msgs::GridCells& poly){
    poly_right.clear();
    if (poly.cell_width == 3 && poly.cells[0].x > 0) {
        polyDetectedRight = true;
        for(int i = 0; i < poly.cell_width; i++) {
            poly_right.addData(poly.cells[i].x, poly.cells[i].y);
        }
    }
    else {
        polyDetectedRight = false;
    }
}

void local_planner::get_localization(const std_msgs::Float32MultiArray& locArray) {
    int estadoPrevio = estado_actual;

    float max=0;
    for(int i = 0; i < NUM_STATES * STATE_WIDTH; i++) {
        if(locArray.data[i]>max) {
            max=locArray.data[i];
        }
    }

    int countStates=0;
    int state = -1;
    for(int i = NUM_STATES * STATE_WIDTH - 1; i >= 0; i--) {
        if(locArray.data[i]==max) {
            int temp_state = (int)floor(i / STATE_WIDTH);
            if (temp_state != state){
                state = temp_state;
                countStates++;
            }
        }
    }

    if (countStates==1)
        estado_actual = state;
    else
        estado_actual = -1; // no se pudo determinar el estado, ya que hay mas de uno posible
}

void local_planner::get_ctrl_desired_state(const std_msgs::Int16& val) {
    // ctrl_estado = val.data;
    // prevenir un estado deseado fuera de limites
    if (estado_deseado + val.data < 0)
        estado_deseado = 0;
    else if (estado_deseado + val.data > NUM_STATES)
        estado_deseado = NUM_STATES;
    else
        estado_deseado += val.data;
}




/* compute based on distance y_next_dist the points in pixels that the car needs to head to */
bool local_planner::ackerman_control_next_points(double y_next_dist, cv::Point& pt_car, cv::Point& y_next_pt, cv::Point& y_next_pt2,
                                                 NewtonPolynomial& polyLeft, NewtonPolynomial& polyCenter, NewtonPolynomial& polyRight) {

    // ransac
    // int next_move_y = image_height-y_next_dist; // sin el 2* funcionaba bien, checar fuera de limite para polylinea
    // int next_move2_y = image_height-y_next_dist-10;

    // points
    int next_move_y = 0; // sin el 2* funcionaba bien, checar fuera de limite para polylinea
    int next_move2_y = 10;
    int num_suma = 5;

    double x_center = 0.0;
    double x_right = 0.0;
    bool center_closer = false;

    double angle;
    double hip;

    // calcular los puntos a moverse de acuerdo al estado actual y una distancia next_move_y
    // para entender un poco mas por que se utiliza esa linea, checar el mismo switch en det_hit en lane_states_node
    switch(estado_actual) {
        case 0: // OL
            if (L > 0 && pt_car.x < arr_center.cells[next_move_y].x ) {
                // ransac
                // cv::Point pt_next = cv::Point(polyLeft.at(next_move_y), next_move_y);
                // cv::Point pt_next_2 = cv::Point(polyLeft.at(next_move2_y), next_move2_y);
                // points
                cv::Point pt_next = cv::Point(arr_left.cells[next_move_y].x, next_move_y - num_suma);
                cv::Point pt_next_2 = cv::Point(arr_left.cells[next_move2_y].x, next_move2_y - num_suma);
                angle = atan2(pt_next.y - pt_next_2.y, pt_next.x - pt_next_2.x);
                hip = state_width_pix / sin(angle); // co = ca * tan (theta)

                printf("\n 2. angulo %.2f, co: %.2f", angle, hip);

                // ransac
                // y_next_pt = cv::Point(polyLeft.at(next_move_y) - hip, next_move_y);
                // y_next_pt2 = cv::Point(polyLeft.at(next_move2_y) - hip, next_move2_y);
                // points
                y_next_pt = cv::Point(arr_left.cells[next_move_y].x - hip, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point(arr_left.cells[next_move2_y].x - hip, image_height - next_move2_y - num_suma);
            } else if (C > 0 && pt_car.x < arr_right.cells[next_move_y].x) {
                // ransac
                // cv::Point pt_next = cv::Point(polyCenter.at(next_move_y), next_move_y);
                // cv::Point pt_next_2 = cv::Point(polyCenter.at(next_move2_y), next_move2_y);
                // points
                cv::Point pt_next = cv::Point(arr_center.cells[next_move_y].x, next_move_y + num_suma);
                cv::Point pt_next_2 = cv::Point(arr_center.cells[next_move2_y].x, next_move2_y + num_suma);
                angle = atan2(pt_next.y - pt_next_2.y, pt_next.x - pt_next_2.x);
                hip = state_width_pix / sin(angle); // co = ca * tan (theta)

                printf("\n 3. angulo %.2f, co: %.2f", angle, hip);
                // ransac
                // y_next_pt = cv::Point(polyCenter.at(next_move_y) - hip, next_move_y);
                // y_next_pt2 = cv::Point(polyCenter.at(next_move2_y) - hip, next_move2_y);
                // points
                y_next_pt = cv::Point(arr_center.cells[next_move_y].x - hip, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point(arr_center.cells[next_move2_y].x - hip, image_height - next_move2_y - num_suma);
            } else if (R > 0) {
                // ransac
                // cv::Point pt_next = cv::Point(polyRight.at(next_move_y), next_move_y);
                // cv::Point pt_next_2 = cv::Point(polyRight.at(next_move2_y), next_move2_y);
                // points
                cv::Point pt_next = cv::Point(arr_right.cells[next_move_y].x, next_move_y + num_suma);
                cv::Point pt_next_2 = cv::Point(arr_right.cells[next_move2_y].x, next_move2_y + num_suma);
                angle = atan2(pt_next.y - pt_next_2.y, pt_next.x - pt_next_2.x);
                hip = state_width_pix / sin(angle); // co = ca * tan (theta)

                printf("\n 4. angulo %.2f, co: %.2f", angle, hip);

                // ransac
                // y_next_pt = cv::Point(polyRight.at(next_move_y) - hip, next_move_y);
                // y_next_pt2 = cv::Point(polyRight.at(next_move2_y) - hip, next_move2_y);
                // points
                y_next_pt = cv::Point(arr_right.cells[next_move_y].x - hip, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point(arr_right.cells[next_move2_y].x - hip, image_height - next_move2_y - num_suma);
            }
            else {
                return false;
            }
            break;
        case 1: // LL
            // ransac
            // if (polyDetectedLeft && pt_car.x < polyCenter.at(next_move_y)) {
            // points
            if (L > 0 && pt_car.x < arr_center.cells[next_move_y].x) {
                // ransac
                // y_next_pt = cv::Point(polyLeft.at(next_move_y), next_move_y);
                // y_next_pt2 = cv::Point(polyLeft.at(next_move2_y), next_move2_y);
                // points
                y_next_pt = cv::Point(arr_left.cells[next_move_y].x - hip, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point(arr_left.cells[next_move2_y].x - hip, image_height - next_move2_y - num_suma);
            } else if (C > 0 && pt_car.x < arr_right.cells[next_move_y].x) {
                // ransac
                // y_next_pt = cv::Point(polyCenter.at(next_move_y), next_move_y);
                // y_next_pt2 = cv::Point(polyCenter.at(next_move2_y), next_move2_y);
                // points
                y_next_pt = cv::Point(arr_center.cells[next_move_y].x - hip, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point(arr_center.cells[next_move2_y].x - hip, image_height - next_move2_y - num_suma);
            } else if (R > 0) {
                // y_next_pt = cv::Point(polyRight.at(next_move_y), next_move_y);
                // y_next_pt2 = cv::Point(polyRight.at(next_move2_y), next_move2_y);
                // points
                y_next_pt = cv::Point(arr_right.cells[next_move_y].x - hip, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point(arr_right.cells[next_move2_y].x - hip, image_height - next_move2_y - num_suma);
            }
            else {
                return false;
            }
            break;
        case 2: // LC
            // ransac
            /*
            if (polyDetectedCenter && polyDetectedLeft && pt_car.x < polyCenter.at(next_move_y)) {
                y_next_pt = cv::Point((polyCenter.at(next_move_y) + polyLeft.at(next_move_y))/2, next_move_y);
                y_next_pt2 = cv::Point((polyCenter.at(next_move2_y) + polyLeft.at(next_move2_y))/2, next_move2_y);
            } else if (polyDetectedCenter && polyDetectedRight) {
                y_next_pt = cv::Point((polyCenter.at(next_move_y) + polyRight.at(next_move_y))/2, next_move_y);
                y_next_pt2 = cv::Point((polyCenter.at(next_move2_y) + polyRight.at(next_move2_y))/2, next_move2_y);
            }
            */
            // points
            if (C > 0 && L > 0 && pt_car.x < arr_center.cells[next_move_y].x) {
                y_next_pt = cv::Point((arr_center.cells[next_move_y].x + arr_left.cells[next_move_y].x)/2, image_height - next_move_y  - num_suma);
                y_next_pt2 = cv::Point((arr_center.cells[next_move2_y].x + arr_left.cells[next_move2_y].x)/2, image_height - next_move2_y - num_suma);
            } else if (C > 0 && R > 0) {
                y_next_pt = cv::Point((arr_center.cells[next_move_y].x + arr_right.cells[next_move_y].x)/2, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point((arr_center.cells[next_move2_y].x + arr_right.cells[next_move2_y].x)/2, image_height - next_move2_y - num_suma);
            }
            else {
                return false;
            }
            break;
        case 3: // CC
            // ransac
            /*
            if (polyDetectedLeft && polyDetectedCenter && polyDetectedRight) {
                // si veo todas las lineas
                x_center = polyCenter.at(next_move_y);

                y_next_pt = cv::Point( x_center , next_move_y);
                y_next_pt2 = cv::Point( polyCenter.at(next_move2_y) , next_move2_y);
            }
            else if (polyDetectedCenter && polyDetectedRight) {
                // veo dos lineas
                x_center = polyCenter.at(next_move_y);
                x_right = polyRight.at(next_move_y);
                center_closer = abs(pt_car.x - x_center) < abs(pt_car.x - x_right);
                y_next_pt = cv::Point(center_closer ? x_center : x_right, next_move_y);
                y_next_pt2 = cv::Point(center_closer ? polyCenter.at(next_move2_y) : polyRight.at(next_move2_y), next_move2_y);

            }   */
            if (L > 0 && C > 0 && R > 0) {
                // si veo todas las lineas

                y_next_pt = cv::Point( arr_center.cells[next_move_y].x , image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point( arr_center.cells[next_move2_y].x , image_height - next_move2_y - num_suma);
            }
            else if (C > 0 && R > 0) {
                // veo dos lineas
                x_center = arr_center.cells[next_move_y].x;
                x_right = arr_right.cells[next_move_y].x;
                center_closer = abs(pt_car.x - x_center) < abs(pt_car.x - x_right);
                y_next_pt = cv::Point(center_closer ? x_center : x_right, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point(center_closer ? arr_center.cells[next_move2_y].x : arr_right.cells[next_move2_y].x, image_height - next_move2_y - num_suma);

            }
            else {
                return false;
            }
            break;
        case 4: // RC
            // ransac
            /*
            if (polyDetectedCenter && polyDetectedRight) {
                if (car_center > polyRight.at(next_move_y)) {

                    cv::Point pt_next = cv::Point(polyRight.at(next_move_y), next_move_y);
                    cv::Point pt_next_2 = cv::Point(polyRight.at(next_move2_y), next_move2_y);
                    angle = atan2(pt_next.y - pt_next_2.y, pt_next.x - pt_next_2.x);
                    hip = state_width_pix / sin(angle); // co = ca * tan (theta)

                    printf("\n 5. angulo %.2f, co: %.2f", angle, hip);

                    y_next_pt = cv::Point(polyRight.at(next_move_y) + hip, next_move_y);
                    y_next_pt2 = cv::Point(polyRight.at(next_move2_y) + hip, next_move2_y);

                } else {

                    y_next_pt = cv::Point((polyCenter.at(next_move_y) + polyRight.at(next_move_y))/2, next_move_y);
                    y_next_pt2 = cv::Point((polyCenter.at(next_move2_y) + polyRight.at(next_move2_y))/2, next_move2_y);

                }
            } */
            // points
            if (C > 0 && R > 0) {
                if (car_center > arr_right.cells[next_move_y].x) {

                    cv::Point pt_next = cv::Point(arr_right.cells[next_move_y].x, next_move_y);
                    cv::Point pt_next_2 = cv::Point(arr_right.cells[next_move2_y].x, next_move2_y);
                    angle = atan2(pt_next.y - pt_next_2.y, pt_next.x - pt_next_2.x);
                    hip = state_width_pix / sin(angle); // co = ca * tan (theta)

                    printf("\n 5. angulo %.2f, co: %.2f", angle, hip);

                    y_next_pt = cv::Point(arr_right.cells[next_move_y].x + hip, image_height - next_move_y - num_suma);
                    y_next_pt2 = cv::Point(arr_right.cells[next_move2_y].x + hip, image_height - next_move2_y - num_suma);

                } else {
                    // ransac
                    // y_next_pt = cv::Point((polyCenter.at(next_move_y) + polyRight.at(next_move_y))/2, next_move_y);
                    // y_next_pt2 = cv::Point((polyCenter.at(next_move2_y) + polyRight.at(next_move2_y))/2, next_move2_y);
                    // points
                    y_next_pt = cv::Point((arr_center.cells[next_move_y].x + arr_right.cells[next_move_y].x)/2, image_height - next_move_y - num_suma);
                    y_next_pt2 = cv::Point((arr_center.cells[next_move2_y].x + arr_right.cells[next_move2_y].x)/2, image_height - next_move2_y - num_suma);
                }
            }
            else {
                        return false;
                }
            break;
        case 5: // RR
            if (R > 0) {
                y_next_pt = cv::Point(arr_right.cells[next_move_y].x, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point(arr_right.cells[next_move2_y].x, image_height - next_move2_y - num_suma);
            } else {
                return false;
            }
            break;
        case 6: // OR
            if (R > 0) {
                cv::Point pt_next = cv::Point(arr_right.cells[next_move_y].x, next_move_y);
                cv::Point pt_next_2 = cv::Point(arr_right.cells[next_move2_y].x, next_move2_y);
                angle = atan2(pt_next.y - pt_next_2.y, pt_next.x - pt_next_2.x);
                hip = state_width_pix / sin(angle); // co = ca * tan (theta)

                printf("\n 6. angulo %.2f, co: %.2f", angle, hip);

                y_next_pt = cv::Point(arr_right.cells[next_move_y].x + hip, image_height - next_move_y - num_suma);
                y_next_pt2 = cv::Point(arr_right.cells[next_move2_y].x + hip, image_height - next_move2_y - num_suma);
            } else {
                return false;
            }
            break;
    }
    return true;
}

// cv::Mat& imagePaint, NewtonPolynomial& polyLeft, NewtonPolynomial& polyCenter,
// NewtonPolynomial& polyRight, int estado_actual, int estado_deseado
void local_planner::ackerman_control(cv::Mat& imagePaint) {

    //
    // SI estado actual es DIFERENTE a estado requerido
    // 1.- Steering requerido para cambiar de estado
    //      (positivo o negativo dependiendo del estado actual)
    // 2.- Ciertos pixeles en eje y (marco carro) por ejemplo 10 para hacer un cambio suave
    //      con base a y, calcular steering de acuerdo a velocidad
    // 3.- El 10 es positivo o negativo de acuerdo a si nuevo estado es mayor o menor
    // 4.- Enviar este steering como accion de control
    //

    // car location (position bottom to up with respect to next points, to compute angle)
    // ransac
    // cv::Point ptCar = cv::Point(car_center, image_height);
    // points
    cv::Point ptCar = cv::Point(car_center, image_height);
    cv::circle(imagePaint, ptCar, 2, cv::Scalar(0,255,255), -1);

    // calcular siguiente punto sobre eje Y (X con respecto al carro) de acuerdo a velocidad y steering actual
    // aqui obtener velocidad de IMU y orientacion el carro
    // TODO IMU
    double dist_y_nextPoint = 2; // actual_speed * 5 * sin(PI/2 - actual_steering); //*;

    double dist_yPoly = 40 * abs(estado_deseado - estado_actual);

    cv::Point nextPoint, nextPoint2, pointSlope;

    // siguente punto en el estado en que se encuentra
    // primera opcion para TRAYECTORIA
    // if (estado_actual != estado_deseado)
    //     puntosValidos = ackerman_control_next_points(dist_yPoly, ptCar, nextPoint, nextPoint2);
    // else

    bool puntosValidos = ackerman_control_next_points(dist_y_nextPoint, ptCar, nextPoint, nextPoint2,
                                                      poly_left, poly_center, poly_right);

    if (puntosValidos) {
        // Cambio de estado, puede haber una forma mas suave
        if (estado_actual >= 0) {



            double angulo = atan2(nextPoint.y - nextPoint2.y, nextPoint.x - nextPoint2.x);
            double hipo = state_width_pix / sin(angulo); // co = ca * tan (theta)

            // printf("\n actual: %d, deseado: %d \n x_actual: %d, x_deseado_1: %d, x_deseado_2: %d \n hipotenusa: %.2f", estado_actual, estado_deseado, car_center, nextPoint.x, nextPoint2.x, hipo);

            // printf("\n 1. pix: %d, angulo %.2f, co: %.2f, hip: %.2f, cos: %.2f", state_width_pix, angulo,  hipo, state_width_pix / sin(angulo), sin(angulo));

            if (estado_actual < estado_deseado) {

                nextPoint.x = nextPoint.x + hipo * (estado_deseado - estado_actual);
                nextPoint2.x = nextPoint2.x + hipo * (estado_deseado - estado_actual);

            } else if (estado_actual > estado_deseado) {

                nextPoint.x = nextPoint.x - hipo * (estado_actual - estado_deseado);
                nextPoint2.x = nextPoint2.x - hipo * (estado_actual - estado_deseado);

            }
        }

        /* SMOOTH MOVE
        // almacenar diferencia en x, para mantener posteriormente
        double diff_x = nextPoint2.x - nextPoint.x;

        // TRAYECTORIA, Hay (x,y) origen (car_center), faltan puntos intermedios para alcanzar objetivo (x, dist_y)
        if (estado_actual != estado_deseado) {
            // SMOOTH MOVE
            NewtonPolynomial poly = NewtonPolynomial();
            double p1X = ptCar.x;
            double p1Y = ptCar.y;
            double p2X = nextPoint.x;
            double p2Y = nextPoint.y;
            poly.addData(p1X, p1Y);
            poly.addData(p2X, p2Y);

            for (int i = p1Y; i > p2Y; i--) {
                cv::Point pt = cv::Point(poly.at(i), i);
                cv::circle(imagePaint, pt, 0, cv::Scalar(255, 255, 255), -1);
            }

            nextPoint = cv::Point(poly.at(maxYRoi - dist_y), maxYRoi - dist_y);
            nextPoint2 = cv::Point(poly.at(maxYRoi - dist_y) + diff_x, maxYRoi - dist_y - 10);
        }
        */

        // -- visualize points ---------

        cv::circle(imagePaint, nextPoint, 2, cv::Scalar(0, 100, 255), -1);
        cv::circle(imagePaint, nextPoint2, 2, cv::Scalar(0 ,100, 255), -1);
        cv::circle(imagePaint, pointSlope, 2, cv::Scalar(245,245,0), -1);


        // ------------- ACKERMAN CONTROL -------------------
        // TODO falta considerar THETA_CARRO
        // ---angles
        // intercambio de coordenadas por frame rotado


        double G_x_cord = -nextPoint.y - -ptCar.y;
        double G_y_cord = ptCar.x - nextPoint.x;
        // alpha, angel between car and GOAL in radians
        double alpha = atan2(G_y_cord, G_x_cord);

        /*
        // intercambio de coordenadas de punto 2
        double G_sup_x = -nextPoint2.y - -ptCar.y;
        double G_sup_y = ptCar.x - nextPoint2.x;

        // law of cosines to compute beta, the angle of the goal with respect to alpha
        double a = sqrt(pow(G_x_cord, 2) + pow(G_y_cord, 2));
        double b = sqrt(pow(G_x_cord - G_sup_x, 2) + pow(G_y_cord - G_sup_y, 2));
        double c = sqrt(pow(-G_sup_x, 2) + pow(-G_sup_y,2));
        double beta = acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a * b));
        beta = PI - beta;
        printf("\n alpha: %+010.2f, beta: %+010.2f", alpha, beta);
        double steering_cont = kalpha * alpha + kbeta * beta;
        */

        /*

        double theta = yaw;
        double beta = - theta - alpha;
        double steering = kalpha * alpha + kbeta * beta;
        printf("\n alpha: %+010.2f, beta: %+010.2f", alpha, beta);

        */

        //-----PUBLISH ------
        // utilizando pixeles
        double y = ptCar.x - nextPoint.x;
        double error = atan2(y, 20);

        double steering = PID(error, 0.2, Kp, Ki, Kd); // regresa 45 a -45. Izquierda a Derecha.

        // utilizando grados
        // double steering = PID(steering_cont, 0.2, 0.5, 0.001, 0.0);

        printf ("\n PID: car: %d, next: %d, steering: %+04.2f", ptCar.x, nextPoint.x, steering);
        // -------------- FINISH ACKERMAN CONTROL -----------
        if (!std::isnan(steering)) {
            float steering_rounded = round(steering * 100) / 100;            

            // intercambio de coordenadas de punto 2
            // double G_x_above = -pointSlope.y - -ptCar.y;
            // double G_y_above = ptCar.x - pointSlope.x;

            // compute angle of point1 vs point2 to tilt lines to detect polys
            // used above
            // polysAngle = atan2(G_y_above - G_y_cord, G_x_above - G_x_cord);
            // cv::putText(imagePaint,std::to_string(polysAngle),markingLoc,6,.15,cv::Scalar(0,237,221));

            geometry_msgs::Twist vel;
            vel.angular.z = steering_rounded;
            vel.linear.x = car_speed;
            // speed is constant

            // falta PID
            pub_speed_sta.publish(vel);
        }
    }
}

double local_planner::PID(double error, double dt, double Kp, double Ki, double Kd){
    // ROS_INFO_STREAM("PID time");


    double pOut = Kp * error;
    integralPID += error * dt;
    double iOut = Ki * integralPID;
    double derivative = (error - prevErrorPID) / dt;
    double dOut = Kd * derivative;
    double output = pOut + iOut + dOut;
    prevErrorPID = error;

    // Restriction
    double out_restringido = output;
    if( output > max_steering )
            out_restringido = max_steering;
    else if( output <= min_steering )
            out_restringido = min_steering;

    printf("\n Error theta: %+010.4f, Res PID: %+010.4f, Senal Servo: %+010.4f, p: %.2f i: %.2f d: %.2f", error, output, out_restringido, pOut, iOut, dOut );

    return out_restringido;
}

bool local_planner::polynomial_exists() {
    return estado_actual >= 0 && (polyDetectedLeft || polyDetectedCenter || polyDetectedRight);

    /*
     *     if (polyDetectedLeft)
                            printf("\n OK left X: %.2f, LMsize: %d", polyLeft.at(car_text_position), (int) laneMarkingsLeft.size() );
                    if (polyDetectedCenter)
                            printf("\n OK center X: %.2f, LMsize: %d", polyCenter.at(car_text_position), (int) laneMarkingsCenter.size() );
                    if (polyDetectedRight)
                            printf("\n OK right X: %.2f, LMsize: %d", polyRight.at(car_text_position), (int) laneMarkingsRight.size() );
                    printf("\n Creating ackerman points ");
     */
}

void local_planner::plot_polinomials(cv::Mat& image) {

    int pix_size = 0;
    // markings


    cv::Point pointLoc;

    // if (polyDetectedLeft || polyDetectedCenter || polyDetectedRight)
    // {
        for (int i = 0; i < image_height; i++) {
            cv::line(image, cv::Point(0, i), cv::Point(160, i), cv::Scalar(0, 0, 0), 1, CV_AA);

            // ransac
            /*
            if (polyDetectedLeft && poly_left.isInitialized()) {
                pointLoc = cv::Point(poly_left.at(i), i);
                if (i == 159) pix_size = 2;
                else pix_size = 0;
                cv::circle(image, pointLoc, pix_size, cv::Scalar(0,0,200), -1);
            }
            if (polyDetectedCenter && poly_center.isInitialized()) {
                pointLoc = cv::Point(poly_center.at(i), i);
                if (i == 159) pix_size = 2;
                else pix_size = 0;
                cv::circle(image, pointLoc, pix_size, cv::Scalar(0,200,0), -1);
            }
            if (polyDetectedRight && poly_right.isInitialized()) {
                pointLoc = cv::Point(poly_right.at(i), i);
                if (i == 159) pix_size = 2;
                else pix_size = 0;
                cv::circle(image, pointLoc, pix_size, cv::Scalar(200,0,0), -1);
            }
            */
        }
//    }

    // printf("\n planner %d %d %d", L, C, R);


    if (L > 0) {
        // printf("\n left L: %d, width: %d", L, (int) arr_left.cell_width);
        for(int i = 0; i < (int) arr_left.cell_width ; i++) {
            // printf("\n right pt %d (%d, %d) ", i, (int) arr_left.cells[i].x, (int) arr_left.cells[i].y);
            pointLoc = cv::Point( (int) arr_left.cells[i].x, (int) arr_left.cells[i].y);
            cv::circle(image, pointLoc, 1, cv::Scalar(0,0,100), -1);
        }
    }
    if (C > 0) {
        // printf("\n center C: %d, width: %d", C, (int) arr_center.cell_width);
        for(int i = 0; i < (int) arr_center.cell_width; i++) {
            // printf("\n right pt %d (%d, %d) ", i, (int) arr_center.cells[i].x, (int) arr_center.cells[i].y);
            pointLoc = cv::Point( (int) arr_center.cells[i].x, (int) arr_center.cells[i].y);
            cv::circle(image, pointLoc, 1, cv::Scalar(0,100,0), -1);
        }
    }
    if (R > 0) {
        // printf("\n right R: %d, width: %d", R, (int) arr_right.cell_width);
        for(int i = 0; i < (int) arr_right.cell_width; i++) {
            // printf("\n right pt %d (%d, %d) ", i, (int) arr_right.cells[i].x, (int) arr_right.cells[i].y);
            pointLoc = cv::Point( (int) arr_right.cells[i].x, (int) arr_right.cells[i].y);
            cv::circle(image, pointLoc, 1, cv::Scalar(100,0,0), -1);
        }
    }





    printf("\n actual: %d, deseado: %d", estado_actual, estado_deseado);

    // posicion del carro
    pointLoc = cv::Point(car_center, image_height);
    cv::circle(image, pointLoc, 2, cv::Scalar(200,200,200), -1);

    // muestra en el estado en que me encuentro
    cv::Point pointTextEstado = cv::Point(car_center + 30, car_text_position - 20);
    if (estado_actual >= 0)
        cv::putText(image, "ACT: " + nombre_estado[estado_actual], pointTextEstado, 0, .25, cv::Scalar(200,221,0));
    else
        cv::putText(image, "?", pointTextEstado, 0, .3, cv::Scalar(200,221,0));

    // muestra el estado al que me quiero desplazar
    pointTextEstado = cv::Point(car_center + 30, car_text_position);

    cv::putText(image, "DES: " + nombre_estado[estado_deseado], pointTextEstado, 0, .25, cv::Scalar(200,221,0));

}

int main(int argc, char** argv){
    ros::init(argc, argv, "local planner");
    ROS_INFO_STREAM("local planner initialized");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE_HZ);

    local_planner local = local_planner(nh);

    int img_height = 160;
    int img_width = 160;
    cv::Mat imagePaint = cv::Mat(img_height, img_width, CV_8UC3, cv::Scalar(0, 0, 0));
    sensor_msgs::ImagePtr imgmsg;


    while (ros::ok())
    {

            ros::spinOnce();

            local.plot_polinomials(imagePaint);

            if (local.polynomial_exists())
                  local.ackerman_control(imagePaint);

            imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imagePaint).toImageMsg();
            local.pub_image.publish(imgmsg);

            loop_rate.sleep();

    }


    return 0;
}
