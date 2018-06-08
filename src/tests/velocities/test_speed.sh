#!/bin/bash

cd src/tests/velocities/

while read vel; do
	COUNTER=$(expr $COUNTER + 1)

	echo $vel

	source /home/emartinene/git/EK_AutoNOMOS_Sim/devel/setup.bash

	#2>&1 redirect stderr to stdout, 
	# nohup roslaunch autonomos_gazebo_simulation empty_curved_road.launch 2>&1 &

	nohup roslaunch autonomos_gazebo_simulation empty_curved_road_v2.launch 2>&1 &


	pid_gazebo=$!

	source /home/emartinene/git/AutoNOMOS/devel/setup.bash

	nohup rosrun gazebo_ros spawn_model -database AutoNOMOS_mini -sdf -model AutoNOMOS_mini -x 0 -y 0.36 -z 0.019 -R 0 -P 0 -Y 3.14159  &

	nohup rosrun line_detection_fu_mod line_detection_fu_node \
		_cam_w:=640 \
		_cam_h:=480 \
		_proj_y_start:=50 \
		_proj_image_h:=160 \
		_proj_image_w:=160 \
		_proj_image_horizontal_offset:=0 \
		_roi_top_w:=160 \
		_roi_bottom_w:=50 \
		_maxYRoi:=159 \
		_minYDefaultRoi:=110 \
		_minYPolyRoi:=45 \
		_defaultXLeft:=38 \
		_defaultXCenter:=65 \
		_defaultXRight:=93 \
		_defaultYHorizontal:=100 \
		_interestDistancePoly:=5 \
		_interestDistanceDefault:=10 \
		_iterationsRansac:=50 \
		_proportionThreshould:=0.85 \
		_m_gradientThreshold:=1 \
		_m_nonMaxWidth:=4 \
		_laneMarkingSquaredThreshold:=144 \
		_angleAdjacentLeg:=18 \
		_scanlinesVerticalDistance:=2 \
		_scanlinesMaxCount:=200 \
		_polyY1:=155 \
		_polyY2:=145 \
		_polyY3:=130 \
		_detectLaneStartX:=155 \
		_maxAngleDiff:=999 \
		_camera_name:=/app/camera/rgb/image_raw \
		_cam_deg:=4 \
		_cam_height:=14 \
		_f_u:=655.554626 \
		_f_v:=652.052734 \
		_c_u:=312.773367 \
		_c_v:=7.779505 \
		_dbscan_epsilon:=25 \
		_dbscan_min_points:=5 \
		_car_center:=80 \
		_num_execution:=$COUNTER 2>&1 &
	# > detection_$COUNTER.log


	nohup rosrun lane_states_original lane_states_node \
        _car_center:=80 \
        _image_height:=160 \
        _state_width_pixels:=16 \
        _pix_width_mts:=0.00071 \
        _model_num_gazebo:=1 \
        _num_execution:=$vel > lane_states_$vel.log 2>&1 &

    nohup rosrun local_planner local_planner_node \
        _Kp:=1.0 \
        _Ki:=0.01 \
        _Kd:=0.0 \
        _car_center:=80 \
        _car_speed:=$vel \
        _min_steering:=-0.41 \
        _max_steering:=0.41 \
        _topico_estandarizado:=/standarized_vel_ste \
        _image_height:=160 \
        _estado_deseado:=4 \
        _state_width_pixels:=16 2>&1 &
    # > local_planner_$COUNTER.log

    nohup rosrun translate_sim_real translate_sim_real_node \
        _topico_steering_sim:=/AutoNOMOS_mini/manual_control/steering \
        _topico_velocidad_sim:=/AutoNOMOS_mini/manual_control/velocity \
        _topico_steering_sim_gary:=/AutoNOMOS_mini/manual_control/steering \
        _topico_velocidad_sim_gary:=/AutoNOMOS_mini/manual_control/speed \
        _topico_estandarizado:=/standarized_vel_ste \
        _seleccion_real_simulacion:=gary \
        _offset_angle:=90 2>&1 &

	sleep 10m #

	pid_nodes=$(ps -o user,pid,ppid,command -ax | grep 'line_detection_fu_node\|lane_states_node\|local_planner_node\|translate_sim_real_node' | awk '{print $2}')
	
	for child in $pid_nodes
	do
		echo "Killing process pid = $child"
		kill $child
	done

	kill $pid_gazebo

	sleep 1m #

done < velocities.txt

