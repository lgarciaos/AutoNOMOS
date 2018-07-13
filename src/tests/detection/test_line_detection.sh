#!/bin/bash

cd src/tests/detection/

while read rad; do
	echo $rad
	counter=$(expr $counter + 1)

	source /home/emartinene/git/EK_AutoNOMOS_Sim/devel/setup.bash

	#2>&1 redirect stderr to stdout, 
	nohup roslaunch autonomos_gazebo_simulation empty_curved_road.launch 2>&1 &

	pid_gazebo=$!

	source /home/emartinene/git/AutoNOMOS/devel/setup.bash

	nohup rosrun gazebo_ros spawn_model -database AutoNOMOS_mini -sdf -model AutoNOMOS_mini -x -.5 -y 0 -z .19 -R 0 -P 0 -Y $rad 2>&1 &

	pid_spawner=$!
	# echo $pid_spawner

	nohup rosrun line_detection_fu_mod line_detection_fu_node _cam_w:=640 \
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
		_m_nonMaxWidth:=10 \
		_laneMarkingSquaredThreshold:=144 \
		_angleAdjacentLeg:=18 \
		_scanlinesVerticalDistance:=2 \
		_scanlinesMaxCount:=200 \
		_polyY1:=155 \
		_polyY2:=145 \
		_polyY3:=130 \
		_detectLaneStartX:=155 \
		_maxAngleDiff:=999 \
		_camera_name:=app/camera/rgb/image_raw \
		_cam_deg:=4 \
		_cam_height:=14 \
		_f_u:=655.554626 \
		_f_v:=652.052734 \
		_c_u:=312.773367 \
		_c_v:=7.779505 \
		_dbscan_epsilon:=25 \
		_dbscan_min_points:=5 \
		_car_center:=80  \
		_num_execution:=counter > detection_$counter.log 2>&1 &

	pid_vision=$!
	# echo $pid_vision
	# ps -fea | grep rosrun

	sleep 1m #

	# kill $pid_spawner
	# kill $pid_vision
	pid_vision=$(ps -o user,pid,ppid,command -ax | grep line_detection_fu_node | awk '{print $2}')
	# pid_vision=$(ps -o user,pid,ppid,command -ax | grep gzserver | awk '{print $2}')

	for child in $pid_vision
	do
		echo "Killing process pid = $child"
		kill $child
	done

	kill $pid_gazebo

	sleep 20s #

done < radians.txt


