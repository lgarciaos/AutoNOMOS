#!/bin/bash



function lanzar_gazebo {

	source /home/neri/git/EK_AutoNOMOS_Sim/devel/setup.bash
	nohup roslaunch autonomos_gazebo_simulation empty_curved_road.launch 2>&1 &

	# save pid 
	pid_gazebo=$!

	source /home/neri/git/AutoNOMOS/devel/setup.bash
	nohup rosrun gazebo_ros spawn_model -database AutoNOMOS_mini -sdf -model AutoNOMOS_mini -x .75 -y 0.0 -z 0.019 -R 0 -P 0 -Y 1.5707 2>&1 &

    # echo $pid_gazebo
    sleep 15s
}

# recibe un contador, velocidad y el arreglo de parametros
function recorrer_circuito {

	local counter=$1
	local vel=$2

	#local array_ks = $3

	declare -a array_ks=("${!3}")
	# local __resultcte=$4

	echo "recorrer: " $counter $vel ${array_ks[@]} 

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
		_num_execution:=$counter 2>&1 &

	nohup rosrun lane_states_original lane_states_node \
	    _car_center:=80 \
	    _image_height:=160 \
	    _state_width_pixels:=16 \
	    _pix_width_mts:=0.00071 \
	    _model_num_gazebo:=1 \
	    _num_execution:=$counter > lane_states_linea_${vel}_${array_ks[0]}${array_ks[1]}${array_ks[2]}.log 2>&1 &

	nohup rosrun local_planner local_planner_node \
	    _controlador:=linea \
	    _Kp:=${array_ks[0]} \
	    _Ki:=${array_ks[1]} \
	    _Kd:=${array_ks[2]} \
	    _Kdist:=2.816 \
	    _Kh:=0.729 \
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

	sleep 120s #

	pid_nodes=$(ps -o user,pid,ppid,command -ax | grep 'line_detection_fu_node\|lane_states_node\|local_planner_node\|translate_sim_real_node' | awk '{print $2}')

	for child in $pid_nodes;
	do
		# echo "Killing process pid = $child"
		nohup kill $child 2>&1 &
	done

	#frenar al carro
	nohup rostopic pub -1 /AutoNOMOS_mini/manual_control/speed std_msgs/Int16 "data: 0" 2>&1 &
	nohup rostopic pub -1 /AutoNOMOS_mini/manual_control/steering std_msgs/Int16 "data: 90" 2>&1 &

	#reset model poses
	nohup rosservice call /gazebo/reset_world 2>&1 &

	sleep 15s #

	# compute cte
	# local cte_l=$(tail -n +2 lane_states_linea_${vel}_${array_ks[0]}${array_ks[1]}${array_ks[2]}.log | awk '{s+=$169} END {print s}')

    # echo $cte_l
}

function sumar {
	declare -a array=("${!1}")
	local tot=0.0

#	echo "array" ${array[@]}
	for i in ${array[@]}; do
		# bash no soporta operaciones de punto flotante, se requere bc
		tot=$(echo "$tot+$i" | bc) 
	done
	
    echo $tot
}

# TODO: twiddle loop here
# def twiddle(tol=0.2): 

tol=0.02
p=( 0.0000 0.0000 )
dp=( 1.0000 1.0000 )
suma_dp=0
best_err=1000.0
err=0.0
speed=-180
COUNTER=$(expr $COUNTER + 1)

lanzar_gazebo
suma_dp=$(sumar dp[@])

# TWIDDLE algorithm
while [ $(echo "$suma_dp > $tol" | bc) -eq 1 ]; do
    COUNTER=$(expr $COUNTER + 1)

    for index in ${!p[@]}; do
        p[index]=$(echo "${p[index]} + ${dp[index]}" | bc)
	recorrer_circuito $COUNTER $speed p[@]
	err=$(tail -n +2 lane_states_linea_${speed}_${p[0]}${p[1]}${p[2]}.log | awk '{++n;sum+=($169^2)} END {print sum/n}')

        #if [$err -le $best_err]; then
        if [ $(echo "$err < $best_err" | bc) -eq 1 ]; then
            best_err=$err
            dp[index]=$(echo "${dp[index]} * 1.1" | bc)
        else
            p[index]=$(echo "${p[index]} - 2 * ${dp[index]}" | bc)
            recorrer_circuito $COUNTER $speed p[@]
            err=$(tail -n +2 lane_states_linea_${speed}_${p[0]}${p[1]}${p[2]}.log | awk '{++n;sum+=($169^2)} END {print sum/n}')

            if [ $(echo "$err < $best_err" | bc) -eq 1 ]; then
                best_err=$err
                dp[index]=$(echo "${dp[index]} * 0.9" | bc)
            else
                p[index]=$(echo "${p[index]} + ${dp[index]}" | bc)
                dp[index]=$(echo "${dp[index]} * 0.9" | bc)
            fi
        fi
    done
    echo "Iter: " $COUNTER $best_err ${p[@]} ${dp[@]}
    suma_dp=$(sumar dp[@])
done

# # probar la funcion recorrer circuito y calcular el error

# COUNTER=$(expr $COUNTER + 1)
# p[0]=$(echo "${p[0]} + ${dp[0]}" | bc)
# recorrer_circuito $COUNTER -200 p[@]
# err=$(tail -n +2 lane_states_$vel_${p[0]}${p[1]}${p[2]}.log | awk '{s+=$169} END {print s}')
# echo "1:" $err

# COUNTER=$(expr $COUNTER + 1)
# p[0]=$(echo "${p[0]} + ${dp[0]}" | bc)
# recorrer_circuito $COUNTER -200 p[@]
# err=$(tail -n +2 lane_states_$vel_${p[0]}${p[1]}${p[2]}.log | awk '{s+=$169} END {print s}')
# echo "2:" $err


echo "killing gazebo: " $pid_gazebo
kill $pid_gazebo
echo Ks: ${p[@]}
echo Mejor error: $best_err
echo Iteraciones: $COUNTER
