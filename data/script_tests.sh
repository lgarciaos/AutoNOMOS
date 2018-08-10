#!/bin/bash
echo "Using controller: "$1
home_=$(pwd)
source ../devel/setup.bash

date_=$(date +"%d-%m-%y_%H%M%S")
file_aux="/data_aux_"$1"_"$date_".txt"
file_="/data_"$1"_"$date_".txt"
planner="SST"

#rm $home_$file_aux


cont_planner=1
until [ $cont_planner -gt 1 ] # 2
do
  iters=10000 #1000000
  counter_ext=1
  rosservice call /gazebo/reset_simulation "{}"
  until [ $counter_ext -gt 1 ] # 12
  do
    counter_int=1
    rosservice call /gazebo/reset_simulation "{}"
    until [ $counter_int -gt 1 ] # 10
    do
      rosservice call /gazebo/reset_simulation "{}"
      echo $planner" iters: "$iters" count: "$counter_int" using: "$1" to: ("$2", "$3", "$4")."
      roslaunch simulation dummy.launch stopping_check_in:=$iters \
        planner_in:=$planner publish_ctrl_path_in:=false ctrl_to_use_in:=$1 \
        goal_state/x:=$2 goal_state/y:=$3 goal_state/theta:=$4 &>> $home_$file_aux

        # add code to check that there is new data after the roslaunch command
        # it could be done using the output of the cmd awk '/Solution Quality'
        # with wc -l and check that there is greater that the previous iteration
      ((counter_int++))
    done
    iters=$((iters / 2))
    ((counter_ext++))
  done
  planner="RRT"
  ((cont_planner++))
done

# echo $iters
cat $home_$file_aux | awk '/Solution Quality/' >> $home_$file_
