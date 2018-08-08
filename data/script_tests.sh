#!/bin/bash
echo "Using controller: "$1
home_=$(pwd)
cd AutoNOMOS/
source devel/setup.bash

date_=$(date +"%d-%m-%y_%T")
file_aux="/data_aux.txt"
file_="/data_"
planner="SST"

rm $home_$file_aux


cont_planner=1
until [ $cont_planner -gt 2 ] # 2
do
  iters=1000000
  counter_ext=1
  rosservice call /gazebo/reset_simulation "{}"
  until [ $counter_ext -gt 12 ] # 12
  do
    counter_int=1
    rosservice call /gazebo/reset_simulation "{}"
    until [ $counter_int -gt 10 ] # 10
    do
      rosservice call /gazebo/reset_simulation "{}"
      echo "planner: "$planner" iters: "$iters" count: "$counter_int
      roslaunch simulation dummy.launch stopping_check_in:=$iters \
        planner_in:=$planner publish_ctrl_path_in:=false ctrl_to_use_in:=$1 \
        &>> $home_$file_aux
      ((counter_int++))
    done
    iters=$((iters / 2))
    ((counter_ext++))
  done
  planner="RRT"
  ((cont_planner++))
done

# echo $iters
cat $home_$file_aux | awk '/Solution Quality/' >> $home_$file_$1"_"$date_".txt"
