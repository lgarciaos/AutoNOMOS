#!/bin/bash
echo "Motion planner: "$1", with controller: "$2
home_=$(pwd)
source ../devel/setup.bash

if [[ $3 -gt 0 ]]; then
  x_p=${3#+}
  x_p="p"$x_p
else
  echo "neg"
  x_p=${3#-}
  echo "x_p: "$x_p
  x_p="n$x_p"
fi

if [[ $4 -gt 0 ]]; then
  y_p=${4#+}
  y_p="p"$y_p
else
  y_p=${4#-}
  y_p="n"$y_p
fi

date_=$(date +"%d-%m-%y_%H%M%S")
file_aux="/data_aux_"$x_p$y_p$t_p"_"$1"_"$date_".txt"
file_="/data_"$x_p$y_p$t_p"_"$1"_"$date_".txt"


planner=$1


cont_planner=1
last_wc=0

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
    echo $planner" iters: "$iters" count: "$counter_int" using: "$1"-"$2" to: ("$3", "$4", "$5")."
    roslaunch simulation dummy.launch \
      stopping_check_in:=$iters \
      planner_in:=$planner \
      publish_ctrl_path_in:=false \
      ctrl_to_use_in:=$2 \
      goal_state/x:=$3 \
      goal_state/y:=$4 \
      goal_state/theta:=$5 \
      gz_plot_lines_in:=false &>> $home_$file_aux
      if [[ $(cat $home_$file_aux | awk '/Solution Quality/' | wc -l ) -gt last_wc ]]; then
        ((last_wc++))
        ((counter_int++))
      else
        echo "failed test"
      fi
  done
  iters=$((iters / 2))
  ((counter_ext++))
done

echo "Aux file: "$file_aux
echo "Res file: "$file_

cat $home_$file_aux | awk '/Solution Quality/' >> $home_$file_

