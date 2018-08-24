#!/bin/bash
home_=$(pwd)
source ../devel/setup.bash

planner=$1
ctrl=$2
x_in=$3
y_in=$4
th_in=$5

echo "Motion planner: $planner, with controller: $ctrl"


if [[ $x_in -gt 0 ]]; then
  x_p=${x_in#+}
  x_p="p"$x_p
else
  echo "neg"
  x_p=${x_in#-}
  echo "x_p: "$x_p
  x_p="n$x_p"
fi

if [[ $y_in -gt 0 ]]; then
  y_p=${y_in#+}
  y_p="p"$y_p
else
  y_p=${y_in#-}
  y_p="n"$y_p
fi

date_=$(date +"%d-%m-%y_%H%M%S")
file_aux="/data_aux_"$x_p$y_p$t_p"_"$planner"-"$ctrl"_"$date_".txt"
file_="/data_"$x_p$y_p$t_p"_"$planner"-"$ctrl"_"$date_".txt"




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
    echo $planner" iters: "$iters" count: "$counter_int" using: "$planner"-"$ctrl" to: ("$x_in", "$y_in", "$th_in")."
    roslaunch simulation dummy.launch \
      stopping_check_in:=$iters \
      planner_in:=$planner \
      publish_ctrl_path_in:=false \
      ctrl_to_use_in:=$ctrl \
      goal_state/x:=$x_in \
      goal_state/y:=$y_in \
      goal_state/theta:=$th_in \
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
