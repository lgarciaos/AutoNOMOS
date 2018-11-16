#!/bin/bash
if [ -z "$1" ] ; then
  echo "First argument must contain the controller to be used."
  exit 1
else
  if [ "$1" != "RANDOM_CTRL" ] && [ "$1" != "BANG_BANG" ] ; then
    echo "First argument is the controller: RANDOM_CTRL or BANG_BANG"
    exit 1
  else
    echo "Using controller: "$1
  fi
fi

if [ -z "$2" ] ; then
  echo "Second argument must contain a number: goal_state/x."
  exit 1
fi

if [ -z "$3" ] ; then
  echo "Third argument must contain a number: goal_state/y."
  exit 1
fi

if [ -z "$4" ] ; then
  echo "Fourth argument must contain a number: goal_state/theta."
  exit 1
fi

echo "goal_state: ($2, $3, $4)"

if [ -z "$5" ] ; then
  echo "Fifth argument must contain the stopping type: iterations or time."
  exit 1
else 
  if [ "$5" != "time" ] && [ "$5" != "iterations" ] ; then
    echo "Fifth argument is the stopping type: iterations or time."
    exit 1
  else
    echo "Using $5 as stopping type."
    if [ "$5" == "time" ] ; then
      INIT_TIME_ITERS="3600"
    else
      INIT_TIME_ITERS="1000000"
    fi
  fi
fi 

if rostopic list ;
  echo "Starting simulation..."
else
  echo "ROS is not running... use roslaunch simulation parking_lot.launch"
fi


# VARIABLES
planner="SST"  # The planner to start with, it will change to RRT after 
TOT_ITER_EXT=12 # Total iterations in the external loop
TOT_ITER_INT=10 # Total iterations in the internal loop
iters_init="$INIT_TIME_ITERS" # Initial iterations/time, it is divided by 2 each external iteration

# AUX VARIABLES
cont_planner=2 # Total number of planner - 1 
last_wc=0 #



home_=$(pwd)
source ../devel/setup.bash


if [[ $2 -gt 0 ]]; then
  x_p=${2#+}
  x_p="p"$x_p
else
  # echo "neg"
  x_p=${2#-}
  # echo "x_p: "$x_p
  x_p="n$x_p"
fi

if [[ $3 -gt 0 ]]; then
  y_p=${3#+}
  y_p="p"$y_p
else
  y_p=${3#-}
  y_p="n"$y_p
fi

date_=$(date +"%d-%m-%y_%H%M%S")
file_aux="/data_aux_"$x_p$y_p$t_p"_"$1"_"$date_".txt"
file_="/data_"$x_p$y_p$t_p"_"$1"_"$date_".txt"
STOPING_TYPE=$5


until [ $cont_planner -gt 0 ] # 2
do
  iters="$iters_init"
  counter_ext=1
  rosservice call /gazebo/reset_simulation "{}"
  until [ $counter_ext -gt $TOT_ITER_EXT ] # 12
  do
    counter_int=1
    rosservice call /gazebo/reset_simulation "{}"
    until [ $counter_int -gt TOT_ITER_INT ] # 10
    do
      rosservice call /gazebo/reset_simulation "{}"
      echo $planner" iters: "$iters" count: "$counter_int" using: "$1" to: ("$2", "$3", "$4")."
      roslaunch simulation dummy.launch \
        "stopping_type_in:=$STOPING_TYPE" \
        "stopping_check_in:=$iters" \
        "planner_in:=$planner" \
        "publish_ctrl_path_in:=false" \
        "ctrl_to_use_in:=$1" \
        "goal_state/x:=$2" \
        "goal_state/y:=$3" \
        "goal_state/theta:=$4" \
        "gz_plot_lines_in:=false" &>> $home_$file_aux

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
  if [ "$planner" == "SST" ] ; then
    planner="RRT"
  else
    if [ "$planner" == "RRT" ] ; then 
      planner="DIRT"
    fi
  fi
  ((cont_planner++))
done

echo "Aux file: "$file_aux
echo "Res file: "$file_

cat $home_$file_aux | awk '/Solution Quality/' >> $home_$file_
