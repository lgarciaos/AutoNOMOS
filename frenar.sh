#!/bin/bash

rostopic pub -1 /AutoNOMOS_mini/manual_control/steering std_msgs/Int16 "data: 0"
rostopic pub -1 /AutoNOMOS_mini/manual_control/velocity std_msgs/Int16 "data: 0"
