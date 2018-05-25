#!/bin/bash

rostopic pub -1 /steering std_msgs/Int16 "data: 90"
rostopic pub -1 /manual_control/speed std_msgs/Int16 "data: 0"
rostopic pub -1 /AutoNOMOS_mini/manual_control/steering std_msgs/Int16 "data: 45"
rostopic pub -1 /AutoNOMOS_mini/manual_control/speed std_msgs/Int16 "data: 0"
