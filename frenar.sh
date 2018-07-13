#!/bin/bash

# carro real
# rostopic pub -1 /manual_control/speed std_msgs/Int16 "data: 0"
# rostopic pub -1 /steering std_msgs/Int16 "data: 90"
# simulacion
rostopic pub -1 /AutoNOMOS_mini/manual_control/speed std_msgs/Int16 "data: 0"
rostopic pub -1 /AutoNOMOS_mini/manual_control/steering std_msgs/Int16 "data: 90"
