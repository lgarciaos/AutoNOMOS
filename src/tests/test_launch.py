#!/usr/bin/env python

from time import sleep
import roslaunch




# # cli_args2 = ['autonomos_gazebo_simulation', 'curved_road.launch']
# cli_args3 = ['lane_navigation', 'simulador_gary_sinactuacion.launch']
# roslaunch_file3 = roslaunch.rlutil.resolve_launch_arguments(cli_args3)
# launch_files = [roslaunch_file3]

# launch_files = [(['/home/emartinene/git/AutoNOMOS/src/lane_navigation/launch/simulador_gary_sinactuacion.launch'], ['dbscan_epsilon:=24'])] # 25

# print (launch_files)

# launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

# launch.start()

# # sleep(30)

# # launch.shutdown()


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

node_localization = roslaunch.core.Node(
    package='lane_states_original',
    node_type='lane_states_node',
    name='lane_states_node',
    args='_car_center:=80 _image_height:=160 _state_width_pixels:=16 _model_num_gazebo:=1',
    output='screen')


# rosrun gazebo_ros spawn_model -database AutoNOMOS_mini -sdf -model AutoNOMOS_mini -x .75 -y 1.48 -z .16 -R 0 -P 0 -Y 1.5707

#empty_curved_road.launch
launch_files_gazebo = ['/home/emartinene/git/EK_AutoNOMOS_Sim/src/autonomos_gazebo_simulation/launch/straight_road.launch']
launch_gazebo = roslaunch.parent.ROSLaunchParent(uuid, launch_files_gazebo)

launch_files = ['/home/emartinene/git/AutoNOMOS/src/fu_line_detection/launch/line_detection_fu.launch']
launch_vision = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

launch_gazebo.start()

launch_vision.start()

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process2 = launch.launch(node_localization)
# do stuff

sleep(60)


launch_gazebo.stop()
launch_vision.stop()
launch.stop()