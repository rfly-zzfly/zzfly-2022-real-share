#!/bin/bash

SH_DIR=$(dirname $0)
WS_DIR="${SH_DIR}/../.."

RUN_DATE=`date +%Y%m%d_%H%M%S`


d435i_topic="d435i/color/image_raw d435i/aligned_depth_to_color/image_raw"
mavros_topic="mavros/local_position/pose mavros/local_position/velocity_body mavros/local_position/velocity_local"
ctrl_topic="mavros/setpoint_raw/local mavros/setpoint_raw/attitude"
debug_topic="bs_debuger/task_state bs_debuger/image_res"
topic="${d435i_topic} ${mavros_topic} ${ctrl_topic} ${debug_topic}"

sleep 10s

gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:115200";exec bash"
sleep 5s

gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_t265_bridge.launch;exec bash"
sleep 5s

gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_d435i_30hz.launch;exec bash"
sleep 5s

gnome-terminal --window -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_main.launch;exec bash"

sleep 10s


gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;rosbag record ${topic} -o ${WS_DIR}/bag/zzfly_real_;exec bash"
