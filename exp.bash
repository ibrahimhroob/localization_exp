#!/bin/bash

######### BASH TERMINAL COLORS ################################################
declare -A COLORS=(
  [RED]='\033[0;31m'
  [BLUE]='\033[0;34m'
  [CYAN]='\033[0;36m'
  [GREEN]='\033[0;32m'
  [PURPLE]='\033[0;35m'
  [NC]='\033[0m' # No Color
)

color_printf() {
  printf "${COLORS[$1]}$2${COLORS[NC]}\n"
}

color_printf PURPLE "***Script ID: NDT localization experiment***"

######### DIRECTORIES & FILES #################################################
CATKIN_WS=/home/computing/neptune/catkin_ws
FILTER_SCRIPT=/home/computing/neptune/inference_model/stability_filter.py
NDT_ODOM_TOPIC=/odom
NDT_STATUS_TOPIC=/status
GT_TOPIC=/odometry/gps
POINTS_TOPIC_RAW=/os_cloud_node/points
POINTS_TOPIC_FILTRED=/cloud_filtered

TYPE="filtered" # raw || filtered
PLAY_RATE=1   # Note the play rate will be overridden if the type is not raw

DURATION=20 #640

REMOTE_IP=10.5.37.139
BAGS_REMOTE_DIR=/mnt/2564a3b0-8e42-43a4-b917-2b1af0c78052/bacchus/dataset/bags

source $CATKIN_WS/devel/setup.bash

error() {
  color_printf RED "Error: ${1}"
  exit 1
}

kill_ros(){
    color_printf CYAN "Killing all ROS nodes"
    rosnode kill -a >/dev/null 2>&1 || true
    sleep 1
}

command -v rosbag >/dev/null 2>&1 || error "rosbag command not found"
command -v roslaunch >/dev/null 2>&1 || error "roslaunch command not found"
command -v python >/dev/null 2>&1 || error "python command not found"

bags=("june_1" "june_8" "june_29" "july_13")

for bag in "${bags[@]}"; do
    color_printf "" "Mapping: ${COLORS[BLUE]}${bag}${COLORS[NC]} ..."

    exp_dir=$bag
    log_dir=$exp_dir/log
    status_dir=$exp_dir/status
    hdl_odom_dir=$exp_dir/hdl_odom

    kill_ros

    mkdir -p $log_dir $status_dir $hdl_odom_dir

    odom_bag=$hdl_odom_dir/$bag
    color_printf CYAN "Recording ${NDT_ODOM_TOPIC}, ${NDT_STATUS_TOPIC} and ${GT_TOPIC} to ${odom_bag}.bag"
    rosbag record -O $odom_bag $NDT_ODOM_TOPIC $NDT_STATUS_TOPIC $GT_TOPIC &>$log_dir/${TYPE}_rosbag_record.txt &

    LOC_POINT_TOPIC=$POINTS_TOPIC_RAW
    if [ $TYPE != "raw" ]; then
      color_printf CYAN "Running filter ..."
      nohup python $FILTER_SCRIPT --model ktima &>$log_dir/filter.txt &
      PLAY_RATE=0.2
      LOC_POINT_TOPIC=$POINTS_TOPIC_FILTRED
      color_printf GREEN "Points topic ${POINTS_TOPIC}"
      color_printf GREEN "Rosbag play rate set to: ${PLAY_RATE}"
      sleep 5
    fi

    color_printf CYAN "Running hdl-localization background ..."
    nohup roslaunch hdl_localization hdl_loc_bacchus.launch points_topic:=$LOC_POINT_TOPIC &>$log_dir/${TYPE}_hdl_log.txt &
    sleep 5

    bag_path=$BAGS_REMOTE_DIR/$bag.bag

    if ! ssh ibrahim@$REMOTE_IP "[ -f ${bag_path} ]"; then
        error "Bag file not found on remote machine: ${bag_path}"
    fi

    color_printf CYAN "Playing ${bag_path} on remote machine"
    ssh ibrahim@$REMOTE_IP "source /opt/ros/melodic/setup.bash && rosbag play ${bag_path} \
                            --topics $IMU_TOPIC $POINTS_TOPIC_RAW $GT_TOPIC -r ${PLAY_RATE} --clock     \
                            --duration=${DURATION} " || error "Failed to play rosbag on remote machine"

    kill_ros

    color_printf CYAN "Killing all background processes"
    kill $(jobs -p) >/dev/null 2>&1 || true
    sleep 5

    color_printf CYAN "Converting ${NDT_ODOM_TOPIC} to .tum format"
    evo_traj bag $odom_bag.bag $NDT_ODOM_TOPIC --save_as_tum || error "Failed to convert odometry topic to .tum format"
    mv odom.tum $hdl_odom_dir/${TYPE}.tum

    color_printf CYAN "Converting ${GT_TOPIC} to .tum format"
    evo_traj bag $odom_bag.bag $GT_TOPIC --save_as_tum || error "Failed to convert odometry topic to .tum format"
    mv odometry_gps.tum $hdl_odom_dir/.

    mv $odom_bag.bag $status_dir/${TYPE}.bag || error "Failed to move ${odom_bag}.bag to ${status_dir}/${bag}_${TYPE}.bag"

    printf "======================================================\n"

done 