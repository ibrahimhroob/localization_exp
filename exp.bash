#!/bin/bash

######### BASH TERMINAL COLORS ################################################
# Black        0;30     Dark Gray     1;30
# Red          0;31     Light Red     1;31
# Green        0;32     Light Green   1;32
# Brown/Orange 0;33     Yellow        1;33
# Blue         0;34     Light Blue    1;34
# Purple       0;35     Light Purple  1;35
# Cyan         0;36     Light Cyan    1;36
# Light Gray   0;37     White         1;37

RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

printf "${PURPLE}***Script ID: NDT localization experiment***${NC}\n"

######### DIRECTORIES & FILES #################################################
CATKIN_WS=/home/computing/neptune/catkin_ws

FILTER_SCRIPT=/home/computing/neptune/inference_model/stability_filter.py

NDT_ODOM_TOPIC=/odom
NDT_STATUS_TOPIC=/status
GT_TOPIC=/odometry/gps

POINTS_TOPIC=/os_cloud_node/points
POINTS_TOPIC_FILTRED=/cloud_filtered

TYPE="raw"
PLAY_RATE=1   # Note the play rate will be overriden if the type is not raw

MAP_LOADING_TIME=2

# Set the IP address of the remote machine
REMOTE_IP=10.5.37.139
BAGS_REMOTE_DIR=/mnt/2564a3b0-8e42-43a4-b917-2b1af0c78052/bacchus/dataset/bags

# Source the ROS environment setup file
source $CATKIN_WS/devel/setup.bash

# Define a function for error messages
function error() {
  printf "${RED}Error: ${1}${NC}\n"
  exit 1
}

# Define function for killing ros nodes 
kill_ros(){
    # Kill all ROS nodes
    printf "${CYAN}Killing all ROS nodes${NC}\n"
    rosnode kill -a >/dev/null 2>&1 || true # Ignore errors if there are no nodes to kill
    sleep 1
}

# Check that the necessary dependencies are installed
command -v rosbag >/dev/null 2>&1 || error "rosbag command not found"
command -v roslaunch >/dev/null 2>&1 || error "roslaunch command not found"
command -v python >/dev/null 2>&1 || error "python command not found"

# Define an array of bags to process
bags=("june_1" "june_8" "june_29" "july_13")

for bag in "${bags[@]}"; do
    printf "Mapping: ${BLUE}${bag}${NC} ...\n"
    exp_dir=$bag
    log_dir=$exp_dir/log
    status_dir=$exp_dir/status
    hdl_odom_dir=$exp_dir/hdl_odom

    # Kill all ROS nodes
    kill_ros

    mkdir -p $log_dir
    mkdir -p $status_dir
    mkdir -p $hdl_odom_dir 

    odom_bag=$hdl_odom_dir/$bag
    printf "${CYAN}Recording ${NDT_ODOM_TOPIC}, ${NDT_STATUS_TOPIC} and ${GT_TOPIC} to ${odom_bag}.bag${NC}\n"
    rosbag record -O $odom_bag $NDT_ODOM_TOPIC $NDT_STATUS_TOPIC $GT_TOPIC &>$log_dir/rosbag_record.txt &
    
    # Run the filter in the background if the type is not raw 
    if [$TYPE -ne "raw"]; then
      printf "${CYAN}Running filter ...${NC}\n"
      nohup python $FILTER_SCRIPT --model ktima &>$log_dir/filter.txt &
      # Override the play rate of the rosbag this is due to the slow inference time
      PLAY_RATE=0.2
      POINTS_TOPIC=$POINTS_TOPIC_FILTRED
      sleep 5
    fi

    # Run fast_lio in the background
    printf "${CYAN}Running hdl-localization background ... ${NC}\n"
    nohup hdl_localization hdl_loc_bacchus.launch points_topic:=$POINTS_TOPIC &>$log_dir/hdl_log.txt & 
    sleep 5   

    bag_path=$BAGS_REMOTE_DIR/$bag.bag

    # Check if the bag file exists on the remote machine
    if ! ssh ibrahim@$REMOTE_IP "[ -f ${bag_path} ]"; then
        error "Bag file not found on remote machine: ${bag_path}"
    fi

    printf "${CYAN}Playing ${bag_path} on remote machine${NC}\n"
    ssh ibrahim@$REMOTE_IP "source /opt/ros/melodic/setup.bash && rosbag play ${bag_path} \
                            --topics $IMU_TOPIC $POINTS_TOPIC -r ${PLAY_RATE} --clock     \
                            --duration=640 " || error "Failed to play rosbag on remote machine"

    # Kill all ROS nodes
    kill_ros


    # Kill all background processes
    printf "${CYAN}Killing all background processes${NC}\n"
    kill $(jobs -p) >/dev/null 2>&1 || true # Ignore errors if there are no background processes to kill
    sleep 5

    # Convert the odometry topic to .tum format
    printf "${CYAN}Converting ${NDT_ODOM_TOPIC} to .tum format${NC}\n"
    evo_traj bag $odom_bag.bag $NDT_ODOM_TOPIC --save_as_tum || error "Failed to convert odometry topic to .tum format"
    mv odom.tum $hdl_odom_dir/${bag}_${TYPE}.tum

    printf "${CYAN}Converting ${GT_TOPIC} to .tum format${NC}\n"
    evo_traj bag $odom_bag.bag $GT_TOPIC --save_as_tum || error "Failed to convert odometry topic to .tum format"
    mv gps.tum $hdl_odom_dir/$bag.tum

    mv $odom_bag.bag $status_dir/${bag}_${TYPE}.bag || error "Failed to move ${odom_bag}.bag to ${status_dir}/${bag}_${TYPE}.bag"

    printf "======================================================\n"

done
