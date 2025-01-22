#!/bin/bash

usage() {
    echo "  options:"
    echo "      -v: open rviz. Default launch"
    echo "      -t: use keyboard teleoperation. Default not launch"
    echo "      -r: record rosbag. Default not launch"
}

# Initialize variables with default values
rviz="true"
keyboard_teleoperation="false"
rosbag="false"
use_sim_time="false"

# Parse command line arguments
while getopts "vtrs" opt; do
  case ${opt} in
    v )
      rviz="false"
      ;;
    t )
      keyboard_teleoperation="true"
      ;;
    r )
      rosbag="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# Add models to Gazebo sources
# export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PWD/gz_resources/worlds
export GZ_SIM_RESOURCE_PATH=:$GZ_SIM_RESOURCE_PATH:$PWD/gz_resources/models
export AS2_EXTRA_DRONE_MODELS=$AS2_EXTRA_DRONE_MODELS:zmr250

ros2 launch as2_visualization swarm_viz.launch.py namespace_list:=drone0 rviz_config:=config/rviz2_config.rviz drone_model:=zmr250 use_sim_time:=true

# # Launch aerostack2 ground station
# eval "tmuxinator start -n ground_station -p tmuxinator/ground_station.yaml \
#   keyboard_teleoperation=${keyboard_teleoperation} \
#   rviz=${rviz} \
#   rosbag=${rosbag}"
