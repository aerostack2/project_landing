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

# Launch aerostack2 ground station
eval "tmuxinator start -n ground_station -p tmuxinator/ground_station.yaml \
  keyboard_teleoperation=${keyboard_teleoperation} \
  rviz=${rviz} \
  rosbag=${rosbag}"
