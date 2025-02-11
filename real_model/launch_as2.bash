#!/bin/bash

usage() {
    echo "  options:"
    echo "      -r: record rosbag"
}

# Initialize variables with default values
rosbag="false"

# Arg parser
while getopts "r" opt; do
  case ${opt} in
    r )
      rosbag="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PWD/gz_resources/worlds
export GZ_SIM_RESOURCE_PATH=:$GZ_SIM_RESOURCE_PATH:$PWD/gz_resources/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/gz_resources/plugin_ws/install/plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD/gz_resources/plugin_ws/install/lib
export AS2_EXTRA_DRONE_MODELS=$AS2_EXTRA_DRONE_MODELS:zmr250

eval "tmuxinator start -n drone -p tmuxinator/aerostack2.yaml \
  rosbag=${rosbag}"