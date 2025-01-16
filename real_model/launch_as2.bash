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

eval "tmuxinator start -n drone -p tmuxinator/aerostack2.yaml \
  rosbag=${rosbag}"