#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: select drones namespace to launch, values are comma separated. By default, it will get all drones from world description file"
    echo "      -s: if set, the simulation will not be launched. Default launch simulation"
    echo "      -g: launch using gnome-terminal instead of tmux. Default not set"
    echo "      -c: switch control mode to ACRO. Default is SPEED"
}

# Initialize variables with default values
drones_namespace_comma=""
launch_simulation="true"
use_gnome="false"
acro_control="false"

# Add models to Gazebo sources
export GZ_SIM_RESOURCE_PATH=$PWD/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=$PWD/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/plugin/install/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH


# Arg parser
while getopts "mn:sgc" opt; do
  case ${opt} in
    n )
      drones_namespace_comma="${OPTARG}"
      ;;
    s )
      launch_simulation="false"
      ;;
    g )
      use_gnome="true"
      ;; 
    c )
      acro_control="true"
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

# Set simulation world description config file according to control mode
if [[ ${acro_control} == "true" ]]; then
  simulation_config="config/world_simple.yaml"
  config_file="config/config_acro.yaml"
else
  simulation_config="config/world.yaml"
  config_file="config/config.yaml"
fi

# If no drone namespaces are provided, get them from the world description config file
if [ -z "$drones_namespace_comma" ]; then
  drones_namespace_comma=$(python3 utils/get_drones.py -p ${simulation_config} --sep ',')
fi
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 for each drone namespace
for namespace in ${drone_namespaces[@]}; do
  base_launch="false"
  if [[ ${namespace} == ${drone_namespaces[0]} && ${launch_simulation} == "true" ]]; then
    base_launch="true"
  fi
  eval "tmuxinator ${tmuxinator_mode} -n ${namespace} -p tmuxinator/aerostack2.yaml \
    drone_namespace=${namespace} \
    simulation_config_file=${simulation_config} \
    base_launch=${base_launch} \
    acro_control=${acro_control} \
    config_file=${config_file} \
    ${tmuxinator_end}"

  sleep 0.1 # Wait for tmuxinator to finish
done

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ${drone_namespaces[0]}
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi
