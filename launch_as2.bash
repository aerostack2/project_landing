#!/bin/bash

# Add models to Gazebo sources
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PWD/gz_resources/worlds
export GZ_SIM_RESOURCE_PATH=:$GZ_SIM_RESOURCE_PATH:$PWD/gz_resources/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/gz_resources/plugin_ws/install/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Launch aerostack2
# eval "tmuxinator start -n drone -p tmuxinator/aerostack2.yaml"
ros2 launch as2_gazebo_assets launch_simulation.py use_sim_time:=true simulation_config_file:=config/world.yaml