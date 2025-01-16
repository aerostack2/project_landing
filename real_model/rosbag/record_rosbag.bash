#!/bin/bash

usage() {
    echo "  Add drones namespaces as arguments, separated by commas"
}

# Get drone namespaces from command-line argument
drones_namespace_comma=${1:-"drone0"}
drone_namespaces=$(echo $drones_namespace_comma | tr "," " ")

echo "Recording rosbag for drones: ${drones_namespace_comma[@]}"

# Create directory for rosbags
mkdir rosbag/rosbags 2>/dev/null
cd rosbag/rosbags

# Construct the rosbag record command
rosbag_cmd="ros2 bag record"

# Uncomment the following lines to record specific topics
# Add topics and drone namespaces to the rosbag record command
# for drone_namespace in ${drone_namespaces[@]}; do
#   rosbag_cmd+=" /${drone_namespace}/sensor_measurements/imu \
#                 /${drone_namespace}/sensor_measurements/camera/camera_info \
#                 /${drone_namespace}/sensor_measurements/camera/image/compressed \
#                 /${drone_namespace}/raw_imu"
# done

# Add remaining topics
# rosbag_cmd+=" /mocap/rigid_bodies"

for drone_namespace in ${drone_namespaces[@]}; do
  rosbag_cmd+=" /${drone_namespace}/actuator_command/thrust \
                /${drone_namespace}/actuator_command/twist \
                /${drone_namespace}/debug/ref_traj_point \
                /${drone_namespace}/debug/traj_generated \
                /${drone_namespace}/motion_reference/trajectory \
                /${drone_namespace}/self_localization/pose \
                /${drone_namespace}/self_localization/twist \
                /${drone_namespace}/sensor_measurements/battery \
                /${drone_namespace}/sensor_measurements/imu \
                /${drone_namespace}/raw_imu \
                /${drone_namespace}/debug/rc"
done

# Add remaining topics
rosbag_cmd+=" /rosout /tf /tf_static /mocap/rigid_bodies"

# Comment the following line to record specific topics
# Record all topics
# rosbag_cmd+=" --all"

# Include hidden topics
# rosbag_cmd+="  --include-hidden-topics"

# Wait for key press to start recording
read -p "Press any key to start recording... " -n1 -s

echo "Recording rosbag..."

# Execute the rosbag record command
eval "$rosbag_cmd"