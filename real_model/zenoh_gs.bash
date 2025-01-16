#!/bin/bash

# Default value for the input parameter
DEFAULT_HOST="cvar-nx.local"

# Use the input parameter if provided, otherwise fall back to the default
HOST=${1:-$DEFAULT_HOST}

# Get the IP address of the host
IP=$(ping -c 1 "$HOST" | grep "PING" | awk -F'[()]' '{print $2}')

# Check if the IP address was found
if [ -z "$IP" ]; then
  echo "Could not retrieve the IP address for $HOST"
  exit 1
fi

zenoh-bridge-ros2dds -e tcp/$IP:7447 -c zenoh_config_gs.json5