#!/bin/bash
# Script to install the model datasets required
# to GeographicLib apply certain conversions

echo "start wait for $1 seconds"
sleep $1
echo "end wait for $1 seconds"
shift
    echo "now running 'roslaunch px4.launch'"
roslaunch mavros px4.launch
timedatectl set-ntp no
