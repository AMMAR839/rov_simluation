#!/usr/bin/env bash
set -e

export ROV_WS="${ROV_WS:-/home/ammar/Documents/WS_ROV}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

source /opt/ros/jazzy/setup.bash
source "$ROV_WS/install/setup.bash"

echo "Ground station environment ready"
echo "ROV_WS=$ROV_WS"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
