#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env_onboard.sh"

echo
echo "ROS nodes:"
ros2 node list || true

echo
echo "ROV topics:"
ros2 topic list | sort | grep -E "/rov/(manual_cmd_vel|cmd_vel|thruster_pwm|hardware_pwm_status|gripper_cmd|camera|qr_code)" || true

echo
echo "One sample from /rov/hardware_pwm_status:"
timeout 3s ros2 topic echo --once /rov/hardware_pwm_status || true

echo
echo "One sample from /rov/manual_cmd_vel:"
timeout 3s ros2 topic echo --once /rov/manual_cmd_vel || true
