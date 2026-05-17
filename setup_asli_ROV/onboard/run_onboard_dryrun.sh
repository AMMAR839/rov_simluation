#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROV_WS="${ROV_WS:-/home/ammar/Documents/WS_ROV}"
source "$SCRIPT_DIR/../common/env_onboard.sh"

exec ros2 launch "$ROV_WS/setup_asli_ROV/onboard/onboard_rov.launch.py" \
  hardware_dry_run:=true \
  serial_port:="$ROV_SERIAL_PORT"
