#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROV_WS="${ROV_WS:-/home/ammar/Documents/WS_ROV}"
SERIAL_PORT="${1:-${ROV_SERIAL_PORT:-/dev/ttyACM0}}"
source "$SCRIPT_DIR/../common/env_onboard.sh"

echo "WARNING: real serial mode will send PWM to $SERIAL_PORT"
echo "Make sure propellers are removed during first tests."

exec ros2 launch "$ROV_WS/setup_asli_ROV/onboard/onboard_rov.launch.py" \
  hardware_dry_run:=false \
  serial_port:="$SERIAL_PORT"
