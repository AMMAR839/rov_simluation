#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROV_WS="${ROV_WS:-/home/ammar/Documents/WS_ROV}"
source "$SCRIPT_DIR/../common/env_ground_station.sh"

JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-/dev/input/js0}"
JOYSTICK_ENABLE_BUTTON="${JOYSTICK_ENABLE_BUTTON:-4}"
USE_VISION="${USE_VISION:-false}"
KKI_GUI="${KKI_GUI:-true}"

exec ros2 launch "$ROV_WS/setup_asli_ROV/gcs/gcs_station.launch.py" \
  joystick_device:="$JOYSTICK_DEVICE" \
  joystick_enable_button:="$JOYSTICK_ENABLE_BUTTON" \
  use_vision:="$USE_VISION" \
  kki_gui:="$KKI_GUI"
