#!/usr/bin/env bash
# Repeatable 3-ASV GUI demo (fast). Known-good between-row target at (0, -85).
#
# Usage:
#   cd ~/simulation_ws/version_3
#   ./run_demo_gui.sh
#
# Optional:
#   ./run_demo_gui.sh --slow          # real-time Gazebo (no fast)
#   ./run_demo_gui.sh --headless      # no GUI (sanity check)

set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

FAST=true
HEADLESS=false
for arg in "$@"; do
  case "$arg" in
    --slow) FAST=false ;;
    --headless) HEADLESS=true ;;
    -h|--help)
      sed -n '2,12p' "$0"
      exit 0
      ;;
  esac
done

source /opt/ros/jazzy/setup.bash
source /home/robot/simulation_ws/install/setup.bash

# Prefer version_3 boat_* packages over parent workspace duplicates.
V3_PREFIXES=""
for pkg in boat_bringup boat_mission boat_mapping boat_sensing boat_navigation boat_msgs boat_description boat_control; do
  if [[ -d "$ROOT/install/$pkg" ]]; then
    V3_PREFIXES="$ROOT/install/$pkg:$V3_PREFIXES"
  fi
done
FILTERED=""
IFS=':' read -ra PARTS <<< "${AMENT_PREFIX_PATH:-}"
for p in "${PARTS[@]}"; do
  [[ -z "$p" ]] && continue
  [[ "$p" == "$ROOT/install" ]] && continue
  [[ "$p" == /home/robot/simulation_ws/install/boat_* ]] && continue
  FILTERED="${FILTERED:+$FILTERED:}$p"
done
export AMENT_PREFIX_PATH="${V3_PREFIXES}${FILTERED}"
export COLCON_PREFIX_PATH="$AMENT_PREFIX_PATH"

DEMO_YAML="$ROOT/src/boat_bringup/config/demo_target.yaml"
echo "GUI demo: target from $DEMO_YAML (default 0, -85 between lawnmower rows)"
echo "fast=$FAST headless=$HEADLESS"
exec ros2 launch boat_bringup multi_asv_phase4.launch.py \
  headless:="$HEADLESS" \
  fast:="$FAST" \
  plot_trajectory:=true \
  trial_params_file:="$DEMO_YAML"
