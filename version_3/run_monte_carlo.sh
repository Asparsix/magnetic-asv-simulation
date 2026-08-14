#!/usr/bin/env bash
# Run Monte Carlo with version_3 Python packages (avoids parent workspace overlay).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
source /opt/ros/jazzy/setup.bash
source /home/robot/simulation_ws/install/setup.bash

# Prepend version_3 isolated package prefixes; drop parent boat_* duplicates.
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

V3_PY=(
  "$ROOT/install/boat_sensing/lib/python3.12/site-packages"
  "$ROOT/install/boat_mission/lib/python3.12/site-packages"
  "$ROOT/install/boat_mapping/lib/python3.12/site-packages"
  "$ROOT/src/boat_sensing"
  "$ROOT/src/boat_mission"
)
export PYTHONPATH="$(IFS=:; echo "${V3_PY[*]}")${PYTHONPATH:+:$PYTHONPATH}"

exec python3 "$ROOT/install/boat_bringup/lib/boat_bringup/monte_carlo_run.py" "$@"
