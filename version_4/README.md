# NIOT Magnetic ASV Simulation — Version 4 (1 km / 3 ASVs)

Version 4 is a standalone ROS 2 Jazzy + Gazebo Harmonic workspace for
**three cooperative ASVs** on a **1000 m × 1000 m** open lake (no central
island). It sits beside the root tree, `version_2/`, and `version_3/`; it
does not replace them.

## What this version adds

- Three shoreline-started boats: SW / SE / NE (`/asv1`, `/asv2`, `/asv3`)
- Voronoi regional lawnmower coverage across three geographic sectors
- Shared Bayesian belief map + dipole least-squares fix from all magnetometers
- Cooperative verify: discoverer HOLDs, another ASV approaches from the opposite side
- Live multi-boat trajectory / anomaly plot (red / blue / green)

## Quick start

```bash
cd version_4   # or open this folder as the workspace root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch boat_bringup multi_asv_phase4.launch.py
```

Coverage box is ±450 m with 50 m lawnmower spacing. Belief grid is 1000 m
at 20 m cells. Planted magnetic target for evaluation only: `(55, -35)` m —
estimators do not read that pose.
