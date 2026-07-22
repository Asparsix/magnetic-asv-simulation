# NIOT Magnetic ASV Simulation — Version 3 (Dual ASV)

Version 3 is a standalone ROS 2 Jazzy + Gazebo Harmonic workspace for
**two cooperative ASVs** on a **300 m × 300 m** lake. It sits beside the
original root tree and `version_2/` (single-ASV); it does not replace them.

## What this version adds

- Two shoreline-started boats (SW / SE) with independent `/asv1` and `/asv2` stacks
- Voronoi regional lawnmower coverage (geographic halves, not interleaved lanes)
- Shared Bayesian belief map + dipole least-squares fix from both magnetometers
- Cooperative verify: discoverer HOLDs, peer approaches from the opposite side
- Live multi-boat trajectory / anomaly plot (red + blue)

## Quick start

```bash
cd version_3   # or open this folder as the workspace root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Full dual-ASV mission (coverage → hunt → cooperative verify)
ros2 launch boat_bringup multi_asv_phase4.launch.py
```

Phased demos: `multi_asv_phase1` (control), `phase2` (regions), `phase3` (shared belief).

Planted magnetic target for evaluation only: `(55, -35)` m. Estimators do not
read that pose; they localize from anomaly samples alone.
