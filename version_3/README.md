# NIOT Magnetic ASV Simulation — Version 3 (Dual ASV)

Version 3 is a standalone ROS 2 Jazzy + Gazebo Harmonic workspace for
**three cooperative ASVs** on a **300 m × 300 m** lake (also supports 1–2).
It sits beside the original root tree and `version_2/` (single-ASV); it does
not replace them.

## What this version adds

- Three boats spawn **on** their first lawnmower waypoint (west / center / east),
  heading east along the first leg
- Vertical-strip lawnmower coverage (20 m dead corridor; **15 m** leg spacing)
- Shared Bayesian belief map + dipole least-squares fix from all magnetometers
- Cooperative verify: discoverer HOLDs, peer approaches from the opposite side
- Live multi-boat trajectory / anomaly plot (red + blue + green)
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
