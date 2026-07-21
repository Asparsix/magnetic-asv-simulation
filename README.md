# Standalone Boat Simulation

An independent ROS 2 Jazzy and Gazebo Harmonic simulation of the NIOT
differential-thrust boat. The workspace contains all model and world assets;
it does not require `niot_ws` at runtime.

The default world is a **300 m × 300 m lake** with shoreline collision,
a central island, dock, and marker buoys. Autonomous navigation uses
**LOS guidance + PID heading/speed control** to follow a built-in patrol
route or an external `nav_msgs/Path`.

ASV ROS topics are namespaced under **`/asv1/...`** (multi-ASV ready).
Custom interfaces live in the **`boat_msgs`** package.

## Build

```bash
cd /home/robot/simulation_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Start Gazebo, the ROS-Gazebo bridge, thrust mixer, and autonomous LOS follower:

```bash
ros2 launch boat_bringup sim.launch.py
```

The GUI launch also opens a live trajectory window showing:
- measured boat track,
- active LOS route,
- current position and heading arrow,
- lake shoreline and island.

Headless:

```bash
ros2 launch boat_bringup sim.launch.py headless:=true
```

Disable only the plot window:

```bash
ros2 launch boat_bringup sim.launch.py plot_trajectory:=false
```

Manual teleop mode (disables the LOS follower):

```bash
ros2 launch boat_bringup sim.launch.py autonomy:=false
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/asv1/cmd_vel
```

## Part 0 interfaces (`boat_msgs`)

| Message | Purpose |
|---------|---------|
| `MagReading` | Timestamped Bx/By/Bz + pose/heading |
| `MagAnomaly` | Cleaned anomaly for Bayes/mission |
| `CalibrationStatus` | CALIBRATING / READY / DEGRADED |
| `MissionState` | Mode, hunt phase, peak, confirmations |
| `BeliefGrid` | Shared probability map |
| `VerifyRequest` / `VerifyResult` | Swarm verification |

List them:

```bash
ros2 interface list | grep boat_msgs
```

## Autonomous navigation topics (`/asv1`)

| Topic | Type |
|-------|------|
| `/asv1/odom` | `nav_msgs/Odometry` |
| `/asv1/imu/data` | `sensor_msgs/Imu` |
| `/asv1/gps/fix` | `sensor_msgs/NavSatFix` |
| `/asv1/mag/gazebo` | `sensor_msgs/MagneticField` (Gazebo plugin) |
| `/asv1/mag/raw` | `boat_msgs/MagReading` (fixed-rate driver) |
| `/asv1/mag/filtered` | `boat_msgs/MagReading` (LPF + spike reject) |
| `/asv1/mag/anomaly` | `boat_msgs/MagAnomaly` (baseline-subtracted) |
| `/asv1/mag/filter_status` | `diagnostic_msgs/DiagnosticStatus` |
| `/asv1/calibration/status` | `boat_msgs/CalibrationStatus` |
| `/asv1/pose2d` | `geometry_msgs/Pose2D` |
| `/asv1/trajectory` | `nav_msgs/Path` |
| `/asv1/plan` | `nav_msgs/Path` (external override) |
| `/asv1/plan/active` | `nav_msgs/Path` |
| `/asv1/cmd_vel` | `geometry_msgs/Twist` |
| `/asv1/nav/debug` | `std_msgs/Float32MultiArray` |

## Part 1 sensing pipeline

```
/asv1/mag/gazebo  →  mag_driver (20 Hz)  →  /asv1/mag/raw
                                           ↓
                                      mag_filter
                                           ↓
                                    /asv1/mag/filtered
                                           ↓
                                   calibration_node
                                           ↓
                          /asv1/mag/anomaly + /asv1/calibration/status
```

- `mag_driver` converts Tesla → nT, stamps pose/heading, sets `motor_on` from `cmd_vel`
- `mag_driver` also overlays a planted `1/r³` magnetic dipole (Gazebo has no local mag sources)
- `mag_filter` applies spike rejection then moving-average low-pass
- `calibration_node` builds per-cell / heading-bin baseline, temporal high-pass, and cleaned anomaly

### Planted magnetic target

| Item | Value |
|------|-------|
| Position | `(-50, 70)` m (red marker midway between 20 m-spaced lawnmower legs; not driven over) |
| Model | Earth background + `A / (r³ + 1)` Bz dipole in `mag_driver` (Gazebo ambient bypassed while planted) |
| Strength | `dipole_strength_nt: 4.0e5`, `dipole_soft_m: 20` → **50 nT peak directly overhead**, ~44 nT @ 10 m, ~25 nT @ 20 m (HIT at 15 nT) |
| Background | `synthetic_background_nt: 45000` nT (Earth), noise `synthetic_noise_nt: 3` nT (1-sigma) |
| Grid | `cell_size_m: 10.0` (10 m × 10 m cells for calibration + belief map) |
| Enable | `plant_magnetic_target: true` in `sensing.yaml` |

```bash
# Disable the planted dipole (ambient Gazebo field only)
ros2 launch boat_bringup sim.launch.py sensing:=true
# then: ros2 param set /asv1/mag_driver plant_magnetic_target false
```

## Part 2 calibration topics

| Topic | Type |
|-------|------|
| `/asv1/mag/anomaly` | `boat_msgs/MagAnomaly` |
| `/asv1/calibration/status` | `boat_msgs/CalibrationStatus` (`CALIBRATING` / `READY`) |

Inspect:

```bash
ros2 topic echo /asv1/mag/anomaly --once
ros2 topic echo /asv1/calibration/status --once
```

## Part 3 Bayesian belief map

`bayes_fusion` consumes `/asv1/mag/anomaly` and publishes a shared swarm map:

| Topic | Type |
|-------|------|
| `/swarm/belief/map` | `boat_msgs/BeliefGrid` |
| `/swarm/belief/peak` | `geometry_msgs/PoseStamped` |
| `/swarm/belief/peak_probability` | `std_msgs/Float64` |

Model: uniform prior, HIT/MISS/ABSTAIN from anomaly thresholds, likelihood from `Pdet(d) = Pbg + K/(d³+C)`.

```bash
ros2 topic echo /swarm/belief/peak --once
ros2 topic echo /swarm/belief/peak_probability --once
ros2 launch boat_bringup sim.launch.py mapping:=false
```

## Part 4 mission manager + lawnmower

`mission_manager` (namespaced under `/asv1`) publishes a lawnmower coverage path to the LOS follower and switches modes from belief peak:

```
GLOBAL_SEARCH (lawnmower)
        │  peak_p ≥ P_enter for N consecutive ticks
        │  + peak near ASV + calibration READY
        ▼
TARGET_SEARCH
   ├── INFO_GAIN  (mutual information surfing)
   └── SPIRAL     (expanding rings around peak)
```

| Topic | Type |
|-------|------|
| `/asv1/mission/state` | `boat_msgs/MissionState` |
| `/asv1/mission/mode` | `std_msgs/String` |
| `/asv1/mission/info_gain` | `std_msgs/Float64` (last MI score) |
| `/asv1/plan` | `nav_msgs/Path` (mission → LOS) |

When `mission:=true` (default), LOS uses `use_builtin_path:=false` so the lawnmower owns the route.

```bash
ros2 topic echo /asv1/mission/state --once
ros2 topic echo /asv1/mission/mode --once
ros2 topic echo /asv1/plan --once
ros2 launch boat_bringup sim.launch.py mission:=false
```

## Part 5 mutual-information INFO_GAIN

During `TARGET_SEARCH` / `INFO_GAIN`, the mission manager:

1. Subscribes to `/swarm/belief/map`
2. Scores local ring candidates by expected entropy reduction (HIT/MISS MI)
3. Publishes the best waypoint on `/asv1/plan`
4. Exits to `SPIRAL` when near the peak, MI collapses, or max steps hit

Model matches `bayes_fusion` (`Pdet = Pbg + K/(d³+C)`). Config knobs live in `mission.yaml` (`info_gain_*`, `p_bg`, `p_max`, `d_half`).

```bash
ros2 topic echo /asv1/mission/info_gain --once
ros2 topic echo /asv1/mission/state --once   # hunt_phase: INFO_GAIN | SPIRAL
```

## Part 6 spiral + swarm VERIFY

After INFO_GAIN, the boat densifies with an expanding spiral around the belief peak, then declares a candidate. `verify_coordinator` assigns a verifier (same ASV for single-boat) which orbits and counts strong anomaly confirmations.

```
SPIRAL densify
   │  duration + peak_p + near peak
   ▼
HOLD + /swarm/verify/declare
   ▼
VERIFY orbit (confirmations)
   ▼
COMPLETE  (+ /swarm/mission/halt)
```

| Topic | Type |
|-------|------|
| `/swarm/verify/declare` | `boat_msgs/VerifyRequest` |
| `/swarm/verify/request` | `boat_msgs/VerifyRequest` |
| `/swarm/verify/result` | `boat_msgs/VerifyResult` |
| `/swarm/mission/complete` | `std_msgs/Bool` |
| `/swarm/mission/halt` | `std_msgs/Bool` |

```bash
ros2 topic echo /asv1/mission/mode --once          # VERIFY | COMPLETE
ros2 topic echo /swarm/verify/result --once
ros2 topic echo /swarm/mission/status --once
# Pipeline shortcut:
ros2 param set /asv1/mission_manager force_verify true
```

Force hunt mode for pipeline checks:

```bash
ros2 param set /asv1/mission_manager force_target_search true
```

Config: `src/boat_bringup/config/mission.yaml`.

Disable sensing only:

```bash
ros2 launch boat_bringup sim.launch.py sensing:=false
```

Inspect mag chain:

```bash
ros2 topic echo /asv1/mag/raw --once
ros2 topic echo /asv1/mag/filtered --once
ros2 topic hz /asv1/mag/raw
```

Thruster commands remain absolute Gazebo topics:
`/model/simple_boat/joint/{left,right}_propeller_joint/cmd_thrust`

Inspect:

```bash
ros2 topic echo /asv1/odom --once
ros2 topic echo /asv1/pose2d --once
ros2 topic echo /asv1/plan/active --once
```

Publish a custom route:

```bash
ros2 topic pub --once /asv1/plan nav_msgs/msg/Path "{
  header: {frame_id: map},
  poses: [
    {header: {frame_id: map}, pose: {position: {x: -80.0, y: -80.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 80.0, y: -80.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 80.0, y: 80.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: -80.0, y: 80.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: -80.0, y: -80.0}, orientation: {w: 1.0}}}
  ]
}"
```

## Tuning

Controller parameters live in
`src/boat_bringup/config/navigation.yaml`.

Use `Ctrl+C` in the launch terminal to stop Gazebo and all ROS nodes.
