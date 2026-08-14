# ROS 2 Node Catalog — Magnetic ASV Stack (`version_3`)

**Scope:** `/home/robot/simulation_ws/version_3`  
**Packages:** `boat_control`, `boat_sensing`, `boat_mapping`, `boat_mission`, `boat_navigation`, plus `boat_bringup` Gazebo bridges and `boat_msgs`.  
**Source of truth:** executable Python nodes, `setup.py` entry points, `declare_parameter` defaults, and `boat_bringup/config/*.yaml` overrides as of this tree.

**Topic naming convention used throughout**

| Form | Meaning |
|------|---------|
| *relative* | No leading `/` — resolved under the node's ROS namespace (e.g. node in `/asv1` → `cmd_vel` becomes `/asv1/cmd_vel`). |
| *absolute* | Leading `/` — global name, not remapped by namespace (e.g. `/swarm/belief/peak`, `/model/simple_boat/...`). |

**Typical launch namespaces** (`sim.launch.py`): ASV1 → `/asv1` + Gazebo model `simple_boat`; ASV2 → `/asv2` + `simple_boat_2`. Swarm fusion / verify coordinator run at root (no ASV namespace).

**Data-flow overview (one ASV + swarm)**

```
Gazebo ──(gz transport / ros_gz_bridge)──► odom, imu, gps, mag/gazebo, cmd_thrust
gazebo_pose2d ──► pose2d
los_path_follower ◄── odom, plan ──► cmd_vel ──► thrust_mixer ──► cmd_thrust
mag_driver ◄── mag/gazebo?, pose2d, cmd_vel ──► mag/raw
mag_filter ──► mag/filtered
calibration_node ──► mag/anomaly, calibration/status
bayes_fusion ◄── mag/anomaly(+) ──► /swarm/belief/*
mission_manager ◄── pose, belief, anomaly, cal ──► plan, mission/*, verify declare
verify_coordinator ◄── declare, result ──► request, halt, complete
trajectory_plotter (viz)
```

---

# Part I — Custom Messages (`boat_msgs`)

## `MagReading.msg`

| Field | Type | Meaning |
|-------|------|---------|
| `header` | `std_msgs/Header` | Stamp + `frame_id` of the magnetometer sample. |
| `bx`, `by`, `bz` | `float64` | Magnetic field components in nanotesla (nT). |
| `scalar` | `float64` | \(\sqrt{b_x^2+b_y^2+b_z^2}\) magnitude (nT). |
| `x`, `y` | `float64` | Planar world pose at sample time (m). |
| `heading` | `float64` | Yaw / heading at sample time (rad). |
| `motor_on` | `bool` | True when commanded thruster surge or yaw exceeds the driver’s motor threshold (for future motor-state bins). |

## `MagAnomaly.msg`

| Field | Type | Meaning |
|-------|------|---------|
| `header` | `std_msgs/Header` | Propagated from the filtered `MagReading`. |
| `raw_nt` | `float64` | Scalar field used as raw input to calibration (nT). |
| `baseline_nt` | `float64` | Estimated ambient baseline (nT); `nan` if not calibrated. |
| `cleaned_anomaly_nt` | `float64` | Spatially/temporally cleaned anomaly feature for Bayes / mission (nT). |
| `is_calibrated` | `bool` | True when a usable baseline estimate exists for this sample. |
| `heading_bin` | `int32` | Discrete heading bin used for baseline lookup. |
| `grid_cell_x`, `grid_cell_y` | `int32` | Baseline-map cell indices \((i,j)\). |
| `x`, `y`, `heading` | `float64` | Pose/heading of the sample (m, rad). |

## `CalibrationStatus.msg`

| Field | Type | Meaning |
|-------|------|---------|
| `header` | `std_msgs/Header` | Status stamp; `frame_id` set to `map`. |
| `phase` | `string` | `CALIBRATING` \| `READY` \| `DEGRADED`. |
| `cells_sampled` | `int32` | Number of baseline cells that have received samples. |
| `coverage_percent` | `float32` | Percent of grid cells sampled. |

## `BeliefGrid.msg`

| Field | Type | Meaning |
|-------|------|---------|
| `header` | `std_msgs/Header` | Map stamp; `frame_id` = `map`. |
| `resolution` | `float32` | Cell size (m). |
| `origin_x`, `origin_y` | `float64` | World coordinates of the grid origin (m). |
| `width`, `height` | `uint32` | Grid dimensions in cells. |
| `data` | `float64[]` | Row-major posterior probabilities; should sum ≈ 1. |

## `MissionState.msg`

| Field | Type | Meaning |
|-------|------|---------|
| `header` | `std_msgs/Header` | Snapshot stamp; `frame_id` = `map`. |
| `mode` | `string` | `GLOBAL_SEARCH` \| `TARGET_SEARCH` \| `HOLD` \| `VERIFY` \| `COMPLETE` (and related). |
| `hunt_phase` | `string` | Within target hunt: `INFO_GAIN` \| `SPIRAL`, else empty. |
| `peak_p` | `float64` | Latest swarm peak belief probability. |
| `peak_x`, `peak_y` | `float64` | Latest discrete peak cell centre (m). |
| `info_gain_steps` | `int32` | Number of MI replan steps taken in `INFO_GAIN`. |
| `confirmations` | `int32` | Verification confirmation count. |

## `VerifyRequest.msg`

| Field | Type | Meaning |
|-------|------|---------|
| `header` | `std_msgs/Header` | Request stamp; `frame_id` = `map`. |
| `discoverer_id` | `string` | ASV that declared the candidate (e.g. `asv1`). |
| `verifier_id` | `string` | Assigned verifier; empty on declare, filled by coordinator on request. |
| `candidate_x`, `candidate_y` | `float64` | Candidate location (weighted centroid / estimate) (m). |
| `candidate_peak_p` | `float64` | Peak probability at declaration. |
| `discoverer_x`, `discoverer_y` | `float64` | Discoverer pose at declaration (for opposite-side orbit approach). |

## `VerifyResult.msg`

| Field | Type | Meaning |
|-------|------|---------|
| `header` | `std_msgs/Header` | Result stamp; `frame_id` = `map`. |
| `success` | `bool` | True if required confirmations were collected. |
| `verifier_id` | `string` | ASV that performed verification. |
| `candidate_x`, `candidate_y` | `float64` | Candidate under verification (m). |
| `confirmations` | `int32` | Number of confirming readings. |
| `final_peak_p` | `float64` | Swarm peak probability at finish. |

---

# Part II — Gazebo ↔ ROS Bridges (`boat_bringup`)

Bridges are **not** Python package executables; they are `ros_gz_bridge` `parameter_bridge` nodes started from `sim.launch.py`.

## Bridge nodes

| Launch name | Executable | Config | When |
|-------------|------------|--------|------|
| `boat_bridge` | `ros_gz_bridge/parameter_bridge` | `config/bridge.yaml` | Always with `sim.launch.py` |
| `clock_bridge` | `ros_gz_bridge/parameter_bridge` | `config/clock_bridge.yaml` | Only when launch arg `fast:=true` |

**Dependencies:** Gazebo Sim world running (`gz sim … water_world.sdf`); thruster topics require `thrust_mixer` publishing on the matching `/model/.../cmd_thrust` names.

## `bridge.yaml` topic mappings

### ASV1 (`simple_boat`)

| ROS topic | GZ topic | ROS type | GZ type | Direction |
|-----------|----------|----------|---------|-----------|
| `/model/simple_boat/joint/left_propeller_joint/cmd_thrust` | same | `std_msgs/msg/Float64` | `gz.msgs.Double` | ROS_TO_GZ |
| `/model/simple_boat/joint/right_propeller_joint/cmd_thrust` | same | `std_msgs/msg/Float64` | `gz.msgs.Double` | ROS_TO_GZ |
| `/asv1/imu/data` | `/model/simple_boat/imu` | `sensor_msgs/msg/Imu` | `gz.msgs.IMU` | GZ_TO_ROS |
| `/asv1/gps/fix` | `/model/simple_boat/gps` | `sensor_msgs/msg/NavSatFix` | `gz.msgs.NavSat` | GZ_TO_ROS |
| `/asv1/mag/gazebo` | `/model/simple_boat/magnetometer` | `sensor_msgs/msg/MagneticField` | `gz.msgs.Magnetometer` | GZ_TO_ROS |
| `/asv1/odom` | `/model/simple_boat/odometry` | `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` | GZ_TO_ROS |

### ASV2 (`simple_boat_2`)

| ROS topic | GZ topic | ROS type | GZ type | Direction |
|-----------|----------|----------|---------|-----------|
| `/model/simple_boat_2/joint/left_propeller_joint/cmd_thrust` | same | `std_msgs/msg/Float64` | `gz.msgs.Double` | ROS_TO_GZ |
| `/model/simple_boat_2/joint/right_propeller_joint/cmd_thrust` | same | `std_msgs/msg/Float64` | `gz.msgs.Double` | ROS_TO_GZ |
| `/asv2/imu/data` | `/model/simple_boat_2/imu` | `sensor_msgs/msg/Imu` | `gz.msgs.IMU` | GZ_TO_ROS |
| `/asv2/gps/fix` | `/model/simple_boat_2/gps` | `sensor_msgs/msg/NavSatFix` | `gz.msgs.NavSat` | GZ_TO_ROS |
| `/asv2/mag/gazebo` | `/model/simple_boat_2/magnetometer` | `sensor_msgs/msg/MagneticField` | `gz.msgs.Magnetometer` | GZ_TO_ROS |
| `/asv2/odom` | `/model/simple_boat_2/odometry` | `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` | GZ_TO_ROS |

## `clock_bridge.yaml`

| ROS topic | GZ topic | ROS type | GZ type | Direction |
|-----------|----------|----------|---------|-----------|
| `/clock` | `/clock` | `rosgraph_msgs/msg/Clock` | `gz.msgs.Clock` | GZ_TO_ROS |

## Related config (not bridges)

`record_topics.yaml` lists bag topics for offline replay (`/asv1/pose2d`, mag chain, belief topics, etc.); it does not start a node by itself.

---

# Part III — Nodes by Package

---

## 1. `boat_control` — `thrust_mixer`

### 1.1 Identity

| Item | Value |
|------|-------|
| Package | `boat_control` |
| Entry point | `thrust_mixer = boat_control.mixer:main` |
| Class | `ThrustMixer` |
| File | `src/boat_control/boat_control/mixer.py` |
| Core helper | `boat_control.core.mix_thrust` |
| ROS node name | `thrust_mixer` |

### 1.2 Parameters

| Name | Code default | YAML / launch override |
|------|--------------|------------------------|
| `model_name` | `'simple_boat'` | Launch: `simple_boat` / `simple_boat_2` per ASV. **No** `boat_bringup/config` YAML. |
| `cmd_vel_topic` | `'cmd_vel'` | Launch: `'cmd_vel'` (relative). |
| `thrust_scale` | `50.0` | Launch: `50.0` |
| `turn_gain` | `1.0` | Launch: `1.0` |
| `max_thrust` | `100.0` | Launch: `100.0` |

### 1.3 Subscriptions

| Topic | Rel/Abs | Type | Callback purpose |
|-------|---------|------|------------------|
| `cmd_vel` (param) | relative | `geometry_msgs/Twist` | `on_cmd_vel`: convert surge `linear.x` and yaw rate `angular.z` into left/right thrust and publish immediately. |

### 1.4 Publishers

| Topic | Rel/Abs | Type | When |
|-------|---------|------|------|
| `/model/{model_name}/joint/left_propeller_joint/cmd_thrust` | **absolute** | `std_msgs/Float64` | On every `cmd_vel` |
| `/model/{model_name}/joint/right_propeller_joint/cmd_thrust` | **absolute** | `std_msgs/Float64` | On every `cmd_vel` |

### 1.5 Timers / QoS

- **Timers:** none (event-driven).
- **QoS:** depth 10, default (reliable, volatile).

### 1.6 Algorithm pipeline

- Read `linear.x` (surge) and `angular.z` (yaw).
- \(L = (u - r \cdot g) s\), \(R = (u + r \cdot g) s\) with scale \(s\), turn gain \(g\).
- Clamp each channel to \(\pm\) `max_thrust`.
- Publish left/right `Float64` thrust commands to Gazebo joints (via bridge).

### 1.7 Dependencies

- **Upstream:** `los_path_follower` (or teleop) publishing `cmd_vel`.
- **Downstream / infra:** `boat_bridge` mapping thrust topics into Gazebo; Gazebo model spawned.

---

## 2. `boat_sensing` — `mag_driver`

### 2.1 Identity

| Item | Value |
|------|-------|
| Entry point | `mag_driver = boat_sensing.mag_driver:main` |
| Class | `MagDriver` |
| File | `src/boat_sensing/boat_sensing/mag_driver.py` |

### 2.2 Parameters

| Name | Code default | `sensing.yaml` (`/**/mag_driver`) |
|------|--------------|-----------------------------------|
| `publish_rate_hz` | `20.0` | `20.0` |
| `gazebo_mag_topic` | `'mag/gazebo'` | `mag/gazebo` |
| `pose_topic` | `'pose2d'` | `pose2d` |
| `cmd_vel_topic` | `'cmd_vel'` | `cmd_vel` |
| `raw_topic` | `'mag/raw'` | `mag/raw` |
| `frame_id` | `'asv1/mag_link'` | Launch overrides to `{ns}/mag_link` |
| `motor_on_threshold` | `1.0e-3` | `0.001` |
| `plant_magnetic_target` | `True` | `true` |
| `target_x` | `80.0` | **`55.0`** |
| `target_y` | `-40.0` | **`-35.0`** |
| `dipole_strength_nt` | `1.5e12` | **`4.0e5`** |
| `dipole_soft_m` | `1.0` | **`20.0`** |
| `synthetic_background_nt` | `4.5e8` | **`45000.0`** |
| `synthetic_noise_nt` | `5.0e4` | **`3.0`** |

### 2.3 Subscriptions

| Topic | Rel/Abs | Type | Callback purpose |
|-------|---------|------|------------------|
| `mag/gazebo` | relative | `sensor_msgs/MagneticField` | Cache latest Gazebo magnetometer (used when planting disabled). |
| `pose2d` | relative | `geometry_msgs/Pose2D` | Cache pose for stamping / synthetic dipole. |
| `cmd_vel` | relative | `geometry_msgs/Twist` | Set `motor_on` if \|surge\| or \|yaw\| exceeds threshold. |

### 2.4 Publishers

| Topic | Rel/Abs | Type | When |
|-------|---------|------|------|
| `mag/raw` | relative | `boat_msgs/MagReading` | Timer at `publish_rate_hz` (skip if planted mode and no pose, or non-planted and no mag). |

### 2.5 Timers / QoS

- **Timer:** `1/publish_rate_hz` (default **20 Hz**) → `on_timer`.
- **QoS:** mag sub depth 50; others 10; default QoS.

### 2.6 Algorithm pipeline

- If `plant_magnetic_target`: require pose; compute \(A/(r^3+s^3)\) dipole anomaly; set \(b_z =\) background + anomaly + Gaussian noise; \(b_x=b_y=0\).
- Else: convert Gazebo Tesla → nT; attach pose if available (else NaN).
- Fill `MagReading` (axes, scalar, motor_on, pose, heading); publish.

### 2.7 Dependencies

- **Required for planted mode:** `gazebo_pose2d` (`pose2d`).
- **Optional:** Gazebo mag via bridge (`mag/gazebo`) when planting off; `cmd_vel` for `motor_on`.
- **Consumers:** `mag_filter`.

---

## 3. `boat_sensing` — `mag_filter`

### 3.1 Identity

| Item | Value |
|------|-------|
| Entry point | `mag_filter = boat_sensing.mag_filter:main` |
| Class | `MagFilter` |
| File | `src/boat_sensing/boat_sensing/mag_filter.py` |
| Core | `MagnetometerFilterChain` in `filter_core.py` |

### 3.2 Parameters

| Name | Code default | `sensing.yaml` (`/**/mag_filter`) |
|------|--------------|-----------------------------------|
| `raw_topic` | `'mag/raw'` | same |
| `filtered_topic` | `'mag/filtered'` | same |
| `status_topic` | `'mag/filter_status'` | same |
| `lowpass_window` | `5` | `5` |
| `spike_history` | `20` | `20` |
| `spike_n_sigma` | `3.0` | **`10.0`** |
| `min_std_nt` | `1.0` | `1.0` |
| `status_rate_hz` | `1.0` | `1.0` |

### 3.3 Subscriptions

| Topic | Rel/Abs | Type | Callback purpose |
|-------|---------|------|------------------|
| `mag/raw` | relative | `boat_msgs/MagReading` | Spike-reject then low-pass each axis/scalar; republish cleaned reading with pose fields copied. |

### 3.4 Publishers

| Topic | Rel/Abs | Type | When |
|-------|---------|------|------|
| `mag/filtered` | relative | `boat_msgs/MagReading` | On every accepted/filtered raw sample |
| `mag/filter_status` | relative | `diagnostic_msgs/DiagnosticStatus` | Timer at `status_rate_hz` |

### 3.5 Timers / QoS

- **Timer:** status at **1 Hz** (default).
- **QoS:** raw sub depth 50; pubs depth 10; default.

### 3.6 Algorithm pipeline

- Per axis + scalar: spike gate vs rolling mean ± `n_sigma`·std (min std floor); after 5 consecutive rejects, accept step change.
- Moving-average low-pass of accepted (or held) value.
- Copy pose / `motor_on` / header; publish; status reports counts and rejects.

### 3.7 Dependencies

- **Upstream:** `mag_driver`.
- **Downstream:** `calibration_node`.

---

## 4. `boat_sensing` — `calibration_node`

### 4.1 Identity

| Item | Value |
|------|-------|
| Entry point | `calibration_node = boat_sensing.calibration_node:main` |
| Class | `CalibrationNode` |
| File | `src/boat_sensing/boat_sensing/calibration_node.py` |
| Core | `MagneticCalibrator`, `BaselineMap`, `TemporalHighPass` |

### 4.2 Parameters

| Name | Code default | `sensing.yaml` (`/**/calibration_node`) |
|------|--------------|-----------------------------------------|
| `filtered_topic` | `'mag/filtered'` | same |
| `anomaly_topic` | `'mag/anomaly'` | same |
| `status_topic` | `'calibration/status'` | same |
| `area_size_m` | `300.0` | `300.0` |
| `origin_x` | `-150.0` | `-150.0` |
| `origin_y` | `-150.0` | `-150.0` |
| `cell_size_m` | `20.0` | **`10.0`** |
| `num_heading_bins` | `8` | `8` |
| `min_cell_samples` | `1` | `1` |
| `reject_residual_nt` | `5.0e7` | **`35.0`** |
| `temporal_window` | `12` | `12` |
| `noise_floor_nt` | `0.0` | `0.0` |
| `ready_coverage_percent` | `5.0` | `5.0` |
| `ready_min_cells` | `8` | `8` |
| `freeze_baseline_when_ready` | `True` | `true` |
| `status_rate_hz` | `1.0` | `1.0` |

### 4.3 Subscriptions

| Topic | Rel/Abs | Type | Callback purpose |
|-------|---------|------|------------------|
| `mag/filtered` | relative | `boat_msgs/MagReading` | Drop NaN pose; run calibrator; publish `MagAnomaly`. |

### 4.4 Publishers

| Topic | Rel/Abs | Type | When |
|-------|---------|------|------|
| `mag/anomaly` | relative | `boat_msgs/MagAnomaly` | Per valid filtered sample |
| `calibration/status` | relative | `boat_msgs/CalibrationStatus` | Timer at `status_rate_hz` |

### 4.5 Timers / QoS

- **Timer:** status **1 Hz**.
- **QoS:** filtered sub depth 50; default elsewhere.

### 4.6 Algorithm pipeline

- Update (or freeze) spatial baseline map keyed by cell × heading bin; reject learning when residual too large (near dipole).
- Estimate baseline (own bin → cell mean → neighbour median → global mean).
- Spatial anomaly = \|raw − baseline\|; temporal high-pass / track boost → `cleaned_anomaly_nt`.
- Phase → `READY` when coverage or cell count thresholds met; optionally freeze further baseline learning.
- Publish anomaly + periodic status.

### 4.7 Dependencies

- **Upstream:** `mag_filter` (and thus `mag_driver` + pose).
- **Downstream:** `bayes_fusion`, `mission_manager`, `trajectory_plotter`.

---

## 5. `boat_mapping` — `bayes_fusion`

### 5.1 Identity

| Item | Value |
|------|-------|
| Entry point | `bayes_fusion = boat_mapping.bayes_fusion:main` |
| Class | `BayesFusionNode` |
| File | `src/boat_mapping/boat_mapping/bayes_fusion.py` |
| Core | `BeliefMap` (`bayes_core.py`), `DipoleFitter` (`dipole_fit.py`) |
| Namespace | typically **root** (not under `/asv*`) |

### 5.2 Parameters

| Name | Code default | `mapping.yaml` (`bayes_fusion`) |
|------|--------------|---------------------------------|
| `anomaly_topics` | `['/asv1/mag/anomaly']` | **`['/asv1/mag/anomaly','/asv2/mag/anomaly']`** |
| `map_topic` | `/swarm/belief/map` | same |
| `peak_topic` | `/swarm/belief/peak` | same |
| `peak_probability_topic` | `/swarm/belief/peak_probability` | same |
| `centroid_topic` | `/swarm/belief/centroid` | same |
| `centroid_mass_topic` | `/swarm/belief/centroid_mass` | same |
| `centroid_spread_topic` | `/swarm/belief/centroid_spread` | same |
| `fix_topic` | `/swarm/belief/fix` | same |
| `fix_rms_topic` | `/swarm/belief/fix_rms` | same |
| `fix_samples_topic` | `/swarm/belief/fix_samples` | same |
| `publish_rate_hz` | `2.0` | `2.0` |
| `area_size_m` | `300.0` | `300.0` |
| `origin_x` / `origin_y` | `-150.0` | same |
| `cell_size_m` | `20.0` | **`10.0`** |
| `p_bg` | `0.05` | `0.05` |
| `p_max` | `0.95` | `0.95` |
| `d_half` | `30.0` | `30.0` |
| `hit_threshold_nt` | `5.0e7` | **`15.0`** |
| `miss_threshold_nt` | `1.0e6` | **`5.0`** |
| `hit_only` | `True` | `true` |
| `centroid_threshold_frac` | `0.5` | `0.5` |
| `dipole_fit_enable` | `True` | `true` |
| `dipole_soft_m` | `20.0` | `20.0` |
| `dipole_fit_min_anomaly_nt` | `10.0` | `10.0` |
| `dipole_fit_min_samples` | `12` | `12` |
| `dipole_fit_max_samples` | `400` | `400` |
| `dipole_fit_guess_strength_nt` | `4.0e5` | `4.0e5` |

### 5.3 Subscriptions

| Topic | Rel/Abs | Type | Callback purpose |
|-------|---------|------|------------------|
| each of `anomaly_topics` | absolute list | `boat_msgs/MagAnomaly` | Bayesian HIT/MISS update; buffer strong samples for dipole LS fit; log HITs. |

### 5.4 Publishers

| Topic | Rel/Abs | Type | When |
|-------|---------|------|------|
| `/swarm/belief/map` | absolute | `boat_msgs/BeliefGrid` | Timer `publish_rate_hz` |
| `/swarm/belief/peak` | absolute | `geometry_msgs/PoseStamped` | same |
| `/swarm/belief/peak_probability` | absolute | `std_msgs/Float64` | same |
| `/swarm/belief/centroid` | absolute | `geometry_msgs/PoseStamped` | same |
| `/swarm/belief/centroid_mass` | absolute | `std_msgs/Float64` | same |
| `/swarm/belief/centroid_spread` | absolute | `std_msgs/Float64` | same |
| `/swarm/belief/fix` | absolute | `geometry_msgs/PoseStamped` | Timer, only if last fit succeeded |
| `/swarm/belief/fix_rms` | absolute | `std_msgs/Float64` | with fix |
| `/swarm/belief/fix_samples` | absolute | `std_msgs/Float64` | with fix (sample count as float) |

### 5.5 Timers / QoS

- **Timer:** belief publish at **2 Hz** (default/yaml).
- **QoS:** anomaly sub depth 50; pubs depth 10; **no** transient_local.

### 5.6 Algorithm pipeline

- Skip uncalibrated samples (ABSTAIN).
- Classify anomaly: HIT if ≥ hit threshold; MISS if ≤ miss (ignored when `hit_only`); else ABSTAIN.
- For HIT: multiply each cell belief by \(P_{\mathrm{det}}(d)\) with soft \(1/d^3\) detection model; renormalize.
- Maintain weighted centroid of cells with belief ≥ `centroid_threshold_frac` × peak.
- Dipole fitter: buffer strong anomalies; LM fit \(A/(r^3+s^3)\) seeded from centroid → continuous fix.
- Periodically publish grid, peak, centroid, and optional fix.

### 5.7 Dependencies

- **Upstream:** one or more `calibration_node` anomaly streams.
- **Downstream:** `mission_manager`, `trajectory_plotter`.
- Launch condition: `mapping:=true` in `sim.launch.py`.

---

## 6. `boat_mission` — `mission_manager`

### 6.1 Identity

| Item | Value |
|------|-------|
| Entry point | `mission_manager = boat_mission.mission_manager:main` |
| Class | `MissionManager` |
| File | `src/boat_mission/boat_mission/mission_manager.py` |
| Helpers | `InfoGainPlanner`, path generators, `VerificationTracker` |

### 6.2 Parameters

Topic / ID parameters (code defaults; yaml under `/**/mission_manager` matches unless noted):

| Name | Code default | YAML notes |
|------|--------------|------------|
| `asv_id` | `''` (→ namespace or `asv1`) | `''` |
| `pose_topic` | `pose2d` | same |
| `plan_topic` | `plan` | same |
| `state_topic` | `mission/state` | same |
| `mode_topic` | `mission/mode` | same |
| `info_gain_topic` | `mission/info_gain` | same |
| `anomaly_topic` | `mag/anomaly` | same |
| `calibration_status_topic` | `calibration/status` | same |
| `peak_topic` | `/swarm/belief/peak` | same |
| `peak_probability_topic` | `/swarm/belief/peak_probability` | same |
| `centroid_topic` | `/swarm/belief/centroid` | same |
| `belief_map_topic` | `/swarm/belief/map` | same |
| `declare_topic` | `/swarm/verify/declare` | same |
| `verify_request_topic` | `/swarm/verify/request` | same |
| `verify_result_topic` | `/swarm/verify/result` | same |
| `halt_topic` | `/swarm/mission/halt` | same |

Behaviour / geometry (highlighting yaml overrides):

| Name | Code default | YAML |
|------|--------------|------|
| `min_x`…`max_y` | ±120 | same |
| `lawnmower_spacing` | `40.0` | **`20.0`** |
| `lawnmower_asv_index` | `0` | launch sets per ASV |
| `lawnmower_num_asvs` | `1` | launch sets to `num_asvs` |
| `lawnmower_partition` | `'voronoi'` | `voronoi` |
| `region_seeds` | `[-110,-110,110,-110]` | same (cleared if `num_asvs≤1`) |
| `control_rate_hz` | `2.0` | `2.0` |
| `p_enter_target_search` | `0.25` | same |
| `consecutive_high_p_required` | `4` | same |
| `peak_near_asv_radius_m` | `150.0` | same |
| `min_seconds_before_switch` | `20.0` | same |
| `require_calibration_ready` | `True` | same |
| `force_target_search` / `force_spiral_complete` / `force_verify` | `False` | same |
| `info_gain_*` | see code | yaml mirrors |
| `spiral_*` | see code | yaml mirrors |
| `self_verify` | `True` | launch: `True` if 1 ASV, `False` if 2 |
| `verify_orbit_radius_m` | `20.0` | same |
| `verify_orbit_points` | `12` | same |
| `verify_confirmations_required` | `4` | same |
| `verify_arrival_radius_m` | `30.0` | same |
| `verify_peak_tolerance_m` | `50.0` | same |
| `verify_confirmation_threshold_nt` | `5.0e7` | **`15.0`** |
| `verify_min_peak_probability` | `0.30` | same |

### 6.3 Subscriptions

| Topic | Rel/Abs | Type | Callback purpose |
|-------|---------|------|------------------|
| `pose2d` | relative | `Pose2D` | Own pose for planning / distance checks. |
| `/swarm/belief/peak` | absolute | `PoseStamped` | Discrete peak cell location. |
| `/swarm/belief/peak_probability` | absolute | `Float64` | Peak probability for mode switch / verify. |
| `/swarm/belief/centroid` | absolute | `PoseStamped` | Preferred target estimate (else peak). |
| `/swarm/belief/map` | absolute | `BeliefGrid` | Feed MI planner belief. |
| `mag/anomaly` | relative | `MagAnomaly` | Latest anomaly; during VERIFY, register confirmations. |
| `calibration/status` | relative | `CalibrationStatus` | Gate GLOBAL→TARGET on `READY`. |
| `/swarm/verify/request` | absolute | `VerifyRequest` | If addressed to this ASV, start VERIFY orbit. |
| `/swarm/mission/halt` | absolute | `Bool` | On true, enter COMPLETE. |

### 6.4 Publishers

| Topic | Rel/Abs | Type | When |
|-------|---------|------|------|
| `plan` | relative | `nav_msgs/Path` | Lawnmower / MI waypoint / spiral / orbit / hold; **TRANSIENT_LOCAL** |
| `mission/state` | relative | `MissionState` | Every control tick |
| `mission/mode` | relative | `std_msgs/String` | Every control tick |
| `mission/info_gain` | relative | `Float64` | Every control tick (last MI gain) |
| `/swarm/verify/declare` | absolute | `VerifyRequest` | Once after spiral → HOLD |
| `/swarm/verify/result` | absolute | `VerifyResult` | End of VERIFY; **TRANSIENT_LOCAL** |

### 6.5 Timers / QoS

- **Timer:** `control_rate_hz` → **2 Hz** state machine.
- **QoS:** `plan` and `verify_result`: Reliable + **TRANSIENT_LOCAL**, depth 1. Others depth 10/50 default.

### 6.6 Algorithm pipeline

- **GLOBAL_SEARCH:** publish partitioned lawnmower (voronoi regions or interleaved lanes); wait for cal ready, time, and consecutive high peak near ASV → TARGET_SEARCH.
- **TARGET_SEARCH / INFO_GAIN:** mutual-information waypoint selection on belief grid; replan every `info_gain_replan_period_s`; exit on max steps / low MI / near peak / confidence.
- **SPIRAL:** expanding spiral about centroid/peak; complete on duration + peak_p + proximity (or force); → HOLD + declare candidate.
- **HOLD:** wait for coordinator assignment unless `self_verify` starts VERIFY immediately.
- **VERIFY:** opposite-side orbit; count confirming anomalies (at site, peak aligned, strong, confident); publish result → COMPLETE.
- **COMPLETE / halt:** hold-in-place plan.

### 6.7 Dependencies

- `gazebo_pose2d`, `los_path_follower` (consumes `plan`), sensing chain + `bayes_fusion`, optionally `verify_coordinator`.
- Launch: `mission:=true` and `autonomy:=true` for LOS with `use_builtin_path:=false`.

---

## 7. `boat_mission` — `verify_coordinator`

### 7.1 Identity

| Item | Value |
|------|-------|
| Entry point | `verify_coordinator = boat_mission.verify_coordinator:main` |
| Class | `VerifyCoordinator` |
| File | `src/boat_mission/boat_mission/verify_coordinator.py` |
| Namespace | root (launched once, typically with ASV1 stack) |

### 7.2 Parameters

| Name | Code default | `mission.yaml` (`verify_coordinator`) |
|------|--------------|---------------------------------------|
| `declare_topic` | `/swarm/verify/declare` | same |
| `request_topic` | `/swarm/verify/request` | same |
| `result_topic` | `/swarm/verify/result` | same |
| `complete_topic` | `/swarm/mission/complete` | same |
| `halt_topic` | `/swarm/mission/halt` | same |
| `known_asvs` | `['asv1']` | **`['asv1','asv2']`** |
| `prefer_other_verifier` | `True` | `true` |

### 7.3 Subscriptions

| Topic | Rel/Abs | Type | Callback purpose |
|-------|---------|------|------------------|
| `/swarm/verify/declare` | absolute | `VerifyRequest` | Pick verifier (prefer other ASV); publish assigned request. |
| `/swarm/verify/result` | absolute | `VerifyResult` | On success: latch complete+halt; on failure: clear active request. |

### 7.4 Publishers

| Topic | Rel/Abs | Type | When |
|-------|---------|------|------|
| `/swarm/verify/request` | absolute | `VerifyRequest` | On accept declare |
| `/swarm/mission/complete` | absolute | `Bool` | Success; **TRANSIENT_LOCAL** |
| `/swarm/mission/halt` | absolute | `Bool` | Success; **TRANSIENT_LOCAL** |
| `/swarm/mission/status` | absolute (**hardcoded**) | `String` | Assignment / complete strings; **TRANSIENT_LOCAL** |

### 7.5 Timers / QoS

- **Timers:** none.
- **QoS:** complete / halt / status: Reliable + **TRANSIENT_LOCAL**, depth 1. Request/declare: depth 10 default.

### 7.6 Algorithm pipeline

- Ignore declares while busy or mission already complete.
- Choose verifier ≠ discoverer if possible from `known_asvs`; else self.
- Forward candidate + discoverer pose as `VerifyRequest`.
- On successful `VerifyResult`: publish `complete=true`, `halt=true`, status string.

### 7.7 Dependencies

- At least one `mission_manager` publishing declares / results.
- Peer `mission_manager` instances listening for requests / halt.

---

## 8. `boat_mission` — `spiral_demo`

### 8.1 Identity

| Item | Value |
|------|-------|
| Entry point | `spiral_demo = boat_mission.spiral_demo:main` |
| Class | `SpiralDemo` |
| File | `src/boat_mission/boat_mission/spiral_demo.py` |
| Launch | `spiral_demo.launch.py` (not full mission) |

### 8.2 Parameters

| Name | Code default | Launch override |
|------|--------------|-----------------|
| `pose_topic` | `pose2d` | — |
| `plan_topic` | `plan` | — |
| `center_x` / `center_y` | `30.0` / `20.0` | launch args (defaults −30 / −40) |
| `arrival_radius_m` | `8.0` | — |
| `spiral_ring_spacing_m` | `15.0` | launch |
| `spiral_max_radius_m` | `80.0` | launch (e.g. 45) |
| `spiral_step_spacing_m` | `10.0` | launch |
| `min_x`…`max_y` | ±120 | — |

**No** entry in `mission.yaml` for this node.

### 8.3 Subscriptions / publishers / timers

| | |
|--|--|
| Sub | relative `pose2d` (`Pose2D`) — track arrival at center |
| Pub | relative `plan` (`Path`), **TRANSIENT_LOCAL** — transit then production spiral |
| Timer | **2 Hz** (`0.5` s) phase machine |

### 8.4 Algorithm pipeline

- Wait for pose → publish transit path to `(center_x, center_y)`.
- When within `arrival_radius_m`, publish same expanding spiral as mission manager.
- Stay in SPIRAL phase (no further replans).

### 8.5 Dependencies

- `gazebo_pose2d`, `los_path_follower` with `use_builtin_path:=false`; Gazebo + thruster chain. Sensing/mapping/mission usually off in demo launch.

---

## 9. `boat_navigation` — `gazebo_pose2d`

### 9.1 Identity

| Item | Value |
|------|-------|
| Entry point | `gazebo_pose2d = boat_navigation.gazebo_pose2d:main` |
| Class | `GazeboPose2D` |
| File | `src/boat_navigation/boat_navigation/gazebo_pose2d.py` |

### 9.2 Parameters

| Name | Code default | Launch |
|------|--------------|--------|
| `model_name` | `'simple_boat'` | `simple_boat` / `simple_boat_2` |
| `world_name` | `'niot_world'` | `'niot_world'` |
| `pose_topic` | `'pose2d'` | `'pose2d'` |

**No** navigation.yaml entry for this node.

### 9.3 I/O

| Kind | Topic | Notes |
|------|-------|-------|
| GZ sub | `/world/{world_name}/dynamic_pose/info` | Gazebo Transport `Pose_V` (not ROS) |
| ROS pub | relative `pose2d` | `geometry_msgs/Pose2D` on each matching model pose |

### 9.4 Timers / QoS

- Event-driven from Gazebo callback; ROS pub depth 10 default.

### 9.5 Algorithm pipeline

- Subscribe Gazebo dynamic poses; find pose named `model_name`.
- Extract \(x,y\); yaw from quaternion; publish `Pose2D`.

### 9.6 Dependencies

- Gazebo world `niot_world` running with the model present. **Does not** use `ros_gz_bridge` for pose (direct gz transport).

---

## 10. `boat_navigation` — `los_path_follower`

### 10.1 Identity

| Item | Value |
|------|-------|
| Entry point | `los_path_follower = boat_navigation.los_path_follower:main` |
| Class | `LosPathFollower` |
| File | `src/boat_navigation/boat_navigation/los_path_follower.py` |
| Helpers | `guidance.py`, `pid.py`, `path_utils.py` |

### 10.2 Parameters

All mirrored in `navigation.yaml` (`los_path_follower`) with same values as code defaults, except launch may set `use_builtin_path: false` when mission is on.

| Name | Default |
|------|---------|
| `control_rate_hz` | `10.0` |
| `lookahead_m` | `15.0` |
| `accept_radius_m` | `8.0` |
| `pass_epsilon_m` | `2.0` |
| `loop` | `True` |
| `u_ref` | `0.35` |
| `u_max` | `0.5` |
| `r_max` | `0.3` |
| `patrol_half_size` | `120.0` |
| `pose_timeout_s` | `1.0` |
| `odom_topic` | `odom` |
| `plan_topic` | `plan` |
| `active_plan_topic` | `plan/active` |
| `cmd_vel_topic` | `cmd_vel` |
| `debug_topic` | `nav/debug` |
| `use_builtin_path` | `True` (false under mission) |
| `kp_yaw`, `ki_yaw`, `kd_yaw`, `integral_limit_yaw` | `1.5`, `0.05`, `0.2`, `1.0` |
| `kp_u`, `ki_u`, `kd_u`, `integral_limit_u` | `1.0`, `0.1`, `0.0`, `1.0` |
| `speed_mode` | `'open_loop'` (`'pid'` optional) |

### 10.3 Subscriptions

| Topic | Rel/Abs | Type | Callback purpose |
|-------|---------|------|------------------|
| `odom` | relative → e.g. `/asv1/odom` | `nav_msgs/Odometry` | Pose, yaw, measured surge; stamp for staleness. |
| `plan` | relative | `nav_msgs/Path` | Replace active path; reindex nearest segment; **TRANSIENT_LOCAL** sub. |

### 10.4 Publishers

| Topic | Rel/Abs | Type | When |
|-------|---------|------|------|
| `cmd_vel` | relative | `Twist` | Control timer (or stop) |
| `plan/active` | relative | `Path` | When path set; **TRANSIENT_LOCAL** |
| `nav/debug` | relative | `Float32MultiArray` | Each control tick: cross, along, heading, heading_err, surge, segment_index |

### 10.5 Timers / QoS

- **Timer:** **10 Hz** control loop.
- **QoS:** plan in/out Reliable + **TRANSIENT_LOCAL**, depth 1.

### 10.6 Algorithm pipeline

- Optional builtin square patrol if `use_builtin_path`.
- If no path / stale odom → publish zero Twist.
- LOS heading to lookahead point on current segment; PID yaw → `angular.z`.
- Speed: open-loop scaled by heading error, or closed-loop PID on surge.
- Advance segment on accept radius / pass geometry; loop or stop at end.

### 10.7 Dependencies

- Bridge providing `odom`; `thrust_mixer` on `cmd_vel`; plan source = `mission_manager` / `spiral_demo` / builtin path.
- Requires `autonomy:=true` in sim launch.

---

## 11. `boat_navigation` — `trajectory_plotter`

### 11.1 Identity

| Item | Value |
|------|-------|
| Entry point | `trajectory_plotter = boat_navigation.trajectory_plotter:main` |
| Class | `TrajectoryPlotter` |
| File | `src/boat_navigation/boat_navigation/trajectory_plotter.py` |

### 11.2 Parameters

No dedicated YAML file; `sim.launch.py` / multi-ASV block set key overlays. Code defaults:

| Name | Code default | Typical launch |
|------|--------------|----------------|
| `pose_topic` | `pose2d` | same |
| `trajectory_topic` | `trajectory` | same |
| `active_plan_topic` | `plan/active` | same |
| `anomaly_topic` | `mag/anomaly` | same |
| `asv_namespaces` | `['']` (single relative mode) | `['asv1','asv2']` multi |
| `peak_topic` | `/swarm/belief/peak` | same |
| `peak_probability_topic` | `/swarm/belief/peak_probability` | same |
| `centroid_topic` | `/swarm/belief/centroid` | same |
| `centroid_spread_topic` | `/swarm/belief/centroid_spread` | same |
| `fix_topic` / `fix_rms_topic` | `/swarm/belief/fix` / `..._rms` | same |
| `lake_half_size_m` | `150.0` | same |
| `max_history_points` | `20000` | — |
| `sample_distance_m` | `0.2` | — |
| `heading_arrow_m` | `12.0` | — |
| `show_true_target` | `True` | `True` |
| `target_x` / `target_y` | `80.0` / `-40.0` | **`55.0` / `-35.0`** (match planted dipole) |
| `mag_history_points` | `6000` | — |
| `anomaly_vmax_nt` | `55.0` | `55.0` |
| `anomaly_hit_threshold_nt` | `15.0` | `15.0` |
| `mission_state_topic` | `mission/state` | — |
| `verify_result_topic` | `/swarm/verify/result` | — |
| `mission_complete_topic` | `/swarm/mission/complete` | — |
| `mission_status_topic` | `/swarm/mission/status` | — |
| `verify_confirmations_required` | `4` | — |

### 11.3 Subscriptions (per boat + swarm)

**Per boat** (relative, or `/ns/...` when `asv_namespaces` set):

| Topic | Type | Purpose |
|-------|------|---------|
| `pose2d` | `Pose2D` | Track history / heading |
| `plan/active` | `Path` | Draw LOS route (**TRANSIENT_LOCAL**) |
| `mag/anomaly` | `MagAnomaly` | Mag scatter + time series |
| `mission/state` | `MissionState` | Phase banner |

**Swarm / absolute:**

| Topic | Type | Purpose |
|-------|------|---------|
| peak / peak_probability | `PoseStamped` / `Float64` | Peak overlay |
| centroid / centroid_spread | `PoseStamped` / `Float64` | Estimate + σ ring |
| fix / fix_rms | `PoseStamped` / `Float64` | Dipole fix |
| `/swarm/verify/result` | `VerifyResult` | Confirmation UI |
| `/swarm/mission/complete` | `Bool` | Latch confirmed (**TL**) |
| `/swarm/mission/status` | `String` | Status text (**TL**) |

### 11.4 Publishers / timers

| | |
|--|--|
| Pub | `{ns}/trajectory` or relative `trajectory` (`Path`), **TRANSIENT_LOCAL**, **1 Hz** |
| Timer | **5 Hz** (`0.2` s) matplotlib redraw; **1 Hz** trajectory publish |

### 11.5 Algorithm pipeline

- Maintain per-boat track, route, anomaly history.
- Overlay true target, centroid, peak cell, dipole fix, shore box.
- Status strip maps modes → SEARCHING / HUNTING / HOLD / VERIFYING / CONFIRMED.
- Visualization only — no control outputs.

### 11.6 Dependencies

- Optional: any combination of pose, plan, anomaly, belief, mission topics. Disabled when `headless` or `plot_trajectory:=false`.

---

# Part IV — Executable Index

| Package | Console script | Class | Source file |
|---------|----------------|-------|-------------|
| `boat_control` | `thrust_mixer` | `ThrustMixer` | `boat_control/mixer.py` |
| `boat_sensing` | `mag_driver` | `MagDriver` | `boat_sensing/mag_driver.py` |
| `boat_sensing` | `mag_filter` | `MagFilter` | `boat_sensing/mag_filter.py` |
| `boat_sensing` | `calibration_node` | `CalibrationNode` | `boat_sensing/calibration_node.py` |
| `boat_mapping` | `bayes_fusion` | `BayesFusionNode` | `boat_mapping/bayes_fusion.py` |
| `boat_mission` | `mission_manager` | `MissionManager` | `boat_mission/mission_manager.py` |
| `boat_mission` | `verify_coordinator` | `VerifyCoordinator` | `boat_mission/verify_coordinator.py` |
| `boat_mission` | `spiral_demo` | `SpiralDemo` | `boat_mission/spiral_demo.py` |
| `boat_navigation` | `gazebo_pose2d` | `GazeboPose2D` | `boat_navigation/gazebo_pose2d.py` |
| `boat_navigation` | `los_path_follower` | `LosPathFollower` | `boat_navigation/los_path_follower.py` |
| `boat_navigation` | `trajectory_plotter` | `TrajectoryPlotter` | `boat_navigation/trajectory_plotter.py` |
| `ros_gz_bridge` | `parameter_bridge` (`boat_bridge`) | (C++) | `config/bridge.yaml` |
| `ros_gz_bridge` | `parameter_bridge` (`clock_bridge`) | (C++) | `config/clock_bridge.yaml` |

---

# Part V — QoS Summary (non-default)

| Node | Topic(s) | Policy |
|------|----------|--------|
| `mission_manager` | `plan`, `/swarm/verify/result` | Reliable + TRANSIENT_LOCAL, depth 1 |
| `verify_coordinator` | `/swarm/mission/complete`, `halt`, `/swarm/mission/status` | Reliable + TRANSIENT_LOCAL, depth 1 |
| `spiral_demo` | `plan` | Reliable + TRANSIENT_LOCAL, depth 1 |
| `los_path_follower` | `plan` (sub), `plan/active` (pub) | Reliable + TRANSIENT_LOCAL, depth 1 |
| `trajectory_plotter` | `plan/active` (sub), `trajectory` (pub), complete/status (sub) | Reliable + TRANSIENT_LOCAL, depth 1 |

All other pubs/subs use rclpy defaults (typically Reliable, Volatile, depth as coded 10/20/50).

---

# Part VI — Minimum Runtime Graph (single ASV, full stack)

Must be running for a closed-loop magnetic search demo:

1. Gazebo (`niot_world` + `simple_boat`)
2. `boat_bridge` (+ `clock_bridge` if `use_sim_time`)
3. `/asv1/thrust_mixer`
4. `/asv1/gazebo_pose2d`
5. `/asv1/los_path_follower` (`use_builtin_path:=false`)
6. `/asv1/mag_driver` → `mag_filter` → `calibration_node`
7. `bayes_fusion`
8. `/asv1/mission_manager`
9. `verify_coordinator` (self-verify or multi-ASV)
10. Optional: `trajectory_plotter`

Two-ASV adds mirrored `/asv2/*` stack, dual anomaly topics into fusion, `self_verify:=false`, and peer verification via coordinator.

---

*End of catalog. Generated from source under `version_3/src`.*
