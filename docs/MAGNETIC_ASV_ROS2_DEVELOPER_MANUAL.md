# Cooperative Magnetic ASV Search — ROS 2 Developer Manual

**Reference stack:** `/home/robot/simulation_ws/version_3`  
**Middleware:** ROS 2 Jazzy · Gazebo Harmonic · `ros_gz_bridge`  
**Primary demo launch:** `ros2 launch boat_bringup multi_asv_phase4.launch.py`  
**Audience:** ROS developers who need strategy, math, topic graph, node I/O, and step-by-step runtime behaviour.

This manual explains *what the system is trying to do*, *why each algorithm exists*, *how the ROS graph is wired*, and *what happens second-by-second* from spawn to `MISSION_COMPLETE`. Companion artifacts:

| Artifact | Path | Role |
|----------|------|------|
| This manual | `docs/MAGNETIC_ASV_ROS2_DEVELOPER_MANUAL.md` | Narrative strategy + workflow + math |
| Exhaustive node catalog | `version_3/docs/ros2_node_catalog.md` | Every param / pub / sub / QoS |
| Overleaf book | `docs/overleaf_magnetic_asv/` | PDF-oriented chapter set (upload whole folder) |

**Numbers below match `version_3` YAML** (`sensing.yaml`, `mapping.yaml`, `mission.yaml`, `navigation.yaml`) as of the strip-coverage 3-ASV stack. Do not mix with root `src/` (1 km Voronoi twin) without rescaling.

---

## Table of contents

1. [Mission concept](#1-mission-concept)
2. [Workspace layout and packages](#2-workspace-layout-and-packages)
3. [Coordinate frames, namespaces, QoS](#3-coordinate-frames-namespaces-qos)
4. [End-to-end architecture](#4-end-to-end-architecture)
5. [Step-by-step runtime workflow](#5-step-by-step-runtime-workflow)
6. [Mathematical models](#6-mathematical-models)
7. [ROS nodes (developer digest)](#7-ros-nodes-developer-digest)
8. [Complete topic catalog](#8-complete-topic-catalog)
9. [Custom messages](#9-custom-messages)
10. [Mission FSM and verify handshake](#10-mission-fsm-and-verify-handshake)
11. [Path generation and multi-ASV deconfliction](#11-path-generation-and-multi-asv-deconfliction)
12. [Launches, configs, and operator checklist](#12-launches-configs-and-operator-checklist)
13. [Acceptance, scoring, and common failure modes](#13-acceptance-scoring-and-common-failure-modes)
14. [File index (code map)](#14-file-index-code-map)

---

## 1. Mission concept

### 1.1 Problem statement

Find a **compact magnetic dipole-like target** on a bounded lake using one or more **differential-thrust ASVs**. The vehicle(s) never receive the ground-truth target coordinates. They only observe magnetometer readings that may contain a small anomaly on top of Earth’s field (~45 000 nT) and sensor noise (~3 nT).

In simulation the anomaly is **planted** for evaluation as a **vector dipole** plus a uniform Earth field (ENU). The scalar magnetometer reading is \(|\mathbf{B}|\); Bayes/MI still use the cleaned scalar \(|a|\). Default plant (YAML): \(\mathbf{t}^{\star}=(85,40,-1)\,\mathrm{m}\) (1 m depth), overhead \(\Delta F\approx 50\,\mathrm{nT}\), \(I=15^\circ\), \(D=-1^\circ\). Legacy \(A/(r^3+s^3)\) remains available via `dipole_model: scalar_soft`.

### 1.2 Strategy in one paragraph

1. **Partition the lake** into non-overlapping vertical **strips** (with a dead corridor) and fly **lawnmower** coverage.  
2. Fusion builds a **shared Bayesian occupancy** of “where is the dipole?” from cleaned anomalies.  
3. When belief concentrates near an ASV, that boat leaves coverage for **information-gain surfing**, then an **expanding spiral** to densify samples.  
4. The discoverer **declares** a candidate; a **peer verifier** (preferred) flies an **orbit**, counting confirmations.  
5. On enough confirmations the swarm **halts**; operators score localization vs planted \(\mathbf{t}^{\star}\) offline.

### 1.3 Design principles (for reviewers)

| Principle | Implementation |
|-----------|----------------|
| Identical per-boat code | Same nodes under `/asv1`, `/asv2`, `/asv3`; only `asv_id` / strip index differ |
| Shared belief | One `bayes_fusion` at root ingesting all `/asvN/mag/anomaly` |
| Discovery ≠ confirmation | `declare` / `request` / `result` via `verify_coordinator` |
| Keep boats in the lake | Spiral / verify orbit radii **shrunk to fit** coverage box with inland inset (no shoreline wall-clamping) |
| HOLD means stop | Empty `plan` clears LOS → zero `cmd_vel` |

---

## 2. Workspace layout and packages

```
version_3/
  src/
    boat_bringup/        # launches, YAML, bridges, bag record/replay
    boat_description/    # SDF models simple_boat{,_2,_3}, water_world.sdf
    boat_msgs/           # Mag*, BeliefGrid, MissionState, Verify*
    boat_control/        # thrust_mixer
    boat_sensing/        # mag_driver, mag_filter, calibration_node, dipole.py
    boat_mapping/        # bayes_fusion, bayes_core, dipole_fit
    boat_mission/        # mission_manager, verify_coordinator, path_planning, info_gain, verify_core
    boat_navigation/     # gazebo_pose2d, los_path_follower, trajectory_plotter, guidance
```

**Versions in this monorepo**

| Tree | Lake | Boats | Coverage default |
|------|------|-------|------------------|
| `version_3` | 300×300 m | up to 3 | **strips**, 10 m row spacing |
| root `src/` / `version_4` | ~1 km | up to 3 | **Voronoi**, larger boxes |

Use **one tree at a time**. Mixing leftover processes across launches causes phantom CONFIRMED / dead odom / ghosts (see §13).

---

## 3. Coordinate frames, namespaces, QoS

### 3.1 Map / lake

- World plan: ENU-like \(x\) east, \(y\) north (m).  
- Gazebo shoreline walls near \(\pm155\,\mathrm{m}\).  
- Plotter draws lake half-size **150 m**.  
- Mission coverage box (lawnmower / spiral bounds): **\(\pm120\,\mathrm{m}\)**.  
- Belief grid: 300 m extent, origin \((-150,-150)\), cell **10 m** → \(30\times30\) cells.

### 3.2 Namespaces

| Pattern | Meaning |
|---------|---------|
| Relative topic `cmd_vel` on node in `/asv2` | Resolves to `/asv2/cmd_vel` |
| Absolute `/swarm/belief/peak` | Same for every ASV |
| Absolute `/model/simple_boat_3/.../cmd_thrust` | Gazebo actuator (not under `/asvN`) |

ASV ↔ Gazebo model mapping (`sim.launch.py`):

| Namespace | Model |
|-----------|--------|
| `/asv1` | `simple_boat` |
| `/asv2` | `simple_boat_2` |
| `/asv3` | `simple_boat_3` |

### 3.3 QoS of note

| Topic family | Durability | Why |
|--------------|------------|-----|
| `plan`, `plan/active` | TRANSIENT_LOCAL | New LOS subscriber gets last path |
| `/swarm/mission/complete`, `halt`, `status` | TRANSIENT_LOCAL | Late joiners see terminal state — **clear domain / kill old nodes before relaunch** |
| `/swarm/verify/result` | TRANSIENT_LOCAL | Coordinator / plotter latch |
| Sensor streams | VOLATILE | Continuous |

---

## 4. End-to-end architecture

```
                    ┌─────────────────────────────────────────────┐
                    │                 Gazebo Harmonic             │
                    │  buoyancy / boat SDF / shore walls / clock  │
                    └───────────────┬─────────────────────────────┘
              gz: odom,imu,gps,mag  │  ◄── cmd_thrust
                    ros_gz_bridge   │
                                    ▼
 /asvN/odom ──────────────────────────────────────────────┐
 /asvN/mag/gazebo                                         │
        │                                                 │
        ▼                                                 │
 gazebo_pose2d ──► pose2d                                 │
        │                                                 │
        ▼                                                 ▼
 mag_driver ──► mag/raw ──► mag_filter ──► mag/filtered    │
                                      │                   │
                                      ▼                   │
                              calibration_node            │
                           mag/anomaly · cal/status       │
                                      │                   │
          ┌───────────────────────────┴──────────────┐    │
          ▼                                          │    │
   bayes_fusion (root)                               │    │
   /swarm/belief/{map,peak,centroid,fix,...}         │    │
          │                                          │    │
          ▼                                          ▼    │
   mission_manager ◄── pose, anomaly, cal, belief ──────┤    │
          │ publishes plan                               │
          ▼                                              │
   los_path_follower ◄── odom ──► cmd_vel ───────────────┘
          │
          ▼
   thrust_mixer ──► /model/.../cmd_thrust

   verify_coordinator (root, once):
      declare ──► request ──► result ──► complete/halt/status
```

**Singleton (root) nodes:** `bayes_fusion`, `verify_coordinator`, multi-ASV `trajectory_plotter`.  
**Per-ASV nodes:** `thrust_mixer`, `gazebo_pose2d`, `los_path_follower`, sensing triad, `mission_manager`.

---

## 5. Step-by-step runtime workflow

This is the operator / debugger story for **Phase 4** (`num_asvs:=3`).

### Phase 0 — Process bring-up (`sim.launch.py`)

1. Export `GZ_SIM_RESOURCE_PATH` for boat models.  
2. Start `gz sim -r water_world.sdf` (fast mode may rewrite SDF for RTF + smaller step).  
3. Start `boat_bridge` (`bridge.yaml`) and optional `clock_bridge` when `fast:=true`.  
4. For each ASV namespace: spawn ROS stack in order (mixer → pose → LOS → sensing → mission).  
5. Start `verify_coordinator` once (attached to ASV1 launch group historically, but topics are absolute).  
6. Start shared plotter (if GUI).  
7. Start `bayes_fusion`.

Spawn poses (typical strip seeds, yaw east):  

| ASV | Approx spawn | Strip |
|-----|--------------|-------|
| ASV1 | \((-120,-120)\) | west |
| ASV2 | \((-33.3,-120)\) | center |
| ASV3 | \((53.3,-120)\) | east |

### Phase A — GLOBAL_SEARCH (lawnmower)

1. Each `mission_manager` waits for `pose2d`, builds a **strip lawnmower** (`lawnmower_partition: strips`, `strip_gap_m: 20`, `lawnmower_spacing: 10`).  
2. Publishes TRANSIENT_LOCAL `plan`.  
3. `los_path_follower` tracks waypoints with LOS + open-loop surge \(u_{\mathrm{ref}}=0.35\,\mathrm{m/s}\).  
4. Sensing chain runs continuously; calibration accumulates baseline cells.  
5. `bayes_fusion` ignores weak anomalies (`hit_only`, HIT if cleaned anomaly \(\ge 15\,\mathrm{nT}\)).

**Mode exit (to TARGET_SEARCH):** all of

- calibration `READY` (if `require_calibration_ready`),  
- mission age \(\ge 20\,\mathrm{s}\),  
- for **4 consecutive** control ticks: `peak_p ≥ 0.25` **and** distance to estimate (prefer centroid) \(\le 150\,\mathrm{m}\).

### Phase B — TARGET_SEARCH / INFO_GAIN

1. Planner loads belief grid from `/swarm/belief/map`.  
2. Every ~5 s, score ring candidates at radii \(\{0.5,1,2\}\,\mathrm{m}\), 16 angles, clipped to coverage box.  
3. Mutual information (entropy reduction under HIT/MISS model) picks next waypoint → `plan`.  
4. Leave INFO_GAIN when: step budget (45), MI collapses, near estimate (\(\le 5\,\mathrm{m}\)), or high confidence near peak (`peak_p≥0.5` within ~1.5× convergence radius).

### Phase C — TARGET_SEARCH / SPIRAL

1. Build expanding spiral about **estimate** (centroid preferred).  
2. Radius capped by:
   \[
   r_{\max}^{\mathrm{use}}=\min\big(8,\ \mathrm{room}(\mathrm{center},\mathrm{box})-10\big)
   \]
   with `path_inland_inset_m: 10` so rings never hug \(\pm120\) (avoids Gazebo shore flings).  
3. Complete → **HOLD** when elapsed \(\ge 45\,\mathrm{s}\) and peak/proximity gates, or \(2\times\) time fallback.

### Phase D — HOLD + declare

1. Discoverer publishes empty `plan` (**hard stop**).  
2. Publishes `/swarm/verify/declare` (`VerifyRequest`, `verifier_id=""`).  
3. Multi-ASV: `self_verify: false` → wait for coordinator (do **not** self-orbit the shoreline).  
4. Non-assigned boats that hear a verify **request for someone else** also enter HOLD and clear path (prevents mid-spiral collisions).

### Phase E — VERIFY (peer)

1. `verify_coordinator` prefers a **different** idle ASV (`prefer_other_verifier: true`).  
2. Publishes `/swarm/verify/request` with filled `verifier_id`.  
3. Verifier builds orbit (radius ≤ fit inside inset box), `start_angle` opposite discoverer; prepends **transit hops** (~40 m) from current pose.  
4. Each `MagAnomaly` while VERIFY may increment confirmations if all hold:

| Gate | Default |
|------|---------|
| Distance to candidate | \(\le 30\,\mathrm{m}\) |
| Belief peak near candidate | \(\le 50\,\mathrm{m}\) |
| Cleaned anomaly | \(\ge 15\,\mathrm{nT}\) |
| Peak probability | \(\ge 0.30\) |

5. Need **4** confirmations → `/swarm/verify/result` `success=true`.

### Phase F — COMPLETE

1. Coordinator latches `/swarm/mission/complete` and `/swarm/mission/halt`.  
2. All managers enter COMPLETE / stop.  
3. Plotter banner: **CONFIRMED**.  
4. Offline score: `\|fix - t*\|` from `/swarm/belief/fix` (dipole LS), optionally vs centroid.

---

## 6. Mathematical models

### 6.1 Planted dipole (simulation only)

Code: `boat_sensing/dipole.py`, parameters in `sensing.yaml`. Default `dipole_model: total_field`.

Earth field (ENU, \(x\) east, \(y\) north, \(z\) up):

\[
\mathbf{F}=F(\cos I\sin D,\ \cos I\cos D,\ -\sin I),\quad
F=45\,000\,\mathrm{nT},\ I=15^\circ,\ D=-1^\circ.
\]

Vector dipole (\(\mu_0/4\pi\) absorbed into \(\mathbf{m}\) in nT·m\(^3\)):

\[
\mathbf{B}_d(\mathbf{r})
=\frac{3(\mathbf{m}\cdot\hat{\mathbf{r}})\hat{\mathbf{r}}-\mathbf{m}}{r^{3}},
\qquad
\mathbf{r}=\mathbf{p}-\mathbf{t},\ \mathbf{t}=(t_x,t_y,t_z).
\]

Published 3-axis field and scalar:

\[
\mathbf{B}_{\mathrm{raw}}=\mathbf{F}+\mathbf{B}_d+\boldsymbol{\eta},\quad
\eta_i\sim\mathcal{N}(0,3^{2})\,\mathrm{nT},\quad
\text{scalar }=|\mathbf{B}_{\mathrm{raw}}|.
\]

First-order total-field anomaly \(\Delta F=\hat{\mathbf{F}}\cdot\mathbf{B}_d\) with regularized denominator \(r^{3}+s^{3}\), \(s=20\,\mathrm{m}\). A vertical \(\mathbf{m}\) is auto-sized so overhead \(\Delta F\approx 50\,\mathrm{nT}\) at \(t_z=-1\,\mathrm{m}\). The footprint \(s\) keeps HIT \(\ge 15\,\mathrm{nT}\) out to \(\sim 20\,\mathrm{m}\), so a \(10\,\mathrm{m}\) lawnmower still sees the target. Set `dipole_mx/my/mz` explicitly to plant a tilted moment. Fallback: `dipole_model: scalar_soft` restores scalar \(A/(r^3+s^3)\) on \(B_z\) only.

### 6.2 Filter

Code: `boat_sensing/filter_core.py` (+ `mag_filter` node).

1. Convert Tesla→nT if needed (\(\times10^9\)).  
2. Spike gate: reject outliers beyond \(n_\sigma\cdot\sigma\) relative to recent history (`spike_n_sigma=10`, history 20).  
3. Moving average (`lowpass_window=5`).  
4. After several consecutive rejects, allow a step (avoids permanent lockout on real jumps).

### 6.3 Calibration / anomaly cleaning

Code: `boat_sensing/calibration_core.py` (+ `calibration_node`).

- Discretize lake into cells; bin heading into 8 bins.  
- Learn ambient baseline \(b(i,j,h)\) while anomaly residual small (\(|raw-b|<35\,\mathrm{nT}\) for learning).  
- Spatial anomaly \(|raw - b|\); temporal enhancement yields `cleaned_anomaly_nt`.  
- Phase `READY` when enough cells / coverage (`ready_min_cells=8` or coverage percent), then may freeze baseline.

### 6.4 Detection likelihood for Bayesian updates

Code: `boat_mapping/bayes_core.py`. Soft detection probability as distance soft-dipoles:

\[
P_{\mathrm{det}}(d)=P_{\mathrm{bg}}+(P_{\max}-P_{\mathrm{bg}})
\frac{d_{1/2}^{3}}{d^{3}+d_{1/2}^{3}},
\]

with \(P_{\mathrm{bg}}=0.05\), \(P_{\max}=0.95\), \(d_{1/2}=30\,\mathrm{m}\).

Observation labels from cleaned anomaly \(a_{\mathrm{cl}}\):

| Label | Condition (`mapping.yaml`) |
|-------|----------------------------|
| HIT | \(a_{\mathrm{cl}}\ge 15\,\mathrm{nT}\) |
| MISS | \(a_{\mathrm{cl}}\le 5\,\mathrm{nT}\) |
| ABSTAIN | otherwise |

With `hit_only: true`, MISS is ignored (treated as ABSTAIN). On HIT:

\[
b_k \leftarrow b_k\, P_{\mathrm{det}}(d_k),\qquad
b \leftarrow b/\sum_j b_j.
\]

**Peak:** \(\arg\max_k b_k\).  
**Centroid:** mass-weighted mean of cells with \(b_k \ge 0.5\, b_{\max}\); also publish spread (RMS radius) and mass.

### 6.5 Dipole least-squares fix

Code: `boat_mapping/dipole_fit.py`. Buffer samples with \(a_i\ge10\,\mathrm{nT}\) (min 12). With `dipole_model: total_field` (default YAML), fit the **same physics as the plant**: Levenberg–Marquardt on \(\lvert\Delta F(\mathbf{t},\mathbf{m})\rvert\) vs cleaned \(|a|\). Pose is warmed up with \(\mathbf{m}\) frozen (vertical prior), then \(\mathbf{m}\) is thawed in scaled units so \((t_x,t_y)\) and moment stay well-conditioned. Depth defaults to the known plant \(t_z\) (`dipole_fit_free_depth: false`); set true to free \(t_z\) with bounds. Publish `/swarm/belief/fix` (now with \(z\)) when RMS \(<12\,\mathrm{nT}\). Bayes/MI **do not** use this fix — only scoring / plotter. Fallback `scalar_soft` still fits \((t_x,t_y,A)\) in the \(A/(r^3+s^3)\) family.

### 6.6 Mutual information path selection

Code: `boat_mission/info_gain.py`.

Let \(H(b)=-\sum_k b_k\log b_k\). For candidate pose \(\mathbf{c}\), compute expected posterior entropy after a virtual HIT or MISS under \(P_{\mathrm{det}}\), then

\[
I(\mathbf{c})=H(b)-\mathbb{E}\big[H(b'\mid \mathrm{obs})\big].
\]

Pick \(\arg\max I\). Caps and bounds prevent leaving coverage box.

### 6.7 Strip lawnmower geometry

Code: `boat_mission/path_planning.py` → `strip_coverage_bounds`, `generate_strip_lawnmower`.

Coverage \([x_{\min},x_{\max}]\times[y_{\min},y_{\max}]\) with \(N\) ASVs and gap \(g\):

1. Divide the \(x\)-span into \(N\) nominal slabs.  
2. Eat \(g/2\) from each shared edge (dead corridor).  
3. Inside each strip, classical boustrophedon zigzag with row spacing \(10\,\mathrm{m}\).

This avoids the “seam collision” failure mode of naive Voronoi edges meeting mid-lake.

### 6.8 Expanding spiral (inland-safe)

\[
r_k = k\cdot\Delta r,\quad
n_k=\max\big(8,\lceil 2\pi r_k / \Delta s\rceil\big),\quad
\Delta r=0.5,\ \Delta s=0.5.
\]

**Critical:** instead of clamping points onto the box (which piles waypoints on the shore), compute

\[
r_{\max}^{\mathrm{fit}}=\min_i\mathrm{dist}(\mathrm{center},\partial\mathrm{box})-\mathrm{inset},
\qquad \mathrm{inset}=10\,\mathrm{m},
\]

then use \(r\le\min(80,r_{\max}^{\mathrm{fit}})\). If the peak is too close to the wall for a real spiral, degrade to a tiny loiter.

### 6.9 Verification orbit

Circle of radius \(\le\min(20,r_{\max}^{\mathrm{fit}})\), \(n=12\) points, closed. Entry angle:

\[
\theta_0=\operatorname{atan2}(y_c-y_d,\ x_c-x_d)
\]

so the peer approaches from the side **opposite** the discoverer. Long peer transits insert linear hops every ~40 m.

### 6.10 LOS guidance and thrust

Code: `boat_navigation/guidance.py`, `los_path_follower.py`, `boat_control/mixer.py`.

Cross-track error \(e_\perp\) on the active segment; lookahead \(L=15\,\mathrm{m}\):

\[
\psi_{\mathrm{LOS}}=\psi_{\mathrm{path}}+\operatorname{atan2}(-e_\perp,L).
\]

Yaw PID on heading error; surge open-loop \(u_{\mathrm{ref}}\) scaled by \(\max(0.2,\cos|e_\psi|)\). Advance segment if within `accept_radius_m=8` of end (or along-track past end−ε).

Mixer (left/right thrust):

\[
T_{L,R}=\mathrm{clip}\big((u\mp \omega\cdot g)\cdot S,\ \pm T_{\max}\big),
\]

defaults \(S=50\), \(g=1\), \(T_{\max}=100\).

Empty `plan` → LOS publishes zero Twist (**HOLD**).

---

## 7. ROS nodes (developer digest)

Full parameter tables: `version_3/docs/ros2_node_catalog.md`. Digest:

### Per-ASV

| Node | Package | Core job |
|------|---------|----------|
| `thrust_mixer` | boat_control | `cmd_vel` → differential Gazebo thrusts |
| `gazebo_pose2d` | boat_navigation | GZ dynamic pose → `pose2d` |
| `los_path_follower` | boat_navigation | `plan`+`odom` → `cmd_vel` |
| `mag_driver` | boat_sensing | synthetic / GZ mag → `mag/raw` |
| `mag_filter` | boat_sensing | spike + MA → `mag/filtered` |
| `calibration_node` | boat_sensing | baseline → `mag/anomaly` + status |
| `mission_manager` | boat_mission | FSM + path publisher |

### Swarm / once

| Node | Core job |
|------|----------|
| `bayes_fusion` | multi-ASV Bayesian map + dipole fix |
| `verify_coordinator` | declare→request→halt/complete |
| `trajectory_plotter` | matplotlib operator console |

### Bridges

| Bridge | Config |
|--------|--------|
| `boat_bridge` | `config/bridge.yaml` — thrusters, odom, imu, gps, mag |
| `clock_bridge` | `config/clock_clock.yaml` / `clock_bridge.yaml` when fast |

---

## 8. Complete topic catalog

### 8.1 Per-ASV relative topics (prefix `/asvN/`)

| Topic | Type | Direction | Producer → consumer |
|-------|------|-----------|---------------------|
| `pose2d` | `geometry_msgs/Pose2D` | pub | gazebo_pose2d → mission, mag_driver, plotter |
| `odom` | `nav_msgs/Odometry` | bridged | GZ → LOS |
| `cmd_vel` | `geometry_msgs/Twist` | pub/sub | LOS → mixer; also mag_driver (motor flag) |
| `plan` | `nav_msgs/Path` (TL) | pub | mission_manager → LOS |
| `plan/active` | `nav_msgs/Path` (TL) | pub | LOS echo |
| `nav/debug` | `Float32MultiArray` | pub | LOS internals |
| `imu/data` | `sensor_msgs/Imu` | bridged | GZ |
| `gps/fix` | `sensor_msgs/NavSatFix` | bridged | GZ |
| `mag/gazebo` | `sensor_msgs/MagneticField` | bridged | GZ → mag_driver |
| `mag/raw` | `boat_msgs/MagReading` | pub | mag_driver → filter |
| `mag/filtered` | `boat_msgs/MagReading` | pub | filter → calibration |
| `mag/filter_status` | status | pub | mag_filter |
| `mag/anomaly` | `boat_msgs/MagAnomaly` | pub | calibration → bayes, mission |
| `calibration/status` | `boat_msgs/CalibrationStatus` | pub | calibration → mission |
| `mission/state` | `boat_msgs/MissionState` | pub | mission → plotter |
| `mission/mode` | `std_msgs/String` | pub | mission |
| `mission/info_gain` | `std_msgs/Float64` | pub | mission |

### 8.2 Absolute `/swarm` topics

| Topic | Type | Role |
|-------|------|------|
| `/swarm/belief/map` | `boat_msgs/BeliefGrid` | Full posterior |
| `/swarm/belief/peak` | `geometry_msgs/PoseStamped` | Argmax cell |
| `/swarm/belief/peak_probability` | `std_msgs/Float64` | \(p^\star\) |
| `/swarm/belief/centroid` | `geometry_msgs/PoseStamped` | Weighted estimate |
| `/swarm/belief/centroid_mass` | `std_msgs/Float64` | Mass in blob |
| `/swarm/belief/centroid_spread` | `std_msgs/Float64` | \(\sigma\) (m) |
| `/swarm/belief/fix` | `geometry_msgs/PoseStamped` | Dipole LS fix |
| `/swarm/belief/fix_rms` | `std_msgs/Float64` | Fit residual |
| `/swarm/belief/fix_samples` | `std_msgs/Int32` | Buffer size |
| `/swarm/verify/declare` | `boat_msgs/VerifyRequest` | Discoverer announce |
| `/swarm/verify/request` | `boat_msgs/VerifyRequest` | Coordinator assign |
| `/swarm/verify/result` | `boat_msgs/VerifyResult` (TL) | Verifier outcome |
| `/swarm/mission/complete` | `std_msgs/Bool` (TL) | Success latch |
| `/swarm/mission/halt` | `std_msgs/Bool` (TL) | Swarm stop |
| `/swarm/mission/status` | `std_msgs/String` (TL) | Human string |

### 8.3 Gazebo model topics (ASV3 example)

- `/model/simple_boat_3/joint/left_propeller_joint/cmd_thrust`  
- `/model/simple_boat_3/joint/right_propeller_joint/cmd_thrust`  
- `/model/simple_boat_3/odometry`, `imu`, `gps`, `magnetometer`, …

### 8.4 Timing rates (typical)

| Stream | Rate |
|--------|------|
| mag_driver | 20 Hz |
| LOS control | 10 Hz |
| mission_manager | 2 Hz |
| bayes publish | 2 Hz |
| Gazebo physics | depends (fast mode ~boosted RTF) |

---

## 9. Custom messages

Schemas live in `boat_msgs/msg/`. Summary:

**`MagReading`** — `bx,by,bz,scalar` (nT), pose `(x,y)`, `heading`, `motor_on`.  
**`MagAnomaly`** — `raw_nt`, `baseline_nt`, **`cleaned_anomaly_nt`**, calibration flags, cell indices.  
**`CalibrationStatus`** — `phase` ∈ {`CALIBRATING`,`READY`,`DEGRADED`}, coverage metrics.  
**`BeliefGrid`** — origin, resolution, width/height, row-major probabilities.  
**`MissionState`** — `mode`, `hunt_phase`, peak, info-gain steps, confirmations.  
**`VerifyRequest`** — discoverer/verifier ids, candidate \((x,y,p)\), discoverer pose.  
**`VerifyResult`** — `success`, verifier id, confirmations, `final_peak_p`.

---

## 10. Mission FSM and verify handshake

### 10.1 State diagram

```
GLOBAL_SEARCH
     │  cal READY ∧ t≥20s ∧ streak(p*,dist)
     ▼
TARGET_SEARCH [INFO_GAIN]
     │  budget / MI / near-peak
     ▼
TARGET_SEARCH [SPIRAL]
     │  dwell ∧ confidence
     ▼
HOLD  ──declare──►  verify_coordinator
     │                     │
     │              assign peer (prefer other)
     │                     ▼
     └──────── request ──► VERIFY (orbit + confirm×4)
                              │ success
                              ▼
                           COMPLETE ◄── halt/complete latch
```

Non-verifier ASVs: on request for someone else → **HOLD** + clear plan.

### 10.2 Handshake sequence (topics)

1. Discoverer → `/swarm/verify/declare`  
2. Coordinator → `/swarm/verify/request` (`verifier_id` set)  
3. Verifier → `/swarm/verify/result`  
4. Coordinator → `/swarm/mission/complete` + `/swarm/mission/halt` + status string  

Single-ASV: launch forces `self_verify:=true` so discoverer verifies itself.

### 10.3 Plotter phase strip

Maps swarm/boat modes to SEARCH → HUNT → VERIFY → CONFIRMED.  
**Important:** CONFIRMED means software verify success / latched complete — **not** automatically offline \(\lt5\,\mathrm{m}\) score (see §13).

---

## 11. Path generation and multi-ASV deconfliction

| Stage | Generator | Deconfliction idea |
|-------|-----------|--------------------|
| GLOBAL | Vertical strips + gap corridor | Spatial separation of lawnmowers |
| INFO_GAIN | Local rings | Soft; all may head to same peak |
| SPIRAL | Inland-capped rings | Radius shrink near shore |
| HOLD | Empty path | True stop |
| VERIFY | Orbit + transit hops | Only assigned verifier moves; others HOLD |

Historical failure: large spiral + clamp-to-boundary → boats ramming east shore → Gazebo fling to kilometres away → odom death → peer verifier “stuck”. Current code prevents wall-hugging spirals and stops non-verifiers.

---

## 12. Launches, configs, and operator checklist

### 12.1 Phase launches

| Launch | Intent |
|--------|--------|
| `multi_asv_phase1.launch.py` | Boats + control only |
| `multi_asv_phase2.launch.py` | + mission lawnmower (no mag/belief) |
| `multi_asv_phase3.launch.py` | Full stack, 2 ASVs |
| **`multi_asv_phase4.launch.py`** | **Full stack, 3 ASVs (primary)** |
| `sim.launch.py` | Monolith with args (`num_asvs`, `fast`, feature toggles) |
| `record.launch.py` / `replay.launch.py` | Bags / offline |

### 12.2 Key YAML files (`boat_bringup/config/`)

| File | Owns |
|------|------|
| `sensing.yaml` | Planted target, dipole, filter, calibration grid |
| `mapping.yaml` | Belief grid, HIT thresholds, dipole fit |
| `mission.yaml` | Coverage box, strips, FSM gates, spiral/orbit, verify |
| `navigation.yaml` | LOS gains, speeds |
| `bridge.yaml` | ros_gz topic pairs |

### 12.3 Bring-up recipe (clean)

```bash
cd /home/robot/simulation_ws/version_3
# kill ALL prior gz/ros/thrust_mixer/mission stacks first
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=43   # fresh domain avoids latched COMPLETE ghosts
ros2 launch boat_bringup multi_asv_phase4.launch.py
```

Inspect:

```bash
ros2 topic echo /asv1/mission/state --once
ros2 topic echo /swarm/belief/peak_probability --once
ros2 topic echo /swarm/mission/status --once
ros2 param get /asv1/mag_driver target_x
```

---

## 13. Acceptance, scoring, and common failure modes

### 13.1 What “CONFIRMED” is

Software success: verify confirmations reached + coordinator halt.  
It does **not** assert \(\|\hat{\mathbf{t}}-\mathbf{t}^{\star}\|<5\,\mathrm{m}\). Score that offline from `/swarm/belief/fix` (or centroid) vs planted target in `sensing.yaml`.

### 13.2 Common failures (ROS-centric)

| Symptom | Likely cause | Mitigation |
|---------|--------------|------------|
| Instant CONFIRMED on new run | Latched `/swarm/mission/complete` from old nodes | Kill all stacks; new `ROS_DOMAIN_ID` |
| Boat at \((\pm600,\pm3000)\) | Shore / boat collision fling | Inland spiral inset; HOLD peers |
| ASV1 VERIFY but not moving | Dead `odom` / ghost mixers / wedged Gazebo | Clean relaunch; check `ros2 topic hz /asv1/odom` |
| Belief peak far from true target early | Spurious spawn HITs / incomplete coverage | hit threshold 15 nT; wait for lawnmower |
| Two boats spiral same peak | Peers not HOLDing | Non-verifier HOLD on verify request |
| Mag never crosses 15 nT | Wrong planted params / far pass | Check `target_x/y`, strip assignment |

### 13.3 Debugging sequence

1. `ros2 node list` — expect 3× sensing/mission/LOS, 1× bayes, 1× verify, 1 plotter.  
2. Confirm **exactly one** `thrust_mixer` per ASV (ghosts from old launches are fatal).  
3. `hz` on `pose2d` and `odom` for the stuck boat.  
4. Echo latched `/swarm/mission/complete` with TRANSIENT_LOCAL QoS.  
5. Read `/tmp/sim_run.log` for SPIRAL / VERIFY / planted lines.

---

## 14. File index (code map)

| Concern | Path |
|---------|------|
| Dipole plant | `boat_sensing/boat_sensing/dipole.py`, `mag_driver.py` |
| Filter | `boat_sensing/boat_sensing/filter_core.py` |
| Calibration | `boat_sensing/boat_sensing/calibration_core.py` |
| Bayes | `boat_mapping/boat_mapping/bayes_core.py`, `bayes_fusion.py` |
| Dipole LS | `boat_mapping/boat_mapping/dipole_fit.py` |
| MI | `boat_mission/boat_mission/info_gain.py` |
| Paths / spiral / orbit | `boat_mission/boat_mission/path_planning.py` |
| FSM | `boat_mission/boat_mission/mission_manager.py` |
| Verify math | `boat_mission/boat_mission/verify_core.py` |
| Coordinator | `boat_mission/boat_mission/verify_coordinator.py` |
| LOS | `boat_navigation/boat_navigation/los_path_follower.py`, `guidance.py` |
| Mixer | `boat_control/boat_control/mixer.py` |
| World / boats | `boat_description/worlds/water_world.sdf`, `models/simple_boat*` |
| Launch | `boat_bringup/launch/sim.launch.py`, `multi_asv_phase4.launch.py` |
| Config | `boat_bringup/config/{sensing,mapping,mission,navigation,bridge}.yaml` |
| Msgs | `boat_msgs/msg/*.msg` |
| Node catalog | `version_3/docs/ros2_node_catalog.md` |
| Overleaf PDF book | `docs/overleaf_magnetic_asv/` |

---

## Appendix A — Default numeric cheat-sheet (`version_3`)

| Quantity | Value |
|----------|-------|
| Lake draw / shore approx | ±150 / walls ~±155 m |
| Coverage box | ±120 m |
| Strip gap | 20 m |
| Lawnmower spacing | 10 m |
| Planted target | (85, 40, −1) m |
| Dipole | total-field, peak 50 nT, \(t_z=-1\) m |
| HIT / MISS | 15 / 5 nT |
| Belief cell | 10 m |
| Enter TARGET \(p^\star\) | 0.25 × 4 ticks |
| Spiral \(\Delta r,\Delta s\) / max / inset | 0.5 / 0.5 / 8 m / 10 m inland |
| Verify orbit / confirms | 0.6 m / 4 |
| LOS \(u_{\mathrm{ref}},L\) | 0.35 m/s, 15 m |
| `self_verify` (3 ASV) | false |

## Appendix B — Sequence diagram (verify)

```
ASV_d (discoverer)     verify_coordinator        ASV_v (verifier)      bayes_fusion
       |                        |                       |                    |
   SPIRAL done               |                       |                    |
   HOLD + stop               |                       |                    |
   declare(candidate) ------>|                       |                    |
                             |-- pick ASV_v -------->|                    |
                             |   request(verifier=v) |                    |
                             |                       | HOLD others        |
                             |                       | transit+orbit      |
                             |                       | confirm using      |
                             |                       | anomaly+belief <---|
                             |<---- result(success) -|                    |
                          complete+halt ----------> both ASVs COMPLETE
```

---

*End of manual. For parameter-level exhaustive tables, continue in `version_3/docs/ros2_node_catalog.md`. For a printable PDF book, upload `docs/overleaf_magnetic_asv/` to Overleaf (`main.tex`).*
