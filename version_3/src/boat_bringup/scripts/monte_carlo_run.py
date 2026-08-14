#!/usr/bin/env python3
"""Monte Carlo batch runner for the magnetic ASV mission stack.

Randomly varies planted target pose, depth, noise, moment orientation, and
mission seed jitter, then launches headless 3-ASV sim trials and records
verify success plus localization error.

Usage (after colcon build):
  source install/setup.bash
  ros2 run boat_bringup monte_carlo_run.py --trials 20 --timeout 900

Water current is not modeled. MC trials add slow mag baseline drift via
monte_carlo_mag_drift (MC launch only; normal sim unchanged).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml


def version3_overlay_env(env: dict[str, str], workspace: Path) -> dict[str, str]:
    """Prefer version_3 boat_* packages while keeping parent deps (e.g. ros_gz_bridge)."""
    v3_install = workspace / 'install'
    v3_prefixes = [
        str(v3_install / name)
        for name in (
            'boat_bringup',
            'boat_mission',
            'boat_mapping',
            'boat_sensing',
            'boat_navigation',
            'boat_msgs',
            'boat_description',
            'boat_control',
        )
        if (v3_install / name).is_dir()
    ]
    existing = [p for p in env.get('AMENT_PREFIX_PATH', '').split(':') if p]
    filtered = [
        p for p in existing
        if '/simulation_ws/install/boat_' not in p
        and p != str(v3_install)
    ]
    env = env.copy()
    env['AMENT_PREFIX_PATH'] = ':'.join(v3_prefixes + filtered)
    env['COLCON_PREFIX_PATH'] = env['AMENT_PREFIX_PATH']
    return env

try:
    from boat_mission.path_planning import strip_coverage_bounds
    from boat_sensing.dipole import vertical_moment_for_peak_nt
except ImportError:
    # When invoked before sourcing, allow PYTHONPATH override.
    repo_root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(repo_root / 'src' / 'boat_mission'))
    sys.path.insert(0, str(repo_root / 'src' / 'boat_sensing'))
    from boat_mission.path_planning import strip_coverage_bounds
    from boat_sensing.dipole import vertical_moment_for_peak_nt


DEFAULT_SEEDS_3 = (
    (-120.0, -120.0),
    (-33.333, -120.0),
    (53.333, -120.0),
)


def _uniform(rng: random.Random, spec: dict) -> float:
    return rng.uniform(float(spec['min']), float(spec['max']))


def _lawnmower_strips(cfg: dict) -> list[tuple[float, float, float, float]]:
    """ASV strip rectangles (x0, x1, y0, y1); excludes inter-boat gap corridors."""
    min_x = float(cfg.get('coverage_min_x', -120.0))
    max_x = float(cfg.get('coverage_max_x', 120.0))
    min_y = float(cfg.get('coverage_min_y', -120.0))
    max_y = float(cfg.get('coverage_max_y', 120.0))
    num_asvs = int(cfg.get('num_asvs', 3))
    gap = float(cfg.get('strip_gap_m', 20.0))
    inset = max(0.0, float(cfg.get('strip_inset_m', 2.0)))
    strips = []
    for index in range(num_asvs):
        x0, x1, y0, y1 = strip_coverage_bounds(
            min_x, max_x, min_y, max_y, index, num_asvs, gap_m=gap
        )
        x0 += inset
        x1 -= inset
        if x1 > x0:
            strips.append((x0, x1, y0, y1))
    return strips


def _sample_target_xy_in_strips(rng: random.Random, cfg: dict) -> tuple[float, float]:
    """Uniform over the union of lawnmower strips (never in 20 m dead gaps)."""
    strips = _lawnmower_strips(cfg)
    if not strips:
        raise RuntimeError('no valid lawnmower strips for Monte Carlo sampling')

    y_cfg_min = float(cfg['target_y']['min'])
    y_cfg_max = float(cfg['target_y']['max'])
    eligible = []
    for x0, x1, y0, y1 in strips:
        y_min = max(y0, y_cfg_min)
        y_max = min(y1, y_cfg_max)
        if x1 <= x0 or y_max <= y_min:
            continue
        eligible.append((x0, x1, y_min, y_max))
    if not eligible:
        raise RuntimeError('target_y range does not overlap any lawnmower strip')

    areas = [(x1 - x0) * (y1 - y0) for x0, x1, y0, y1 in eligible]
    pick = rng.uniform(0.0, sum(areas))
    for (x0, x1, y0, y1), area in zip(eligible, areas):
        if pick <= area:
            return rng.uniform(x0, x1), rng.uniform(y0, y1)
        pick -= area
    x0, x1, y0, y1 = eligible[-1]
    return rng.uniform(x0, x1), rng.uniform(y0, y1)


def _random_moment(
    rng: random.Random,
    peak_nt: float,
    target_z: float,
    inclination_deg: float,
    soft_m: float,
    tilt_max_deg: float,
    magnitude_jitter: float,
) -> tuple[float, float, float]:
    """Tilted moment with magnitude scaled from vertical peak prior."""
    _mx, _my, mz0 = vertical_moment_for_peak_nt(
        peak_nt,
        target_z,
        inclination_deg,
        soft_m=soft_m,
        use_full_delta=True,
    )
    m_mag = abs(mz0) * rng.uniform(1.0 - magnitude_jitter, 1.0 + magnitude_jitter)
    tilt = math.radians(rng.uniform(0.0, tilt_max_deg))
    az = math.radians(rng.uniform(0.0, 360.0))
    mx = m_mag * math.sin(tilt) * math.sin(az)
    my = m_mag * math.sin(tilt) * math.cos(az)
    mz = -m_mag * math.cos(tilt)
    return mx, my, mz


def sample_trial(rng: random.Random, cfg: dict, trial_id: int) -> dict:
    """Draw one Monte Carlo trial configuration."""
    target_x, target_y = _sample_target_xy_in_strips(rng, cfg)
    target_z = _uniform(rng, cfg['target_z'])
    noise_nt = _uniform(rng, cfg['synthetic_noise_nt'])
    peak_nt = _uniform(rng, cfg['dipole_peak_nt'])
    soft_m = float(cfg.get('dipole_soft_m', 11.5))
    inc = float(cfg.get('earth_inclination_deg', 15.0))
    mx, my, mz = _random_moment(
        rng,
        peak_nt,
        target_z,
        inc,
        soft_m,
        float(cfg.get('moment_tilt_max_deg', 45.0)),
        float(cfg.get('moment_magnitude_jitter', 0.2)),
    )
    jitter = float(cfg.get('spawn_y_jitter_m', 15.0))
    seeds = []
    for sx, sy in DEFAULT_SEEDS_3:
        seeds.extend([sx, sy + rng.uniform(-jitter, jitter)])
    drift_rate = _uniform(rng, cfg['mag_baseline_drift_nt_per_min'])
    drift_az = rng.uniform(0.0, 360.0)
    drift_vert = float(cfg.get('mag_baseline_drift_vertical_fraction', 0.15))
    seed = rng.randint(1, 2_000_000_000)

    return {
        'trial_id': trial_id,
        'random_seed': seed,
        'ground_truth': {
            'target_x': target_x,
            'target_y': target_y,
            'target_z': target_z,
            'dipole_peak_nt': peak_nt,
            'dipole_mx': mx,
            'dipole_my': my,
            'dipole_mz': mz,
            'synthetic_noise_nt': noise_nt,
            'mag_baseline_drift_nt_per_min': drift_rate,
            'mag_baseline_drift_azimuth_deg': drift_az,
            'region_seeds': seeds,
        },
        'launch_overrides': {
            'mag_driver': {
                'target_x': target_x,
                'target_y': target_y,
                'target_z': target_z,
                'dipole_peak_nt': peak_nt,
                'dipole_mx': mx,
                'dipole_my': my,
                'dipole_mz': mz,
                'dipole_soft_m': soft_m,
                'synthetic_noise_nt': noise_nt,
                'random_seed': seed,
                'raw_topic': 'mag/plant',
            },
            'mag_baseline_drift': {
                'drift_nt_per_min': drift_rate,
                'drift_azimuth_deg': drift_az,
                'drift_vertical_fraction': drift_vert,
            },
            'bayes_fusion': {
                'dipole_fit_target_z': target_z,
            },
            'mission_manager': {
                'region_seeds': seeds,
            },
            'plotter': {
                'target_x': target_x,
                'target_y': target_y,
            },
        },
    }


def write_trial_yaml(trial: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(trial['launch_overrides'], handle, sort_keys=False)


def _kill_process_group(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.time() + 15.0
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.5)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def _localization_error(
    xy: tuple[float, float] | None,
    tx: float,
    ty: float,
) -> float | None:
    if xy is None:
        return None
    return math.hypot(xy[0] - tx, xy[1] - ty)


def monitor_trial(
    *,
    domain_id: int,
    timeout_s: float,
    ground_truth: dict,
    log_interval_s: float = 1.0,
    trial_label: str = '',
) -> dict:
    """Subscribe for mission completion and belief estimates while sim runs."""
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float64, String

    try:
        from boat_msgs.msg import MissionState, VerifyResult
    except ImportError:
        MissionState = None
        VerifyResult = None

    rclpy.init()
    node = Node('monte_carlo_monitor')
    latched_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    tx = float(ground_truth['target_x'])
    ty = float(ground_truth['target_y'])
    t0 = time.time()
    last_log = t0

    state = {
        'complete': False,
        'status': '',
        'fix_xy': None,
        'fix_rms_nt': None,
        'fix_samples': None,
        'centroid_xy': None,
        'peak_xy': None,
        'peak_p': None,
        'verify_candidate_xy': None,
        'verify_success': None,
        'verify_confirmations': 0,
        'asv_modes': {},
    }

    def _fmt_err(xy) -> str:
        err = _localization_error(xy, tx, ty)
        return 'n/a' if err is None else f'{err:.0f}m'

    def _maybe_log(force: bool = False) -> None:
        nonlocal last_log
        if log_interval_s <= 0.0 and not force:
            return
        now = time.time()
        if not force and now - last_log < log_interval_s:
            return
        last_log = now
        elapsed = int(now - t0)
        modes = state['asv_modes']
        m1 = modes.get('asv1', '?')
        m2 = modes.get('asv2', '?')
        m3 = modes.get('asv3', '?')
        peak_p = state['peak_p']
        peak_s = f'{peak_p:.2f}' if peak_p is not None else 'n/a'
        complete_s = 'DONE' if state['complete'] else 'run'
        prefix = f'  [{trial_label} {elapsed:4d}s]' if trial_label else f'  [{elapsed:4d}s]'
        rms = state['fix_rms_nt']
        rms_s = f' rms={rms:.1f}nT' if rms is not None else ''
        print(
            f'{prefix} {complete_s} '
            f'asv1={m1} asv2={m2} asv3={m3} '
            f'peak_p={peak_s} '
            f'cen={_fmt_err(state["centroid_xy"])} '
            f'fix={_fmt_err(state["fix_xy"])}'
            f'{rms_s}',
            flush=True,
        )

    def _track_best(key: str, xy: tuple[float, float]) -> None:
        err = _localization_error(xy, tx, ty)
        prev = state[key]
        if prev is None:
            state[key] = xy
            return
        if err is None:
            return
        prev_err = _localization_error(prev, tx, ty)
        if prev_err is None or err < prev_err:
            state[key] = xy

    def on_complete(msg: Bool) -> None:
        state['complete'] = bool(msg.data)
        if msg.data:
            _maybe_log(force=True)

    def on_status(msg: String) -> None:
        state['status'] = msg.data

    def on_fix(msg: PoseStamped) -> None:
        _track_best('fix_xy', (float(msg.pose.position.x), float(msg.pose.position.y)))

    def on_centroid(msg: PoseStamped) -> None:
        _track_best(
            'centroid_xy',
            (float(msg.pose.position.x), float(msg.pose.position.y)),
        )

    def on_peak(msg: PoseStamped) -> None:
        _track_best('peak_xy', (float(msg.pose.position.x), float(msg.pose.position.y)))

    def on_peak_p(msg: Float64) -> None:
        state['peak_p'] = float(msg.data)

    def on_fix_rms(msg: Float64) -> None:
        state['fix_rms_nt'] = float(msg.data)

    if MissionState is not None:
        def _make_mode_cb(asv_id: str):
            def _cb(msg) -> None:
                hunt = msg.hunt_phase.strip()
                label = msg.mode if not hunt else f'{msg.mode}/{hunt}'
                state['asv_modes'][asv_id] = label

            return _cb

        for asv_id in ('asv1', 'asv2', 'asv3'):
            node.create_subscription(
                MissionState,
                f'/{asv_id}/mission/state',
                _make_mode_cb(asv_id),
                10,
            )

    node.create_subscription(Bool, '/swarm/mission/complete', on_complete, latched_qos)
    node.create_subscription(String, '/swarm/mission/status', on_status, latched_qos)
    node.create_subscription(PoseStamped, '/swarm/belief/fix', on_fix, 10)
    node.create_subscription(PoseStamped, '/swarm/belief/centroid', on_centroid, 10)
    node.create_subscription(PoseStamped, '/swarm/belief/peak', on_peak, 10)
    node.create_subscription(Float64, '/swarm/belief/peak_probability', on_peak_p, 10)
    node.create_subscription(Float64, '/swarm/belief/fix_rms', on_fix_rms, 10)

    if VerifyResult is not None:
        def on_verify(msg) -> None:
            state['verify_success'] = bool(msg.success)
            state['verify_confirmations'] = int(msg.confirmations)
            state['verify_candidate_xy'] = (
                float(msg.candidate_x),
                float(msg.candidate_y),
            )
            _maybe_log(force=True)

        node.create_subscription(
            VerifyResult, '/swarm/verify/result', on_verify, latched_qos
        )

    end = time.time() + timeout_s
    complete_at = None
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        _maybe_log()
        if state['complete'] and complete_at is None:
            complete_at = time.time()
        if complete_at is not None and time.time() - complete_at > 15.0:
            break

    _maybe_log(force=True)

    node.destroy_node()
    rclpy.shutdown()

    fix_err = _localization_error(state['fix_xy'], tx, ty)
    centroid_err = _localization_error(state['centroid_xy'], tx, ty)
    peak_err = _localization_error(state['peak_xy'], tx, ty)
    verify_err = _localization_error(state['verify_candidate_xy'], tx, ty)

    loc_err = fix_err
    loc_source = 'fix'
    if loc_err is None and centroid_err is not None:
        loc_err = centroid_err
        loc_source = 'centroid'
    elif loc_err is None and verify_err is not None:
        loc_err = verify_err
        loc_source = 'verify_candidate'
    elif loc_err is None and peak_err is not None:
        loc_err = peak_err
        loc_source = 'peak'

    def _xy_out(key):
        xy = state[key]
        if xy is None:
            return None, None
        return xy[0], xy[1]

    fix_x, fix_y = _xy_out('fix_xy')
    cx, cy = _xy_out('centroid_xy')
    px, py = _xy_out('peak_xy')
    vx, vy = _xy_out('verify_candidate_xy')

    return {
        'mission_complete': state['complete'],
        'mission_status': state['status'],
        'verify_success': state['verify_success'],
        'verify_confirmations': state['verify_confirmations'],
        'fix_x': fix_x,
        'fix_y': fix_y,
        'fix_error_m': fix_err,
        'fix_rms_nt': state['fix_rms_nt'],
        'centroid_x': cx,
        'centroid_y': cy,
        'centroid_error_m': centroid_err,
        'peak_x': px,
        'peak_y': py,
        'peak_error_m': peak_err,
        'verify_candidate_x': vx,
        'verify_candidate_y': vy,
        'verify_candidate_error_m': verify_err,
        'localization_error_m': loc_err,
        'localization_source': loc_source if loc_err is not None else None,
        'timed_out': not state['complete'],
    }


def run_trial(
    *,
    trial: dict,
    workspace: Path,
    timeout_s: float,
    domain_id: int,
    trial_yaml_dir: Path,
    headless: bool,
    log_interval_s: float = 1.0,
    show_sim_log: bool = False,
) -> dict:
    trial_id = trial['trial_id']
    yaml_path = trial_yaml_dir / f'trial_{trial_id:04d}.yaml'
    write_trial_yaml(trial, yaml_path)
    trial_label = f't{trial_id + 1}'

    env = version3_overlay_env(os.environ.copy(), workspace)
    env['ROS_DOMAIN_ID'] = str(domain_id)
    launch_file = (
        workspace / 'install' / 'boat_bringup' / 'share' / 'boat_bringup' / 'launch' / 'monte_carlo.launch.py'
    )
    if not launch_file.is_file():
        raise FileNotFoundError(f'MC launch file missing: {launch_file}')
    cmd = [
        'ros2', 'launch', str(launch_file),
        f'trial_params_file:={yaml_path}',
        f'headless:={"true" if headless else "false"}',
        'fast:=true',
        'plot_trajectory:=false',
    ]
    start = time.time()
    stdout = None if show_sim_log else subprocess.DEVNULL
    stderr = None if show_sim_log else subprocess.DEVNULL
    proc = subprocess.Popen(
        cmd,
        cwd=str(workspace),
        env=env,
        stdout=stdout,
        stderr=stderr,
        start_new_session=True,
    )
    print(f'  [{trial_label}] starting sim (domain {domain_id})...', flush=True)
    time.sleep(25.0)
    metrics = monitor_trial(
        domain_id=domain_id,
        timeout_s=max(10.0, timeout_s - 25.0),
        ground_truth=trial['ground_truth'],
        log_interval_s=log_interval_s,
        trial_label=trial_label,
    )
    _kill_process_group(proc)
    metrics['wall_time_s'] = time.time() - start
    metrics['trial_id'] = trial_id
    metrics['domain_id'] = domain_id
    return metrics


def load_config(path: Path) -> dict:
    with path.open(encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def summarize(results: list[dict]) -> dict:
    n = len(results)
    if n == 0:
        return {}

    def _errs(key: str) -> list[float]:
        return [r[key] for r in results if r.get(key) is not None]

    complete = sum(1 for r in results if r.get('mission_complete'))
    verify_ok = sum(1 for r in results if r.get('verify_success'))
    fix_errs = _errs('fix_error_m')
    centroid_errs = _errs('centroid_error_m')
    loc_errs = _errs('localization_error_m')

    def _mean(vals: list[float]) -> float | None:
        return None if not vals else sum(vals) / len(vals)

    return {
        'trials': n,
        'mission_complete_rate': complete / n,
        'verify_success_rate': verify_ok / n,
        'fix_error_mean_m': _mean(fix_errs),
        'fix_error_max_m': None if not fix_errs else max(fix_errs),
        'centroid_error_mean_m': _mean(centroid_errs),
        'centroid_error_max_m': None if not centroid_errs else max(centroid_errs),
        'localization_error_mean_m': _mean(loc_errs),
        'localization_error_max_m': None if not loc_errs else max(loc_errs),
        'timeouts': sum(1 for r in results if r.get('timed_out')),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=10)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--timeout', type=float, default=900.0,
                        help='Wall-clock seconds per trial (3-ASV full mission).')
    parser.add_argument('--output', type=Path,
                        default=Path('monte_carlo_results.json'))
    parser.add_argument('--config', type=Path, default=None)
    parser.add_argument('--workspace', type=Path, default=None)
    parser.add_argument('--domain-id-base', type=int, default=50)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument(
        '--log-interval',
        type=float,
        default=1.0,
        help='Seconds between progress lines (0 = off). Default: 1.0',
    )
    parser.add_argument(
        '--show-sim-log',
        action='store_true',
        help='Stream Gazebo/ROS launch stdout (very verbose).',
    )
    args = parser.parse_args()

    workspace = args.workspace or Path(__file__).resolve().parents[3]
    share_config = workspace / 'install' / 'boat_bringup' / 'share' / 'boat_bringup' / 'config' / 'monte_carlo_config.yaml'
    config_path = args.config or share_config
    if not config_path.is_file():
        config_path = Path(__file__).resolve().parents[1] / 'config' / 'monte_carlo_config.yaml'
    cfg = load_config(config_path)

    rng = random.Random(args.seed)
    trial_yaml_dir = args.output.parent / f'{args.output.stem}_trials'
    results = []
    trials = []

    print(f'Monte Carlo: {args.trials} trials, timeout={args.timeout:.0f}s, seed={args.seed}')
    print(f'Config: {config_path}')
    print('Note: MC adds slow mag baseline drift (monte_carlo_mag_drift node).')

    for trial_id in range(args.trials):
        trial = sample_trial(rng, cfg, trial_id)
        trials.append(trial)
        gt = trial['ground_truth']
        print(
            f'\n[{trial_id + 1}/{args.trials}] target=({gt["target_x"]:.1f}, '
            f'{gt["target_y"]:.1f}, {gt["target_z"]:.2f}) noise={gt["synthetic_noise_nt"]:.1f} nT'
        )
        metrics = run_trial(
            trial=trial,
            workspace=workspace,
            timeout_s=args.timeout,
            domain_id=args.domain_id_base + trial_id,
            trial_yaml_dir=trial_yaml_dir,
            headless=not args.gui,
            log_interval_s=args.log_interval,
            show_sim_log=args.show_sim_log,
        )
        merged = {**trial, **metrics}
        results.append(merged)
        status = 'COMPLETE' if metrics.get('mission_complete') else 'TIMEOUT/FAIL'
        loc = metrics.get('localization_error_m')
        loc_src = metrics.get('localization_source') or '?'
        fix = metrics.get('fix_error_m')
        cen = metrics.get('centroid_error_m')
        loc_s = f'{loc:.1f} m ({loc_src})' if loc is not None else 'n/a'
        fix_s = f'{fix:.1f}' if fix is not None else 'n/a'
        cen_s = f'{cen:.1f}' if cen is not None else 'n/a'
        print(
            f'  -> {status} loc_err={loc_s} fix={fix_s} m centroid={cen_s} m '
            f'wall={metrics["wall_time_s"]:.0f}s'
        )

    summary = summarize(results)
    payload = {
        'summary': summary,
        'config_file': str(config_path),
        'master_seed': args.seed,
        'results': results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2)
    print('\nSummary:', json.dumps(summary, indent=2))
    print(f'Wrote {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
