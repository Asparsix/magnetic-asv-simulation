"""Planted magnetic dipole helpers for simulation.

Physics + mission calibration:

* Vector dipole B ~ [3(m·rhat)rhat - m] / (r^3 + s^3)
* Earth field B0(F, I, D)
* Total-field anomaly dF = |B0 + Banom| - |B0|  (full, not Bz-only)
* s = engineering spread (finite target / effective sensing footprint)

Moment m is in nT·m^3 (mu0/4pi absorbed). ENU: x east, y north, z up.
"""

from __future__ import annotations

import math


def dipole_anomaly_nt(x, y, target_x, target_y, strength_nt, soft_m=1.0):
    """
    Scalar anomaly A / (r^3 + soft^3), matching the offline niot magnetic sim.

    strength_nt is the near-field amplitude constant in the same units as the
    rest of the inflated Gazebo mag pipeline (nT after Tesla conversion).
    """
    dx = float(x) - float(target_x)
    dy = float(y) - float(target_y)
    r = math.hypot(dx, dy)
    soft = max(float(soft_m), 1e-3)
    return float(strength_nt) / (r * r * r + soft * soft * soft)


def apply_vertical_dipole(bx, by, bz, anomaly_nt):
    """Add a vertical (Bz) dipole contribution and return updated axes."""
    return float(bx), float(by), float(bz) + float(anomaly_nt)


def unit_vector(vx, vy, vz):
    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if norm < 1.0e-18:
        return (0.0, 0.0, 0.0)
    inv = 1.0 / norm
    return (vx * inv, vy * inv, vz * inv)


def earth_field_enu(total_nt, inclination_deg, declination_deg):
    """Uniform Earth field in ENU (nT)."""
    f = float(total_nt)
    inc = math.radians(float(inclination_deg))
    dec = math.radians(float(declination_deg))
    cos_i = math.cos(inc)
    sin_i = math.sin(inc)
    cos_d = math.cos(dec)
    sin_d = math.sin(dec)
    return (
        f * cos_i * sin_d,
        f * cos_i * cos_d,
        -f * sin_i,
    )


def dipole_B_nt(rx, ry, rz, mx, my, mz, r_min=0.05, soft_m=0.0):
    """Vector dipole anomaly field (nT): B = [3(m·rhat)rhat - m] / (r^3 + s^3)."""
    r2 = rx * rx + ry * ry + rz * rz
    r = math.sqrt(r2)
    min_r = max(float(r_min), 1.0e-3)
    if r < min_r:
        r = min_r
        r2 = r * r
    inv_r = 1.0 / r
    hx = rx * inv_r
    hy = ry * inv_r
    hz = rz * inv_r
    m_dot_h = mx * hx + my * hy + mz * hz
    soft = max(float(soft_m), 0.0)
    denom = r2 * r + soft * soft * soft
    if denom < 1.0e-12:
        denom = 1.0e-12
    inv_denom = 1.0 / denom
    return (
        (3.0 * m_dot_h * hx - mx) * inv_denom,
        (3.0 * m_dot_h * hy - my) * inv_denom,
        (3.0 * m_dot_h * hz - mz) * inv_denom,
    )


def total_field_anomaly_nt(bx, by, bz, f_hat):
    """First-order total-field anomaly dF ≈ F_hat · Banom."""
    return bx * f_hat[0] + by * f_hat[1] + bz * f_hat[2]


def full_total_field_delta_nt(
    bx_anom,
    by_anom,
    bz_anom,
    earth_total_nt,
    inclination_deg,
    declination_deg,
):
    """Full total-field anomaly: |B0 + Banom| - |B0|."""
    fe = earth_field_enu(earth_total_nt, inclination_deg, declination_deg)
    f0 = float(earth_total_nt)
    bx = fe[0] + float(bx_anom)
    by = fe[1] + float(by_anom)
    bz = fe[2] + float(bz_anom)
    f1 = math.sqrt(bx * bx + by * by + bz * bz)
    return f1 - f0


def vertical_moment_for_peak_nt(
    peak_nt,
    target_z,
    inclination_deg,
    sensor_z=0.0,
    soft_m=0.0,
    earth_total_nt=45000.0,
    declination_deg=-1.0,
    use_full_delta=True,
):
    """Vertical m=(0,0,mz) giving overhead |dF| ≈ peak_nt."""
    h = float(sensor_z) - float(target_z)
    if abs(h) < 0.05:
        h = 0.05 if h >= 0.0 else -0.05
    soft = max(float(soft_m), 0.0)
    peak = abs(float(peak_nt))
    if peak <= 0.0:
        return (0.0, 0.0, 0.0)

    f_hat_z = -math.sin(math.radians(float(inclination_deg)))
    if abs(f_hat_z) < 1.0e-6:
        f_hat_z = -1.0e-6
    denom = abs(h) ** 3 + soft ** 3
    mz_hint = peak * denom / (2.0 * f_hat_z)

    if not use_full_delta:
        return (0.0, 0.0, mz_hint)

    sign = -1.0 if mz_hint < 0.0 else 1.0

    def overhead_abs(mz):
        bd = dipole_B_nt(0.0, 0.0, h, 0.0, 0.0, mz, soft_m=soft)
        return abs(
            full_total_field_delta_nt(
                bd[0],
                bd[1],
                bd[2],
                earth_total_nt,
                inclination_deg,
                declination_deg,
            )
        )

    lo_mag = 0.0
    hi_mag = max(abs(mz_hint) * 4.0, abs(mz_hint) * 0.25, 1.0)
    if overhead_abs(sign * hi_mag) < peak:
        while overhead_abs(sign * hi_mag) < peak and hi_mag < 1.0e8:
            hi_mag *= 2.0

    for _ in range(80):
        mid_mag = 0.5 * (lo_mag + hi_mag)
        mz = sign * mid_mag
        if overhead_abs(mz) < peak:
            lo_mag = mid_mag
        else:
            hi_mag = mid_mag
    mz = sign * 0.5 * (lo_mag + hi_mag)
    return (0.0, 0.0, mz)


def calibrate_soft_m_for_mission(
    peak_nt=50.0,
    target_z=-1.0,
    sensor_z=0.0,
    sensing_range_m=10.0,
    range_anomaly_min_nt=10.0,
    range_anomaly_max_nt=15.0,
    lawnmower_half_spacing_m=5.0,
    hit_threshold_nt=15.0,
    earth_total_nt=45000.0,
    inclination_deg=15.0,
    declination_deg=-1.0,
    s_min=8.0,
    s_max=15.0,
    s_step=0.5,
):
    """
    Tune spread s for NIOT-style missions.

    Targets (with moment re-sized for peak_nt at each s):
    * overhead |dF| = peak_nt
    * at sensing_range_m: |dF| in [range_anomaly_min_nt, range_anomaly_max_nt]
    * at lawnmower_half_spacing_m: |dF| >= hit_threshold_nt
    """
    peak = abs(float(peak_nt))
    range_m = float(sensing_range_m)
    rmin = float(range_anomaly_min_nt)
    rmax = float(range_anomaly_max_nt)
    half = float(lawnmower_half_spacing_m)
    hit = float(hit_threshold_nt)

    best_s = float(s_min)
    best_score = float('inf')
    best_profile = {}

    s = float(s_min)
    while s <= float(s_max) + 1.0e-9:
        mx, my, mz = vertical_moment_for_peak_nt(
            peak,
            target_z,
            inclination_deg,
            sensor_z=sensor_z,
            soft_m=s,
            earth_total_nt=earth_total_nt,
            declination_deg=declination_deg,
            use_full_delta=True,
        )

        def sample(rh):
            bd = dipole_B_nt(
                rh,
                0.0,
                float(sensor_z) - float(target_z),
                mx,
                my,
                mz,
                soft_m=s,
            )
            return abs(
                full_total_field_delta_nt(
                    bd[0],
                    bd[1],
                    bd[2],
                    earth_total_nt,
                    inclination_deg,
                    declination_deg,
                )
            )

        d0 = sample(0.0)
        d_half = sample(half)
        d_range = sample(range_m)
        score = (d0 - peak) ** 2
        if d_range < rmin:
            score += (rmin - d_range) ** 2 * 6.0
        elif d_range > rmax:
            score += (d_range - rmax) ** 2 * 6.0
        if d_half < hit:
            score += (hit - d_half) ** 2 * 4.0

        if score < best_score:
            best_score = score
            best_s = s
            best_profile = {
                'overhead_nt': d0,
                'half_spacing_nt': d_half,
                'sensing_range_nt': d_range,
            }
        s += float(s_step)

    return best_s, best_profile


def planted_dipole_field_nt(
    x,
    y,
    sensor_z,
    target_x,
    target_y,
    target_z,
    mx,
    my,
    mz,
    earth_total_nt,
    inclination_deg,
    declination_deg,
    r_min=0.05,
    soft_m=0.0,
    use_full_delta=True,
):
    """Earth + vector dipole. Returns (bx, by, bz, scalar, dF)."""
    fe = earth_field_enu(earth_total_nt, inclination_deg, declination_deg)
    bd = dipole_B_nt(
        float(x) - float(target_x),
        float(y) - float(target_y),
        float(sensor_z) - float(target_z),
        float(mx),
        float(my),
        float(mz),
        r_min=r_min,
        soft_m=soft_m,
    )
    bx = fe[0] + bd[0]
    by = fe[1] + bd[1]
    bz = fe[2] + bd[2]
    scalar = math.sqrt(bx * bx + by * by + bz * bz)
    if use_full_delta:
        dF = full_total_field_delta_nt(
            bd[0], bd[1], bd[2], earth_total_nt, inclination_deg, declination_deg
        )
    else:
        f_hat = unit_vector(*fe)
        dF = total_field_anomaly_nt(bd[0], bd[1], bd[2], f_hat)
    return bx, by, bz, scalar, dF


def resolve_dipole_moment(
    mx,
    my,
    mz,
    peak_nt,
    target_z,
    inclination_deg,
    sensor_z=0.0,
    soft_m=0.0,
    earth_total_nt=45000.0,
    declination_deg=-1.0,
):
    """Use explicit m if any component is set; otherwise size a vertical dipole."""
    if abs(float(mx)) > 0.0 or abs(float(my)) > 0.0 or abs(float(mz)) > 0.0:
        return (float(mx), float(my), float(mz))
    return vertical_moment_for_peak_nt(
        peak_nt,
        target_z,
        inclination_deg,
        sensor_z=sensor_z,
        soft_m=soft_m,
        earth_total_nt=earth_total_nt,
        declination_deg=declination_deg,
        use_full_delta=True,
    )
