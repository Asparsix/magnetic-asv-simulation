"""Unit tests for planted magnetic dipole helpers."""

import math

from boat_sensing.dipole import (
    apply_vertical_dipole,
    dipole_anomaly_nt,
    dipole_B_nt,
    earth_field_enu,
    planted_dipole_field_nt,
    resolve_dipole_moment,
    total_field_anomaly_nt,
    unit_vector,
    vertical_moment_for_peak_nt,
)


def test_dipole_peaks_at_target():
    strength = 4.0e5
    soft = 20.0
    at_target = dipole_anomaly_nt(-50.0, 60.0, -50.0, 60.0, strength, soft_m=soft)
    near = dipole_anomaly_nt(-40.0, 60.0, -50.0, 60.0, strength, soft_m=soft)
    far = dipole_anomaly_nt(110.0, 110.0, -50.0, 60.0, strength, soft_m=soft)
    assert at_target > near > far
    assert abs(at_target - 50.0) < 0.5  # r=0 → A / soft^3 = 4e5 / 8000 = 50


def test_dipole_profile_realistic():
    """soft=20 gives 50 nT overhead, ~25 nT at the 20 m sweep, HIT within ~20 m."""
    strength = 4.0e5
    soft = 20.0
    hit_threshold = 15.0
    at_0 = dipole_anomaly_nt(-50.0, 60.0, -50.0, 60.0, strength, soft_m=soft)
    at_20 = dipole_anomaly_nt(-50.0, 40.0, -50.0, 60.0, strength, soft_m=soft)
    at_40 = dipole_anomaly_nt(-50.0, 20.0, -50.0, 60.0, strength, soft_m=soft)
    assert abs(at_0 - 50.0) < 0.5
    assert at_20 >= hit_threshold  # ~25 nT
    assert at_40 < hit_threshold  # ~6 nT


def test_apply_vertical_dipole():
    bx, by, bz = apply_vertical_dipole(1.0, 2.0, 3.0, 10.0)
    assert (bx, by, bz) == (1.0, 2.0, 13.0)


def test_earth_field_enu_northern_hemisphere_points_down():
    bx, by, bz = earth_field_enu(45000.0, 15.0, -1.0)
    f_hat = unit_vector(bx, by, bz)
    assert abs(math.sqrt(bx * bx + by * by + bz * bz) - 45000.0) < 1e-6
    assert bz < 0.0  # up-component negative → field dips
    assert by > abs(bx)  # mostly north at small declination
    assert abs(f_hat[0] ** 2 + f_hat[1] ** 2 + f_hat[2] ** 2 - 1.0) < 1e-9


def test_vector_dipole_overhead_bz_is_twice_mz_over_r3():
    h = 1.0
    mz = -96.6
    bx, by, bz = dipole_B_nt(0.0, 0.0, h, 0.0, 0.0, mz)
    assert abs(bx) < 1e-9
    assert abs(by) < 1e-9
    assert abs(bz - 2.0 * mz / (h ** 3)) < 1e-6


def test_total_field_peak_and_hit_radius_without_soft():
    """Pure 1/r^3 at 1 m depth: 50 nT overhead, HIT only within ~0.7 m."""
    peak = 50.0
    inc = 15.0
    dec = -1.0
    tz = -1.0
    mx, my, mz = vertical_moment_for_peak_nt(peak, tz, inc, sensor_z=0.0)
    assert abs(mx) < 1e-9 and abs(my) < 1e-9
    assert mz < 0.0

    _bx, _by, _bz, _s, dF0 = planted_dipole_field_nt(
        0.0, 0.0, 0.0, 0.0, 0.0, tz, mx, my, mz, 45000.0, inc, dec
    )
    _bx, _by, _bz, _s, dF05 = planted_dipole_field_nt(
        0.5, 0.0, 0.0, 0.0, 0.0, tz, mx, my, mz, 45000.0, inc, dec
    )
    _bx, _by, _bz, _s, dF1 = planted_dipole_field_nt(
        1.0, 0.0, 0.0, 0.0, 0.0, tz, mx, my, mz, 45000.0, inc, dec
    )
    assert abs(dF0 - peak) < 1.0
    assert dF05 >= 15.0
    assert abs(dF1) < 15.0


def test_mission_soft_calibration():
    """s=11.5 m: 50 nT overhead, HIT at 5 m offset, ~10-15 nT at 10 m (full ΔF)."""
    from boat_sensing.dipole import calibrate_soft_m_for_mission

    peak = 50.0
    inc = 15.0
    dec = -1.0
    tz = -1.0
    soft = 11.5
    mx, my, mz = vertical_moment_for_peak_nt(
        peak, tz, inc, sensor_z=0.0, soft_m=soft
    )
    kwargs = dict(
        sensor_z=0.0,
        target_x=0.0,
        target_y=0.0,
        target_z=tz,
        mx=mx,
        my=my,
        mz=mz,
        earth_total_nt=45000.0,
        inclination_deg=inc,
        declination_deg=dec,
        soft_m=soft,
    )
    _bx, _by, _bz, _s, dF0 = planted_dipole_field_nt(0.0, 0.0, **kwargs)
    _bx, _by, _bz, _s, dF5 = planted_dipole_field_nt(5.0, 0.0, **kwargs)
    _bx, _by, _bz, _s, dF10 = planted_dipole_field_nt(10.0, 0.0, **kwargs)
    assert abs(abs(dF0) - peak) < 1.0
    assert abs(dF5) >= 15.0
    assert 10.0 <= abs(dF10) <= 15.5

    best_s, profile = calibrate_soft_m_for_mission()
    assert 9.0 <= best_s <= 12.5
    assert abs(profile['overhead_nt'] - peak) < 1.0
    assert profile['half_spacing_nt'] >= 15.0
    assert profile['sensing_range_nt'] >= 9.0


def test_total_field_soft_footprint_hits_10m_lawnmower():
    """s=20 m: 50 nT overhead at 1 m depth, HIT still reaches ~20 m."""
    peak = 50.0
    inc = 15.0
    dec = -1.0
    tz = -1.0
    soft = 20.0
    mx, my, mz = vertical_moment_for_peak_nt(
        peak, tz, inc, sensor_z=0.0, soft_m=soft
    )
    kwargs = dict(
        sensor_z=0.0,
        target_x=0.0,
        target_y=0.0,
        target_z=tz,
        mx=mx,
        my=my,
        mz=mz,
        earth_total_nt=45000.0,
        inclination_deg=inc,
        declination_deg=dec,
        soft_m=soft,
    )
    _bx, _by, _bz, _s, dF0 = planted_dipole_field_nt(0.0, 0.0, **kwargs)
    _bx, _by, _bz, _s, dF5 = planted_dipole_field_nt(5.0, 0.0, **kwargs)
    _bx, _by, _bz, _s, dF10 = planted_dipole_field_nt(10.0, 0.0, **kwargs)
    _bx, _by, _bz, _s, dF40 = planted_dipole_field_nt(40.0, 0.0, **kwargs)
    assert abs(dF0 - peak) < 1.0
    assert abs(dF5) >= 15.0
    assert abs(dF10) >= 15.0
    assert abs(dF40) < 15.0


def test_planted_field_has_horizontal_components():
    mx, my, mz = vertical_moment_for_peak_nt(50.0, -1.0, 15.0, soft_m=20.0)
    bx, by, bz, scalar, dF = planted_dipole_field_nt(
        10.0, 5.0, 0.0, 0.0, 0.0, -1.0, mx, my, mz, 45000.0, 15.0, -1.0,
        soft_m=20.0,
    )
    fe = earth_field_enu(45000.0, 15.0, -1.0)
    assert abs(bx - fe[0]) > 1.0  # dipole leaks into east
    assert abs(scalar - math.sqrt(bx * bx + by * by + bz * bz)) < 1e-9
    # Second-order |Bd|^2/(2F) is a few tenths of nT; first-order is still tight.
    assert abs(scalar - (45000.0 + dF)) < 2.0


def test_resolve_dipole_moment_explicit_overrides_peak():
    m = resolve_dipole_moment(1.0, 2.0, 3.0, 50.0, -1.0, 15.0)
    assert m == (1.0, 2.0, 3.0)
    auto = resolve_dipole_moment(0.0, 0.0, 0.0, 50.0, -1.0, 15.0)
    expected = vertical_moment_for_peak_nt(50.0, -1.0, 15.0)
    assert auto == expected


def test_total_field_anomaly_is_projection():
    f_hat = unit_vector(*earth_field_enu(45000.0, 15.0, -1.0))
    bd = (3.0, -4.0, 12.0)
    assert abs(
        total_field_anomaly_nt(*bd, f_hat)
        - (bd[0] * f_hat[0] + bd[1] * f_hat[1] + bd[2] * f_hat[2])
    ) < 1e-12
