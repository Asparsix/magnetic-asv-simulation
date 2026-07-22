"""Unit tests for planted magnetic dipole helpers."""

from boat_sensing.dipole import apply_vertical_dipole, dipole_anomaly_nt


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
