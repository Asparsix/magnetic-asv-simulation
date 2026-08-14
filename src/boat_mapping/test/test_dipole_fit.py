"""Unit tests for least-squares dipole localization."""

import math
import random

from boat_mapping.dipole_fit import DipoleFitter, model_anomaly_nt


def _ring_samples(tx, ty, strength, soft, radii, noise_nt=0.0, seed=0):
    rng = random.Random(seed)
    samples = []
    for radius in radii:
        n = max(8, int(2.0 * math.pi * radius / 10.0))
        for i in range(n):
            angle = 2.0 * math.pi * i / n
            x = tx + radius * math.cos(angle)
            y = ty + radius * math.sin(angle)
            a = model_anomaly_nt(x, y, tx, ty, strength, soft)
            a += rng.gauss(0.0, noise_nt)
            samples.append((x, y, max(a, 0.0)))
    return samples


def test_model_matches_known_overhead_peak():
    soft = 20.0
    strength = soft ** 3 * 50.0  # 50 nT at r=0
    assert abs(model_anomaly_nt(0, 0, 0, 0, strength, soft) - 50.0) < 1e-9


def test_fit_recovers_target_from_noisy_ring():
    soft = 20.0
    strength = soft ** 3 * 50.0
    true_xy = (100.0, -110.0)
    samples = _ring_samples(
        true_xy[0],
        true_xy[1],
        strength,
        soft,
        radii=(10.0, 20.0, 30.0),
        noise_nt=3.0,
        seed=7,
    )
    fitter = DipoleFitter(
        soft_m=soft, min_anomaly_nt=5.0, min_samples=12
    )
    for x, y, a in samples:
        fitter.add_sample(x, y, a)

    # Deliberately bad warm start (grid-cell-like offset).
    fix = fitter.fit(guess_xy=(85.0, -95.0))
    assert fix is not None
    assert fix.success
    err = math.hypot(fix.x - true_xy[0], fix.y - true_xy[1])
    assert err < 3.0, f'localization error {err:.2f} m too large'
    assert fix.residual_rms_nt < 8.0
    assert fix.num_samples >= 12


def test_fit_better_than_anomaly_weighted_centroid():
    soft = 20.0
    strength = soft ** 3 * 50.0
    true_xy = (-40.0, 25.0)
    # Two parallel lawnmower legs — enough geometry to resolve both axes.
    samples = []
    for y in (5.0, 15.0):
        for x in range(-80, 1, 5):
            a = model_anomaly_nt(
                float(x), y, true_xy[0], true_xy[1], strength, soft
            )
            samples.append((float(x), y, a))

    fitter = DipoleFitter(soft_m=soft, min_anomaly_nt=5.0, min_samples=8)
    for x, y, a in samples:
        fitter.add_sample(x, y, a)

    # Magnitude-weighted centroid of samples (grid-style proxy).
    wsum = sum(a for _, _, a in samples)
    cx = sum(x * a for x, _, a in samples) / wsum
    cy = sum(y * a for _, y, a in samples) / wsum
    centroid_err = math.hypot(cx - true_xy[0], cy - true_xy[1])

    fix = fitter.fit(guess_xy=(cx, cy))
    assert fix is not None and fix.success
    fit_err = math.hypot(fix.x - true_xy[0], fix.y - true_xy[1])
    assert fit_err < centroid_err
    assert fit_err < 5.0
    assert centroid_err > 5.0  # centroid is biased toward the tracks


def test_fit_returns_none_until_enough_samples():
    fitter = DipoleFitter(min_samples=10, min_anomaly_nt=5.0)
    for i in range(5):
        fitter.add_sample(float(i), 0.0, 20.0)
    assert fitter.fit() is None


def test_weak_samples_are_ignored():
    fitter = DipoleFitter(min_anomaly_nt=15.0, min_samples=3)
    assert not fitter.add_sample(0.0, 0.0, 3.0)
    assert fitter.add_sample(0.0, 0.0, 20.0)
    assert len(fitter) == 1
