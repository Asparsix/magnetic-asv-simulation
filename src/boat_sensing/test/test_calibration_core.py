import math

from boat_sensing.calibration_core import (
    BaselineMap,
    MagneticCalibrator,
    TemporalHighPass,
    heading_bin,
    wrap_heading,
)


def test_heading_bin_wraps():
    assert heading_bin(0.0, 8) == 4
    assert heading_bin(math.pi, 8) == 7
    assert heading_bin(-math.pi, 8) == 0
    assert abs(wrap_heading(3.0 * math.pi) - math.pi) < 1e-9 or abs(
        wrap_heading(3.0 * math.pi) + math.pi
    ) < 1e-9


def test_baseline_builds_and_estimates():
    baseline = BaselineMap(
        area_size_m=100.0,
        origin_x=0.0,
        origin_y=0.0,
        cell_size_m=20.0,
        num_heading_bins=4,
        reject_residual_nt=100.0,
    )
    assert baseline.update(10.0, 10.0, 0.0, 1000.0)
    value, calibrated, i, j, hbin = baseline.estimate_baseline(10.0, 10.0, 0.0)
    assert calibrated
    assert abs(value - 1000.0) < 1e-6
    assert i == 0 and j == 0
    assert 0 <= hbin < 4


def test_baseline_rejects_outlier_update():
    baseline = BaselineMap(
        area_size_m=100.0,
        origin_x=0.0,
        origin_y=0.0,
        cell_size_m=20.0,
        reject_residual_nt=50.0,
    )
    assert baseline.update(5.0, 5.0, 0.0, 1000.0)
    assert not baseline.update(5.0, 5.0, 0.0, 2000.0)
    value, calibrated, *_ = baseline.estimate_baseline(5.0, 5.0, 0.0)
    assert calibrated
    assert abs(value - 1000.0) < 1e-6


def test_temporal_highpass_boosts_local_spike():
    temporal = TemporalHighPass(window=8, noise_floor_nt=1.0)
    for _ in range(8):
        out = temporal.update(2.0, 1000.0)
        assert out >= 2.0
    boosted = temporal.update(50.0, 1050.0)
    assert boosted >= 50.0


def test_calibrator_moves_to_ready():
    baseline = BaselineMap(
        area_size_m=40.0,
        origin_x=0.0,
        origin_y=0.0,
        cell_size_m=20.0,
        reject_residual_nt=1.0e9,
    )
    calibrator = MagneticCalibrator(
        baseline_map=baseline,
        temporal=TemporalHighPass(window=6),
        ready_coverage_percent=50.0,
        ready_min_cells=2,
        freeze_baseline_when_ready=True,
    )
    assert calibrator.phase == MagneticCalibrator.PHASE_CALIBRATING
    sample1 = calibrator.process(5.0, 5.0, 0.0, 1000.0)
    sample2 = calibrator.process(25.0, 5.0, 0.0, 1010.0)
    assert sample1.is_calibrated or sample2.is_calibrated
    assert calibrator.phase == MagneticCalibrator.PHASE_READY
    ready_sample = calibrator.process(5.0, 5.0, 0.0, 1005.0)
    assert ready_sample.is_calibrated
    assert ready_sample.cleaned_anomaly_nt >= 0.0
