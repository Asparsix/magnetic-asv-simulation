from boat_sensing.filter_core import (
    MagnetometerFilterChain,
    MovingAverageLowPass,
    SpikeRejectFilter,
    tesla_to_nt,
    vector_scalar,
)


def test_tesla_to_nt():
    assert tesla_to_nt(1.0e-9) == 1.0
    assert abs(tesla_to_nt(4.5e-5) - 45000.0) < 1e-6


def test_vector_scalar():
    assert abs(vector_scalar(3.0, 4.0, 0.0) - 5.0) < 1e-9


def test_moving_average_low_pass():
    filt = MovingAverageLowPass(window_size=3)
    assert filt.update(1.0) == 1.0
    assert abs(filt.update(3.0) - 2.0) < 1e-9
    assert abs(filt.update(5.0) - 3.0) < 1e-9


def test_spike_reject_blocks_outlier():
    filt = SpikeRejectFilter(history_size=10, n_sigma=3.0, min_std_nt=1.0)
    for value in [10.0, 10.1, 9.9, 10.2, 9.8, 10.0]:
        accepted, _ = filt.accept(value)
        assert accepted
    accepted, held = filt.accept(1000.0)
    assert not accepted
    assert abs(held - 10.0) < 0.3
    assert filt.rejected_count == 1


def test_spike_adapts_after_sustained_step():
    filt = SpikeRejectFilter(history_size=10, n_sigma=3.0, min_std_nt=1.0)
    for value in [10.0, 10.1, 9.9, 10.2, 9.8, 10.0]:
        filt.accept(value)
    accepted = [filt.accept(1000.0)[0] for _ in range(5)]
    assert accepted[:4] == [False, False, False, False]
    assert accepted[4] is True


def test_filter_chain_smooths_and_rejects():
    chain = MagnetometerFilterChain(
        lowpass_window=3,
        spike_history=10,
        spike_n_sigma=3.0,
        min_std_nt=1.0,
    )
    for _ in range(6):
        out = chain.update(45000.0, 0.0, 0.0)
        assert abs(out['bx'] - 45000.0) < 1.0
    out = chain.update(45000.0 + 5000.0, 0.0, 0.0)
    assert chain.last_rejected
    assert abs(out['bx'] - 45000.0) < 50.0
