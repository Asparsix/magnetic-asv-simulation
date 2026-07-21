from boat_mapping.bayes_core import (
    BeliefMap,
    classify_observation,
    detection_probability,
)


def test_detection_probability_monotonic():
    near = detection_probability(0.0)
    mid = detection_probability(30.0)
    far = detection_probability(300.0)
    assert near > mid > far
    assert 0.05 <= far <= 0.2
    assert near > 0.9


def test_classify_observation():
    assert classify_observation(10.0, hit_threshold_nt=5.0, miss_threshold_nt=1.0) == 'HIT'
    assert classify_observation(0.2, hit_threshold_nt=5.0, miss_threshold_nt=1.0) == 'MISS'
    assert classify_observation(2.0, hit_threshold_nt=5.0, miss_threshold_nt=1.0) == 'ABSTAIN'


def test_belief_sums_to_one_after_hit():
    belief = BeliefMap(
        area_size_m=100.0,
        origin_x=0.0,
        origin_y=0.0,
        cell_size_m=20.0,
        hit_threshold_nt=5.0,
        miss_threshold_nt=1.0,
        hit_only=False,
    )
    label = belief.update(10.0, 10.0, anomaly_nt=20.0, is_calibrated=True)
    assert label == 'HIT'
    assert abs(sum(belief.belief) - 1.0) < 1e-9
    peak = belief.peak()
    assert peak.probability > 1.0 / len(belief.belief)


def test_hit_raises_probability_near_observation():
    belief = BeliefMap(
        area_size_m=100.0,
        origin_x=0.0,
        origin_y=0.0,
        cell_size_m=20.0,
        hit_threshold_nt=5.0,
        miss_threshold_nt=1.0,
    )
    prior = 1.0 / len(belief.belief)
    belief.update(10.0, 10.0, anomaly_nt=50.0, is_calibrated=True)
    peak = belief.peak()
    assert peak.probability > prior
    # Peak should be near the observation cell around (10,10)
    assert abs(peak.x - 10.0) < 20.0
    assert abs(peak.y - 10.0) < 20.0


def test_uncalibrated_does_not_update():
    belief = BeliefMap(
        area_size_m=60.0,
        origin_x=0.0,
        origin_y=0.0,
        cell_size_m=20.0,
        hit_threshold_nt=1.0,
        miss_threshold_nt=0.1,
    )
    before = list(belief.belief)
    label = belief.update(10.0, 10.0, anomaly_nt=100.0, is_calibrated=False)
    assert label == 'ABSTAIN'
    assert belief.belief == before
    assert belief.update_count == 0


def test_hit_only_skips_miss():
    belief = BeliefMap(
        area_size_m=60.0,
        origin_x=0.0,
        origin_y=0.0,
        cell_size_m=20.0,
        hit_threshold_nt=10.0,
        miss_threshold_nt=1.0,
        hit_only=True,
    )
    before = list(belief.belief)
    label = belief.update(10.0, 10.0, anomaly_nt=0.1, is_calibrated=True)
    assert label == 'ABSTAIN'
    assert belief.belief == before
