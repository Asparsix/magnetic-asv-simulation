"""Unit tests for mutual-information info-gain planner."""

import math

from boat_mission.info_gain import (
    InfoGainPlanner,
    belief_entropy,
    mutual_information_gain,
    posterior_after_reading,
)


def _peaked_belief(width=5, height=5, peak_i=3, peak_j=3, peak_mass=0.7):
    n = width * height
    background = (1.0 - peak_mass) / (n - 1)
    belief = [background] * n
    belief[peak_j * width + peak_i] = peak_mass
    return belief


def test_belief_entropy_uniform_higher_than_peaked():
    n = 25
    uniform = [1.0 / n] * n
    peaked = _peaked_belief()
    assert belief_entropy(uniform) > belief_entropy(peaked)


def test_posterior_hit_concentrates_near_asv():
    belief = [1.0 / 4] * 4
    # High detect on first cell only
    p_detect = [0.9, 0.1, 0.1, 0.1]
    post = posterior_after_reading(belief, p_detect, True)
    assert abs(sum(post) - 1.0) < 1e-9
    assert post[0] > post[1]


def test_mi_higher_near_peak_than_far():
    width = height = 7
    cell = 20.0
    origin = -70.0
    peak_i = peak_j = 5  # near (40, 40)
    belief = _peaked_belief(width, height, peak_i, peak_j, peak_mass=0.6)
    centers = [
        (origin + (i + 0.5) * cell, origin + (j + 0.5) * cell)
        for j in range(height)
        for i in range(width)
    ]
    near = (40.0, 40.0)
    far = (-60.0, -60.0)
    gain_near = mutual_information_gain(belief, near, centers, d_half=30.0)
    gain_far = mutual_information_gain(belief, far, centers, d_half=30.0)
    assert gain_near > gain_far
    assert gain_near > 0.0


def test_planner_picks_candidate_toward_peak():
    planner = InfoGainPlanner(
        radii=[20.0],
        num_angles=8,
        d_half=30.0,
    )
    width = height = 15
    cell = 20.0
    origin = -150.0
    # Peak cell around (80, -40) → i≈11, j≈5
    peak_i, peak_j = 11, 5
    belief = _peaked_belief(width, height, peak_i, peak_j, peak_mass=0.55)
    assert planner.update_belief_grid(
        belief, origin, origin, cell, width, height
    )
    current = (40.0, -40.0)
    bounds = ((-120.0, -120.0), (120.0, 120.0))
    best, gain = planner.plan(current, bounds=bounds, peak_xy=(80.0, -40.0))
    assert gain > 0.0
    # Should move east toward the peak, not west
    assert best[0] > current[0] - 1e-6
    assert math.hypot(best[0] - 80.0, best[1] + 40.0) < math.hypot(
        current[0] - 80.0, current[1] + 40.0
    ) + 25.0


def test_planner_fallback_without_belief():
    planner = InfoGainPlanner(radii=[10.0], num_angles=4)
    best, gain = planner.plan(
        (0.0, 0.0),
        bounds=((-50.0, -50.0), (50.0, 50.0)),
        peak_xy=(30.0, 0.0),
    )
    assert gain == 0.0
    assert best[0] > 0.0
