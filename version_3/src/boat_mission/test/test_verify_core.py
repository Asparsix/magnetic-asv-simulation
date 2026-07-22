"""Unit tests for Phase 8 verification helpers and orbit path."""

from boat_mission.path_planning import generate_verification_orbit
from boat_mission.verify_core import VerificationTracker, reading_confirms_candidate


def test_verification_orbit_closed():
    orbit = generate_verification_orbit((80.0, -40.0), radius=20.0, num_points=12)
    assert len(orbit) >= 12
    assert abs(orbit[0][0] - orbit[-1][0]) < 1e-9
    assert abs(orbit[0][1] - orbit[-1][1]) < 1e-9


def test_verification_orbit_opposite_approach():
    """Peer verifier enters on the side opposite the discoverer."""
    from boat_mission.path_planning import opposite_approach_angle

    candidate = (0.0, 0.0)
    discoverer = (-40.0, 0.0)  # west of candidate → approach from east
    angle = opposite_approach_angle(candidate, discoverer)
    assert abs(angle - 0.0) < 1e-9

    orbit = generate_verification_orbit(
        candidate, radius=20.0, num_points=8, start_angle=angle
    )
    # First waypoint should be on the +X (east) side.
    assert orbit[0][0] > 0.0
    assert abs(orbit[0][1]) < 1e-6


def test_reading_confirms_near_candidate():
    assert reading_confirms_candidate(
        pose_xy=(-48.0, 60.0),
        candidate_xy=(-50.0, 60.0),
        peak_xy=(-50.0, 60.0),
        peak_p=0.8,
        cleaned_anomaly_nt=50.0,
        confirmation_threshold_nt=15.0,
    )
    assert not reading_confirms_candidate(
        pose_xy=(100.0, 100.0),
        candidate_xy=(-50.0, 60.0),
        peak_xy=(-50.0, 60.0),
        peak_p=0.8,
        cleaned_anomaly_nt=50.0,
        confirmation_threshold_nt=15.0,
    )


def test_tracker_reaches_required_confirmations():
    tracker = VerificationTracker(
        confirmations_required=3,
        confirmation_threshold_nt=15.0,
        min_peak_probability=0.3,
    )
    tracker.start((-50.0, 60.0))
    for _ in range(2):
        done = tracker.register(
            (-49.0, 60.0), (-50.0, 60.0), 0.9, 50.0
        )
        assert not done
    assert tracker.confirmations == 2
    assert tracker.register((-49.0, 60.0), (-50.0, 60.0), 0.9, 50.0)
    assert tracker.complete
