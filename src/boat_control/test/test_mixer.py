from boat_control.core import mix_thrust


def test_forward_thrust_is_equal():
    assert mix_thrust(0.5, 0.0, 50.0, 1.0, 100.0) == (25.0, 25.0)


def test_positive_yaw_uses_differential_thrust():
    assert mix_thrust(0.0, 0.5, 50.0, 1.0, 100.0) == (-25.0, 25.0)


def test_thrust_is_clamped():
    assert mix_thrust(10.0, 0.0, 50.0, 1.0, 100.0) == (100.0, 100.0)
