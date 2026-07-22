"""ROS-independent thrust-mixing calculations."""


def mix_thrust(linear, angular, thrust_scale, turn_gain, max_thrust):
    """Return bounded left and right thrust commands."""
    left = (linear - angular * turn_gain) * thrust_scale
    right = (linear + angular * turn_gain) * thrust_scale
    return (
        max(-max_thrust, min(max_thrust, left)),
        max(-max_thrust, min(max_thrust, right)),
    )
