# Timestamped magnetometer sample synchronized with pose/heading.
std_msgs/Header header

# Magnetic field components in nanotesla (nT).
float64 bx
float64 by
float64 bz
float64 scalar

# Planar pose and heading at sample time.
float64 x
float64 y
float64 heading

# True when thrusters are commanded non-zero (for motor-state bins later).
bool motor_on
