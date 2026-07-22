# Calibration node status for mission gating.
std_msgs/Header header

# CALIBRATING | READY | DEGRADED
string phase

int32 cells_sampled
float32 coverage_percent
