# Per-ASV mission state machine snapshot.
std_msgs/Header header

# GLOBAL_SEARCH | TARGET_SEARCH | VERIFY | HOLD | COMPLETE | ...
string mode

# INFO_GAIN | SPIRAL | empty when unused
string hunt_phase

float64 peak_p
float64 peak_x
float64 peak_y

int32 info_gain_steps
int32 confirmations
