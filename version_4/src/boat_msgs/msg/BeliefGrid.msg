# Shared Bayesian belief grid over the search area.
# data is row-major; values should sum approximately to 1.0.
std_msgs/Header header

float32 resolution
float64 origin_x
float64 origin_y
uint32 width
uint32 height
float64[] data
