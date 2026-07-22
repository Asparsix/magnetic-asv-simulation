# Coordinator request for a verifier ASV to confirm a candidate.
std_msgs/Header header

string discoverer_id
string verifier_id
float64 candidate_x
float64 candidate_y
float64 candidate_peak_p
# Discoverer's pose at declaration time (for opposite-side approach).
float64 discoverer_x
float64 discoverer_y
