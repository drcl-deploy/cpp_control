# One depth-bearing camera record evaluated by the production CubeSight.
# All image topics from the observe node carry this exact header stamp.

std_msgs/Header header
uint64 sequence

# Acquisition interlocks at this frame.
bool state_ready
bool stand_locked

# Production Sight result.
bool color_ok
bool pose_ok
int32 color
string reason
float32[3] position
float32 phi_rad
int32 n_px
float32 range_m
float32 bearing_rad

# Gravity-aligned base <- camera transform used for this observation. These
# make a bag replay independent of later LowState synchronization choices.
float32[9] camera_rotation
float32[3] camera_translation

# Every geometric candidate, including candidates rejected by up_dot/big/vis.
# Parallel arrays; candidate_color.size() is the common length.
int32[] candidate_color
int32[] candidate_pixels
float32[] candidate_up_dot
float32[] candidate_visible
float32[] candidate_big
float32[] candidate_center_x
float32[] candidate_center_y
