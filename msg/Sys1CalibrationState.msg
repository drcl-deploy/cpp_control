# Offboard snapshot label. The viewer copies the selected observation header,
# so an offline reader can join the exact RGB-D record without wall-clock
# inference. expected_color=-1 is the negative/no-cube stage. `capturing` is
# true only for a selected snapshot; samples_per_color is this stage's target.

std_msgs/Header header
uint64 source_sequence
int32 expected_color
uint32 stage_index
uint32 attempt
uint32 sample_index
uint32 samples_per_color
bool capturing
bool complete
