# What sys0 is ACTUALLY playing. sys1 has no clock of its own: it times every
# mode off this, so a human pressing B mid-clip cannot leave the planner
# talking to itself. Published only when the node runs with sys1:=true.

uint8 ZEROING=0
uint8 DAMPING=1
uint8 NOMINAL_POSE=2
uint8 STANDING_UP=3
uint8 STAND=4
uint8 POLICY=5

uint8 control_mode
bool stand              # tracking the stand reference rather than a clip
bool accepting          # in POLICY and able to take a new reference

string reference_id     # the reference currently engaged ('' = none)
uint32 frame            # playback position within it
uint32 frames
bool finished           # holding the last frame

# The robot's NOMINAL STANCE, MJ order — the manifest's `default_joint_pos`,
# which is what sys0 actually holds in NOMINAL_POSE. sys1 builds its v7 still
# pose from this rather than parsing the manifest a second time: one source, so
# the pose the planner commands and the pose the controller stands in cannot
# drift. Constant, but it rides the status message so it can never be missed by
# a late subscriber or go stale. Empty until the manifest has loaded.
float32[] default_joint_pos
