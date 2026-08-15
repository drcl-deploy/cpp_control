# Versioned, atomic reference envelope for SONIC/Vibe controllers.
#
# data is a row-major [frames, cols] array in IsaacLab joint order, always the
# 83-column full layout documented by common/g1/motion.hpp:
#   jp29 | jv29 | anchor_pos3 | anchor_quat_wxyz4 |
#   anchor_lin_vel3 | anchor_ang_vel3 | bodywise_contact12
# The validity flags are independent: absent channels are zero-filled in data.
#
# v2 adds entry_yaw_offset. MotionClock heading-aligns frame 0 onto the robot,
# which forces the reference's relative heading there to ZERO — and no amount of
# pre-rotating the rows survives that, because the alignment cancels it exactly.
# A planner that CHOSE a heading residual (sys1's cube-anchored warp) therefore
# needs one scalar of its own. 0.0 is the pre-v2 behaviour.

uint8 SCHEMA_VERSION=2

uint8 schema_version
string reference_id
uint32 frames
uint32 cols
float32 fps

bool has_twist
bool has_contact
bool has_object_goal
float32[3] object_goal_pos
float32[4] object_goal_quat_wxyz

# Extra yaw applied on top of the heading alignment at engage, radians.
float32 entry_yaw_offset
# Free-form producer tag for telemetry (sys1: settle | scan | clip).
uint8 mode

float32[] data
