# Versioned, atomic reference envelope for SONIC/Vibe controllers.
#
# data is a row-major [frames, cols] array in IsaacLab joint order. Version 1
# always uses the 83-column full layout documented by common/g1/motion.hpp:
#   jp29 | jv29 | anchor_pos3 | anchor_quat_wxyz4 |
#   anchor_lin_vel3 | anchor_ang_vel3 | bodywise_contact12
# The validity flags are independent: absent channels are zero-filled in data.

uint8 SCHEMA_VERSION=1

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

float32[] data
