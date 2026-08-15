# The planner's whole state, once per decision — the viser "sys1 — state" pane
# as a topic. scripts/sys1_console.py renders it and sets target_color back.

uint8 INIT=0
uint8 SETTLE=1
uint8 SCAN=2
uint8 CLIP=3

uint8 mode
string label            # 'settle' | 'scan' | '<delta>#<clip>'
uint8 delta             # ascii rung letter being asked for
int32 rung              # ladder position n
int32 tips              # colour changes actually SEEN
int32 stall             # commits at this rung with no tip
int32 burned            # clips tried at this rung
int32 target_color
bool done

# the read this decision consumed
bool sees               # a top face was proven horizontal
bool pose_ok            # ...and whole enough to carry a pose
int32 color
string reason
float32 range_m
float32 bearing_rad
float32 phi_rad
int32 n_px

# the reference it published
float32 cost            # metres of stance residual sys0 must self-correct
float32 entry_yaw
int32 frames
int32 lead_in_frames
string version          # v5 | v6
float32 observe_ms
