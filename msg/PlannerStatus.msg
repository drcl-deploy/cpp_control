# The planner's whole state, at the planner's own rate — the viser "the planner — state"
# pane as a topic. Published EVERY tick, not once per act: the belief and the
# candidate plan both move between commits, and a bag with one sample per act
# cannot show why a decision came out the way it did.
# scripts/planners/repose/repose_console.py renders it and sets target_color back.

uint8 INIT=0
uint8 SETTLE=1
uint8 SCAN=2
uint8 CLIP=3
uint8 APPROACH=4

# what the controller is playing right now (the last committed plan)
uint8 mode
string label            # 'settle' | 'scan' | '<delta>#<clip>'
float32 cost            # metres of stance residual the controller must self-correct
float32 entry_yaw
float32 entry_translation_m
float32 entry_bearing_rad
float32 entry_forward_m
float32 entry_lateral_m
float32 approach_requested_m
float32 approach_covered_m
int32 frames
int32 lead_in_frames
bool committed          # this tick is the one that published the reference

# what the planner WOULD publish if the controller finished now — decide() is pure and
# runs every tick, so intent is visible ~a second before it becomes an act
uint8 cand_mode
string cand_label
float32 cand_cost
float32 cand_entry_translation_m
float32 cand_entry_bearing_rad
float32 cand_entry_forward_m
float32 cand_entry_lateral_m
bool candidate_locked
int32 candidate_lock_row
int32 candidate_lock_sym
int32 search_phase             # -1 outside a stateful blind-search act

# v8.5 approach outcome, evaluated from fresh RGB-D after the walk settles
float32 approach_progress_m
int32 approach_attempts
int32 approach_no_progress
bool approach_only
bool approach_turn_locked     # selected row/sym held across an in-place turn
int32 approach_turn_attempts  # hard-capped; cannot oscillate forever
int32 approach_turn_row
int32 approach_turn_sym

# the ladder
uint8 delta             # ascii rung letter being asked for
int32 rung              # ladder position n
int32 tips              # colour changes actually SEEN
int32 stall             # commits at this rung with no tip
int32 burned            # clips tried at this rung
int32 rolls             # clips committed this episode
int32 episode           # bumped by a target change or a re-arm
int32 target_color
bool done               # LATCHED; cleared by re-picking a target colour

# the belief: a vote over the reads taken while the camera was quiet
int32 n_reads           # accepted reads since the last commit; 0 = BLIND
int32 n_votes           # of those, how many agreed on `color`
float32 omega_cam       # rad/s of camera motion, the quiescence gate's input
float32 accept_rate     # frames read / frames seen outside a clip, since
                        # boot — i.e. how often the quiescence gate opens
bool sees               # the vote carried
bool pose_ok            # ...and an agreeing read placed the cube
int32 color
string reason           # newest read's verdict, or 'blind'
float32 range_m
float32 bearing_rad
float32 phi_rad
int32 n_px

string version          # v6 | v7 | v7.1 | v8 | v8.5 | v9 | v9.1
float32 observe_ms
