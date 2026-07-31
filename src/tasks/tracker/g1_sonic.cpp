#include "cpp_control/tasks/tracker/g1_sonic.hpp"

#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include "common/math_utils.hpp"

namespace cpp_control
{

// IL (IsaacLab BFS) -> MJ (MuJoCo DFS) joint permutation for G1 29-dof.
// Single source of truth: mocke/mdp/joint_maps.py (retargeted motion.npz
// stores joints IL-ordered; pelvis is body 0 in BOTH body orderings).
static const std::vector<int> G1_IL2MJ = {
    0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18,
    2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28,
};

// ── Constructor ──────────────────────────────────────────────────

G1SonicNode::G1SonicNode(const std::string& node_name)
    : G1Node(node_name), last_policy_tick_(0, 0, RCL_ROS_TIME)
{
    const std::string onnx_path = this->declare_parameter("onnx_path", "");
    std::string manifest_path = this->declare_parameter("manifest_path", "");
    const std::string motion_path = this->declare_parameter("motion_path", "");
    future_steps_ = this->declare_parameter("future_steps", 10);
    frame_skip_ = this->declare_parameter("frame_skip", 5);
    anchor_body_ = this->declare_parameter("anchor_body_index", 0);
    cmd_frame_skip_ = this->declare_parameter("cmd_frame_skip", 1);
    start_frame_ = this->declare_parameter("motion_start_frame", 0);
    const bool il_ordered = this->declare_parameter("il_ordered", false);
    auto joint_perm = this->declare_parameter("motion_joint_perm", std::vector<int64_t>{});

    if (onnx_path.empty() || motion_path.empty())
        throw std::runtime_error("g1_sonic: onnx_path and motion_path are required");
    if (manifest_path.empty())
    {
        // <model>.onnx → <model>.manifest.json (the exporter writes them side by side)
        manifest_path = onnx_path.substr(0, onnx_path.rfind(".onnx")) + ".manifest.json";
    }

    manifest_ = deploy::DeployManifest::load(manifest_path);
    std::vector<int> perm(joint_perm.begin(), joint_perm.end());
    if (il_ordered)
    {
        if (!perm.empty())
            throw std::runtime_error("g1_sonic: il_ordered and motion_joint_perm are exclusive");
        perm = G1_IL2MJ;
    }
    clip_ = std::make_unique<MotionClip>(MotionClip::load(motion_path, perm));
    playback_ = std::make_unique<MotionPlayback>(*clip_, anchor_body_);

    init();
    apply_manifest_action_meta();
    make_stand_clip();
    active_clip_ = clip_.get();
    active_pb_ = playback_.get();

    if (config_ && std::abs(config_->control_dt - manifest_.step_dt) > 1e-6)
        RCLCPP_WARN(this->get_logger(),
                    "control_dt %.4f != manifest step_dt %.4f — timer runs at control_dt",
                    config_->control_dt, manifest_.step_dt);

    policy_actions_.assign(G1_NUM_MOTOR, 0.0f);
    session_ = std::make_unique<deploy::OnnxSession>(onnx_path);
    output_name_ = manifest_.outputs.front();
    bind_ports();

    RCLCPP_INFO(this->get_logger(),
                "g1_sonic ready: %s (%s) | %zu input ports | clip %d frames @ %.0f fps%s",
                onnx_path.c_str(), manifest_.model_class.c_str(),
                manifest_.inputs.size(), clip_->num_frames, static_cast<double>(clip_->fps),
                il_ordered ? " (IL->MJ remapped)" : "");
}

// ── Stand reference (textop enter_stand_mode equivalent) ─────────

void G1SonicNode::make_stand_clip()
{
    auto stand = std::make_unique<MotionClip>();
    stand->num_frames = 1;
    stand->num_joints = G1_NUM_MOTOR;
    stand->num_bodies = 1;
    stand->fps = clip_->fps;
    stand->joint_pos.assign(default_angles_.begin(), default_angles_.end());
    stand->joint_vel.assign(G1_NUM_MOTOR, 0.0f);
    stand->body_quat_w = {1.0f, 0.0f, 0.0f, 0.0f};
    stand_clip_ = std::move(stand);
    stand_playback_ = std::make_unique<MotionPlayback>(*stand_clip_, 0);
}

void G1SonicNode::enter_stand()
{
    stand_mode_ = true;
    pending_engage_ = true;
    control_mode_ = ControlMode::POLICY;
}

// ── Manifest is the authority on action metadata ─────────────────

void G1SonicNode::apply_manifest_action_meta()
{
    const auto& a = manifest_.action;
    if (static_cast<int>(a.joint_names.size()) != G1_NUM_MOTOR)
        throw std::runtime_error("g1_sonic: manifest has " +
                                 std::to_string(a.joint_names.size()) + " joints, expected 29");
    // The checkpoint's joint order must equal the hardware motor order
    // (G1 29-dof: unitree_hg index == mjlab MJ index).
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
        if (!joint_names_.empty() && joint_names_[i] != a.joint_names[i])
            throw std::runtime_error("g1_sonic: joint order mismatch at " + std::to_string(i) +
                                     ": config '" + joint_names_[i] + "' vs manifest '" +
                                     a.joint_names[i] + "'");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        default_angles_[i] = a.default_joint_pos[i];
        kps_[i] = a.stiffness[i];
        kds_[i] = a.damping[i];
        action_scale_[i] = a.scale[i];
    }
}

// ── Port binding ─────────────────────────────────────────────────

void G1SonicNode::bind_ports()
{
    for (const auto& port : manifest_.inputs)
    {
        if (!session_->has_input(port.name))
            throw std::runtime_error("g1_sonic: manifest port '" + port.name +
                                     "' not found in the ONNX graph");
        float* buf = session_->input(port.name);
        if (static_cast<int>(session_->input_dim(port.name)) != port.dim())
            throw std::runtime_error("g1_sonic: port '" + port.name + "' dim mismatch: graph " +
                                     std::to_string(session_->input_dim(port.name)) +
                                     " vs manifest " + std::to_string(port.dim()));
        for (const auto& term : port.terms)
        {
            if (term.offset + term.dim > port.dim())
                throw std::runtime_error("g1_sonic: term '" + term.name +
                                         "' overruns port '" + port.name + "'");
            bindings_.push_back(make_binding(buf + term.offset, term));
        }
    }
}

G1SonicNode::Binding G1SonicNode::make_binding(float* dst, const deploy::TermSpec& spec)
{
    auto history_binding = [&](int width, std::function<void(float*)> compute) -> Binding {
        if (spec.history <= 0 || spec.dim != width * spec.history)
            throw std::runtime_error("g1_sonic: term '" + spec.name + "' dim " +
                                     std::to_string(spec.dim) + " != width " +
                                     std::to_string(width) + " x history " +
                                     std::to_string(spec.history));
        auto hist = std::make_unique<obs::HistoryTerm>(width, spec.history);
        obs::HistoryTerm* h = hist.get();
        histories_.push_back(std::move(hist));
        history_updates_.push_back([h, compute, width]() {
            // scratch on the stack is fine: widths are tiny (3 or 29)
            float value[64];
            compute(value);
            h->push(value);
        });
        return {dst, &spec, [h](float* out) { h->write(out); }};
    };

    // ── proprio terms (mocke/sonic/profile.py policy_obs_terms) ──
    if (spec.name == "base_ang_vel")
        return history_binding(3, [this](float* v) {
            const auto& g = robot_state_.imu_gyroscope;
            v[0] = g[0]; v[1] = g[1]; v[2] = g[2];
        });
    if (spec.name == "joint_pos")
        return history_binding(G1_NUM_MOTOR, [this](float* v) {
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
                v[i] = robot_state_.joint_positions[i] - default_angles_[i];
        });
    if (spec.name == "joint_vel")
        return history_binding(G1_NUM_MOTOR, [this](float* v) {
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
                v[i] = robot_state_.joint_velocities[i];
        });
    if (spec.name == "actions")
        return history_binding(G1_NUM_MOTOR, [this](float* v) {
            std::memcpy(v, policy_actions_.data(), G1_NUM_MOTOR * sizeof(float));
        });
    if (spec.name == "gravity_dir")
        return history_binding(3, [this](float* v) {
            auto g = math::get_projected_gravity(robot_state_.imu_quaternion);
            v[0] = g[0]; v[1] = g[1]; v[2] = g[2];
        });

    // ── tokenizer term (mocke/sonic/mdp/observations.py sonic_g1_tokenizer) ──
    if (spec.name == "g1_tokenizer")
    {
        const int expected = future_steps_ * (2 * clip_->num_joints + 6);
        if (spec.dim != expected || clip_->num_joints != G1_NUM_MOTOR)
            throw std::runtime_error("g1_sonic: tokenizer dim " + std::to_string(spec.dim) +
                                     " != F*(2J+6) = " + std::to_string(expected));
        tokenizer_flat_.assign(2 * future_steps_ * clip_->num_joints, 0.0f);
        return {dst, &spec, [this](float* out) { fill_tokenizer(out); }};
    }

    // ── adapter stream (mocke env_cfg adapter=True: FutureMotionCommand.command) ──
    // [jp(f0..fF-1).flat | jv(f0..fF-1).flat] at cmd_frame_skip; F from the dim.
    if (spec.name == "motion_cmd")
    {
        const int J = clip_->num_joints;
        if (spec.dim % (2 * J) != 0)
            throw std::runtime_error("g1_sonic: motion_cmd dim " + std::to_string(spec.dim) +
                                     " not divisible by 2*J");
        const int F = spec.dim / (2 * J);
        return {dst, &spec, [this, F, J](float* out) {
                    for (int s = 0; s < F; ++s)
                    {
                        const int f = active_pb_->future_frame(s * cmd_frame_skip_);
                        std::memcpy(out + s * J, active_clip_->jp(f), J * sizeof(float));
                        std::memcpy(out + (F + s) * J, active_clip_->jv(f), J * sizeof(float));
                    }
                }};
    }

    std::ostringstream known;
    known << "base_ang_vel joint_pos joint_vel actions gravity_dir g1_tokenizer motion_cmd";
    throw std::runtime_error("g1_sonic: no writer for term '" + spec.name +
                             "' — known terms: " + known.str());
}

// ── Tokenizer (verbatim sonic_g1_tokenizer op-chain) ─────────────
//
// flat = [jp(f0..fF) | jv(f0..fF)] reinterpreted as (F, 2J) rows; per-frame
// output row = [flat_row(2J) | 6D of quat_inv(robot_pelvis) * ref_pelvis(f)].

void G1SonicNode::fill_tokenizer(float* dst)
{
    const int F = future_steps_, J = active_clip_->num_joints, skip = frame_skip_;

    for (int s = 0; s < F; ++s)
    {
        const int f = active_pb_->future_frame(s * skip);
        std::memcpy(&tokenizer_flat_[s * J], active_clip_->jp(f), J * sizeof(float));
        std::memcpy(&tokenizer_flat_[(F + s) * J], active_clip_->jv(f), J * sizeof(float));
    }

    const auto& robot_quat = robot_state_.imu_quaternion;
    const int row = 2 * J + 6;
    for (int s = 0; s < F; ++s)
    {
        std::memcpy(dst + s * row, &tokenizer_flat_[s * 2 * J], 2 * J * sizeof(float));
        const int f = active_pb_->future_frame(s * skip);
        auto rot_dif = math::qmul(math::qinv(robot_quat), active_pb_->aligned_anchor_quat(f));
        auto r6d = math::quat_to_rotation_6d(rot_dif);
        std::memcpy(dst + s * row + 2 * J, r6d.data(), 6 * sizeof(float));
    }
}

// ── Engage / reset ───────────────────────────────────────────────

void G1SonicNode::engage_reset()
{
    active_clip_ = stand_mode_ ? stand_clip_.get() : clip_.get();
    active_pb_ = stand_mode_ ? stand_playback_.get() : playback_.get();
    active_pb_->start(robot_state_.imu_quaternion, stand_mode_ ? 0 : start_frame_);
    for (auto& h : histories_)
        h->reset();
    std::fill(policy_actions_.begin(), policy_actions_.end(), 0.0f);
    clip_end_logged_ = false;
    if (stand_mode_)
        RCLCPP_INFO(this->get_logger(), "sonic engaged: STAND (nominal-pose reference)");
    else
        RCLCPP_INFO(this->get_logger(), "sonic engaged: track from frame %d, heading aligned",
                    start_frame_);
}

// ── Control ──────────────────────────────────────────────────────

RobotCommand G1SonicNode::policy_control()
{
    const double dt = config_ ? config_->control_dt : 0.02;

    // Fresh engage = first policy tick after any other mode (detected via the
    // gap in policy_control call times) OR an explicit stand<->track switch
    // (pending_engage_, since those never leave POLICY).
    const auto now = this->now();
    if (pending_engage_ || (now - last_policy_tick_).seconds() > 5.0 * dt)
    {
        engage_reset();
        pending_engage_ = false;
    }
    last_policy_tick_ = now;

    for (auto& update : history_updates_)
        update();
    for (auto& b : bindings_)
        b.write(b.dst);

    session_->run();
    const float* act = session_->output(output_name_);
    std::memcpy(policy_actions_.data(), act, G1_NUM_MOTOR * sizeof(float));
    std::copy(policy_actions_.begin(), policy_actions_.end(), actions_.begin());
    std::copy(policy_actions_.begin(), policy_actions_.end(), last_actions_.begin());

    RobotCommand cmd;
    cmd.motor_commands.resize(G1_NUM_MOTOR);
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        auto& mc = cmd.motor_commands[i];
        mc.q = default_angles_[i] + action_scale_[i] * policy_actions_[i];
        mc.kp = kps_[i];
        mc.kd = kds_[i];
    }

    active_pb_->step(dt);
    if (!stand_mode_ && active_pb_->finished() && !clip_end_logged_)
    {
        clip_end_logged_ = true;
        RCLCPP_INFO(this->get_logger(), "clip finished — holding last frame");
    }

    return cmd;
}

// ── Joystick / Gamepad (textop parity: RB/R1 = stand, A = track) ─

void G1SonicNode::on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
{
    const bool rb = msg->buttons.size() > joy::XMODE_R1 && msg->buttons[joy::XMODE_R1] == 1;
    if (rb && !prev_rb_joy_)
    {
        enter_stand();
        RCLCPP_INFO(this->get_logger(), "-> stand (SONIC @ nominal)");
    }
    prev_rb_joy_ = rb;

    // A (base already switched to POLICY): leave stand, (re)start the clip.
    if (msg->buttons.size() > joy::XMODE_A && msg->buttons[joy::XMODE_A] == 1)
    {
        stand_mode_ = false;
        pending_engage_ = true;
    }
}

#ifdef HAS_UNITREE_HG
void G1SonicNode::on_gamepad()
{
    if (gamepad_.R1.on_press)
    {
        enter_stand();
        RCLCPP_INFO(this->get_logger(), "[GP] -> stand (SONIC @ nominal)");
    }
    if (gamepad_.A.on_press)
    {
        stand_mode_ = false;
        pending_engage_ = true;
    }
}
#endif

}  // namespace cpp_control

// ── Entry point ──────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1SonicNode>());
    rclcpp::shutdown();
    return 0;
}
