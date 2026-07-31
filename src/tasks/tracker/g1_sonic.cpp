#include "cpp_control/tasks/tracker/g1_sonic.hpp"

#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include "common/math_utils.hpp"

namespace cpp_control
{

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
    start_frame_ = this->declare_parameter("motion_start_frame", 0);
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
    clip_ = std::make_unique<MotionClip>(MotionClip::load(motion_path, perm));
    playback_ = std::make_unique<MotionPlayback>(*clip_, anchor_body_);

    init();
    apply_manifest_action_meta();

    if (config_ && std::abs(config_->control_dt - manifest_.step_dt) > 1e-6)
        RCLCPP_WARN(this->get_logger(),
                    "control_dt %.4f != manifest step_dt %.4f — timer runs at control_dt",
                    config_->control_dt, manifest_.step_dt);

    policy_actions_.assign(G1_NUM_MOTOR, 0.0f);
    session_ = std::make_unique<deploy::OnnxSession>(onnx_path);
    output_name_ = manifest_.outputs.front();
    bind_ports();

    RCLCPP_INFO(this->get_logger(),
                "g1_sonic ready: %s (%s) | %zu input ports | clip %d frames @ %.0f fps",
                onnx_path.c_str(), manifest_.model_class.c_str(),
                manifest_.inputs.size(), clip_->num_frames, static_cast<double>(clip_->fps));
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

    std::ostringstream known;
    known << "base_ang_vel joint_pos joint_vel actions gravity_dir g1_tokenizer";
    throw std::runtime_error("g1_sonic: no writer for term '" + spec.name +
                             "' — known terms: " + known.str());
}

// ── Tokenizer (verbatim sonic_g1_tokenizer op-chain) ─────────────
//
// flat = [jp(f0..fF) | jv(f0..fF)] reinterpreted as (F, 2J) rows; per-frame
// output row = [flat_row(2J) | 6D of quat_inv(robot_pelvis) * ref_pelvis(f)].

void G1SonicNode::fill_tokenizer(float* dst)
{
    const int F = future_steps_, J = clip_->num_joints, skip = frame_skip_;

    for (int s = 0; s < F; ++s)
    {
        const int f = playback_->future_frame(s * skip);
        std::memcpy(&tokenizer_flat_[s * J], clip_->jp(f), J * sizeof(float));
        std::memcpy(&tokenizer_flat_[(F + s) * J], clip_->jv(f), J * sizeof(float));
    }

    const auto& robot_quat = robot_state_.imu_quaternion;
    const int row = 2 * J + 6;
    for (int s = 0; s < F; ++s)
    {
        std::memcpy(dst + s * row, &tokenizer_flat_[s * 2 * J], 2 * J * sizeof(float));
        const int f = playback_->future_frame(s * skip);
        auto rot_dif = math::qmul(math::qinv(robot_quat), playback_->aligned_anchor_quat(f));
        auto r6d = math::quat_to_rotation_6d(rot_dif);
        std::memcpy(dst + s * row + 2 * J, r6d.data(), 6 * sizeof(float));
    }
}

// ── Engage / reset ───────────────────────────────────────────────

void G1SonicNode::engage_reset()
{
    playback_->start(robot_state_.imu_quaternion, start_frame_);
    for (auto& h : histories_)
        h->reset();
    std::fill(policy_actions_.begin(), policy_actions_.end(), 0.0f);
    clip_end_logged_ = false;
    RCLCPP_INFO(this->get_logger(), "sonic engaged: frame %d, heading aligned", start_frame_);
}

// ── Control ──────────────────────────────────────────────────────

RobotCommand G1SonicNode::policy_control()
{
    const double dt = config_ ? config_->control_dt : 0.02;

    // Fresh engage = first policy tick after any other mode (mode switches
    // don't reach Level 2, so detect the gap in policy_control call times).
    const auto now = this->now();
    if ((now - last_policy_tick_).seconds() > 5.0 * dt)
        engage_reset();
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

    playback_->step(dt);
    if (playback_->finished() && !clip_end_logged_)
    {
        clip_end_logged_ = true;
        RCLCPP_INFO(this->get_logger(), "clip finished — holding last frame");
    }

    return cmd;
}

}  // namespace cpp_control

// ── Entry point ──────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1SonicNode>());
    rclcpp::shutdown();
    return 0;
}
