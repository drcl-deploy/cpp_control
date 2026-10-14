// Self-test for the deploy infra (manifest / history / g1 motion / tokenizer
// layout), plus an optional smoke run of a real export:
//
//   deploy_selftest                                  # unit checks only
//   deploy_selftest policy.onnx [policy.manifest.json]  # + load, zero-obs run
//   deploy_selftest --manifest policy.manifest.json  # + Vibe contract only
//   deploy_selftest --motion clip.npz                # + contact schedule report
//
// Plain asserts, no gtest — run it, exit 0 means pass.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "cnpy/cnpy.h"
#include "common/asset_path.hpp"
#include "common/deploy_manifest.hpp"
#include "common/g1/joint_orders.hpp"
#include "common/g1/motion.hpp"
#include "common/g1/stand_yaw.hpp"
#include "common/math_utils.hpp"
#include "common/obs_terms.hpp"
#include "common/onnx_session.hpp"
#include "cpp_control/tasks/vibe/task_profile.hpp"

using namespace cpp_control;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            std::exit(1);                                                    \
        }                                                                    \
    } while (0)

static std::string tmp_path(const std::string& name)
{
    const char* base = std::getenv("TMPDIR");
    return std::string(base ? base : "/tmp") + "/" + name;
}

// ── Stand yaw: deadband, acceleration, watchdog, and unwrapped turns ──

static void test_stand_yaw()
{
    g1::StandYawConfig config;
    config.enabled = true;
    config.max_rate = 1.0;
    config.max_accel = 2.0;
    config.deadband = 0.1;
    config.input_timeout = 0.25;
    g1::StandYaw yaw(config);

    yaw.set_input(0.05, 0.0);  // inside deadband
    yaw.step(0.0, 0.1, true);
    CHECK(yaw.rate() == 0.0 && yaw.angle() == 0.0);

    yaw.set_input(1.0, 0.1);
    yaw.step(0.1, 0.1, true);
    CHECK(std::fabs(yaw.rate() - 0.2) < 1e-9);
    CHECK(std::fabs(yaw.angle() - 0.01) < 1e-9);
    yaw.step(0.2, 0.1, true);
    CHECK(std::fabs(yaw.rate() - 0.4) < 1e-9);
    CHECK(std::fabs(yaw.angle() - 0.04) < 1e-9);
    CHECK(std::fabs(yaw.future_angle(0.3) - 0.25) < 1e-9);

    // A stale input brakes rather than continuing forever.
    yaw.step(0.4, 0.1, true);
    CHECK(std::fabs(yaw.rate() - 0.2) < 1e-9);
    yaw.step(0.5, 0.1, true);
    CHECK(std::fabs(yaw.rate()) < 1e-9);
    const double held = yaw.angle();
    yaw.step(0.6, 0.1, false);
    CHECK(std::fabs(yaw.angle() - held) < 1e-9);

    // Arming disables further stick drive and brakes without resetting angle.
    yaw.reset();
    yaw.set_input(1.0, 1.0);
    yaw.step(1.0, 0.1, true);
    yaw.set_input(1.0, 1.1);
    yaw.step(1.1, 0.1, false);
    CHECK(std::fabs(yaw.rate()) < 1e-9);
    CHECK(yaw.angle() > 0.0);

    // The state does not clamp at +/-pi: a held stick can keep turning.
    yaw.reset();
    for (int i = 0; i < 100; ++i) {
        const double now = i * 0.1;
        yaw.set_input(1.0, now);
        yaw.step(now, 0.1, true);
    }
    CHECK(yaw.angle() > 2.0 * 3.14159265358979323846);
    std::puts("ok  stand yaw (bounded rate, watchdog, unlimited heading)");
}

// ── HistoryTerm: mjlab CircularBuffer semantics ─────────────────

static void test_history()
{
    obs::HistoryTerm h(2, 3);
    std::vector<float> out(6);

    // First push backfills every row.
    float a[2] = {1.f, 2.f};
    h.push(a);
    h.write(out.data());
    for (int i = 0; i < 3; ++i)
        CHECK(out[i * 2] == 1.f && out[i * 2 + 1] == 2.f);

    // Oldest→newest flatten after rolling.
    float b[2] = {3.f, 4.f}, c[2] = {5.f, 6.f}, d[2] = {7.f, 8.f};
    h.push(b);
    h.push(c);
    h.write(out.data());
    CHECK(out[0] == 1.f && out[2] == 3.f && out[4] == 5.f);
    h.push(d);  // ring wraps, 'a' falls off
    h.write(out.data());
    CHECK(out[0] == 3.f && out[2] == 5.f && out[4] == 7.f);

    h.reset();
    h.push(d);
    h.write(out.data());
    CHECK(out[0] == 7.f && out[4] == 7.f);
    std::puts("ok  history");
}

// ── Manifest: vibe.onnx.v1 fixture ──────────────────────────────

static void test_manifest()
{
    const std::string path = tmp_path("g1_sonic_selftest_manifest.json");
    std::ofstream f(path);
    f << R"({
      "schema": "vibe.onnx.v1", "model_class": "SonicBaseModel", "checkpoint": "x.pt",
      "control": {"sim_timestep": 0.005, "decimation": 4, "step_dt": 0.02},
      "action": {"type": "joint_position",
                 "joint_names": ["j0", "j1"], "scale": 0.25,
                 "default_joint_pos": [0.1, -0.1],
                 "stiffness": [100.0, 40.0], "damping": [2.0, 1.0]},
      "inputs": [
        {"name": "policy", "shape": [63], "groups": ["policy"],
         "terms": [{"name": "base_ang_vel", "shape": [30], "dim": 30, "offset": 0,
                    "history_length": 10, "flatten_history_dim": true},
                   {"name": "gravity_dir", "shape": [33], "dim": 33, "offset": 30,
                    "history_length": 11, "flatten_history_dim": true}]},
        {"name": "tokenizer", "shape": [640], "groups": ["tokenizer"],
         "terms": [{"name": "g1_tokenizer", "shape": [640], "dim": 640, "offset": 0,
                    "history_length": 0, "flatten_history_dim": true}]}
      ],
      "outputs": ["actions"],
      "versions": {"vibe": "abc", "rsl_rl": "def"}
    })";
    f.close();

    auto m = deploy::DeployManifest::load(path);
    CHECK(m.step_dt == 0.02);
    CHECK(m.action.joint_names.size() == 2);
    CHECK(m.action.scale.size() == 2 && m.action.scale[1] == 0.25f);  // scalar broadcast
    CHECK(m.action.stiffness[0] == 100.f);
    CHECK(m.inputs.size() == 2);
    CHECK(m.find_input("policy") && m.find_input("policy")->dim() == 63);
    CHECK(m.find_input("policy")->terms[1].offset == 30);
    CHECK(m.find_input("policy")->terms[1].history == 11);
    CHECK(!m.find_input("nope"));
    CHECK(m.outputs[0] == "actions");
    std::puts("ok  manifest");
}

// ── g1::Motion + clock: clamp, heading alignment, twist, views ──

static void test_motion()
{
    const std::string path = tmp_path("g1_sonic_selftest_motion.npz");
    const int T = 6, J = 3, B = 2;
    std::vector<float> jp(T * J), jv(T * J), bq(T * B * 4, 0.f);
    std::vector<float> bp(T * B * 3, 0.f), blv(T * B * 3, 0.f), bav(T * B * 3, 0.f);
    for (int t = 0; t < T; ++t)
        for (int j = 0; j < J; ++j)
        {
            jp[t * J + j] = static_cast<float>(t * 10 + j);
            jv[t * J + j] = static_cast<float>(-(t * 10 + j));
        }
    // anchor (body 0): constant yaw = +90 deg; body 1: identity
    const float s = std::sin(M_PI / 4), c = std::cos(M_PI / 4);
    for (int t = 0; t < T; ++t)
    {
        bq[(t * B + 0) * 4 + 0] = c;
        bq[(t * B + 0) * 4 + 3] = s;
        bq[(t * B + 1) * 4 + 0] = 1.f;
        blv[(t * B + 0) * 3 + 0] = 1.f;  // anchor world vel = +x
    }
    std::vector<double> fps = {50.0};
    cnpy::npz_save(path, "joint_pos", jp.data(), {(size_t)T, (size_t)J}, "w");
    cnpy::npz_save(path, "joint_vel", jv.data(), {(size_t)T, (size_t)J}, "a");
    cnpy::npz_save(path, "body_pos_w", bp.data(), {(size_t)T, (size_t)B, 3}, "a");
    cnpy::npz_save(path, "body_quat_w", bq.data(), {(size_t)T, (size_t)B, 4}, "a");
    cnpy::npz_save(path, "body_lin_vel_w", blv.data(), {(size_t)T, (size_t)B, 3}, "a");
    cnpy::npz_save(path, "body_ang_vel_w", bav.data(), {(size_t)T, (size_t)B, 3}, "a");
    cnpy::npz_save(path, "fps", fps.data(), {1}, "a");

    auto mot = g1::Motion::from_npz(path);
    CHECK(mot.num_frames == T && mot.num_joints == J && mot.num_bodies == B);
    CHECK(mot.fps == 50.f);
    CHECK(mot.jp(2)[1] == 21.f && mot.jv(1)[0] == -10.f);

    // ref-anchor-frame twist: world +x through a +90° yaw anchor → body -y
    CHECK(mot.has_twist);
    auto v = mot.root_lin_vel_b(0);
    CHECK(std::fabs(v[0]) < 1e-5 && std::fabs(v[1] + 1.f) < 1e-5);

    g1::MotionClock clk(mot, 0);
    std::array<float, 4> robot_identity = {1.f, 0.f, 0.f, 0.f};
    clk.engage(robot_identity);
    CHECK(clk.frame() == 0);
    // future clamp: mirror FutureMotionCommand.future_frames
    CHECK(clk.future_frame(3) == 3 && clk.future_frame(99) == T - 1);
    for (int i = 0; i < 10; ++i)
        clk.step(0.02);  // 50 fps * 0.02 = 1 frame/step
    CHECK(clk.frame() == 5 && clk.finished());

    // heading alignment: ref yaw 90°, robot identity → aligned ref ≈ identity
    auto q = clk.aligned_root_quat(0);
    CHECK(std::fabs(q[0] - 1.f) < 1e-5 && std::fabs(q[3]) < 1e-5);

    // joint permutation: reversed columns
    auto mot_perm = g1::Motion::from_npz(path, {2, 1, 0});
    CHECK(mot_perm.jp(2)[0] == 22.f && mot_perm.jp(2)[2] == 20.f);

    // int64 fps (retargeted-dataset producer) must read as 50, not denormal garbage
    const std::string path_i = tmp_path("g1_sonic_selftest_motion_ifps.npz");
    std::vector<int64_t> fps_i = {50};
    cnpy::npz_save(path_i, "joint_pos", jp.data(), {(size_t)T, (size_t)J}, "w");
    cnpy::npz_save(path_i, "joint_vel", jv.data(), {(size_t)T, (size_t)J}, "a");
    cnpy::npz_save(path_i, "body_quat_w", bq.data(), {(size_t)T, (size_t)B, 4}, "a");
    cnpy::npz_save(path_i, "fps", fps_i.data(), {1}, "a");
    auto mot_i = g1::Motion::from_npz(path_i);
    CHECK(mot_i.fps == 50.f);
    CHECK(!mot_i.has_twist);  // no vel arrays → zero-filled, flagged

    // no sibling contact_matrix.npz → zeros of graph width, flagged
    CHECK(!mot.has_contact);
    CHECK(static_cast<int>(mot.bodywise_contact.size()) == T * g1::NUM_CONTACT_BODIES);
    for (int k = 0; k < g1::NUM_CONTACT_BODIES; ++k)
        CHECK(mot.contact(T - 1)[k] == 0.f);
    auto vz = mot_i.root_lin_vel_b(0);
    CHECK(vz[0] == 0.f && vz[1] == 0.f && vz[2] == 0.f);
    std::puts("ok  motion");
}

// ── g1 joint orders + IL/MJ views + wire/stand factories ────────

static void test_joint_orders()
{
    const int J = g1::NUM_JOINTS;
    // the two tables must be mutual inverses derived from the name lists
    for (int i = 0; i < J; ++i)
    {
        CHECK(g1::IL2MJ[g1::MJ2IL[i]] == i);
        CHECK(g1::MJ_JOINTS[i] == g1::IL_JOINTS[g1::MJ2IL[i]]);
    }

    // from_wire: IL rows in → MJ storage; jp_il must return the original row.
    // Same fixture at both widths — the FULL tail must not disturb the head.
    const int T = 2;
    for (int cols : {g1::WIRE_COLS_MIN, g1::WIRE_COLS_FULL})
    {
        const bool full = cols == g1::WIRE_COLS_FULL;
        std::vector<float> rows(T * cols, 0.f);
        for (int t = 0; t < T; ++t)
        {
            for (int j = 0; j < J; ++j)
            {
                rows[t * cols + j] = static_cast<float>(t * 100 + j);        // jp IL
                rows[t * cols + J + j] = -static_cast<float>(t * 100 + j);   // jv IL
            }
            rows[t * cols + 2 * J + 2] = 0.7f;   // anchor z
            rows[t * cols + 2 * J + 3] = 1.f;    // anchor quat w
            if (!full)
                continue;
            rows[t * cols + g1::WIRE_COLS_MIN + 0] = 1.f;      // anchor lin vel +x
            rows[t * cols + g1::WIRE_COLS_MIN + 5] = 2.f;      // anchor ang vel +z
            rows[t * cols + g1::WIRE_COLS_MIN + 6 + 4] = 1.f;  // left_wrist_yaw_link
        }
        auto wire = g1::Motion::from_wire(T, rows.data(), cols);
        CHECK(wire.num_frames == T && wire.num_joints == J && wire.num_bodies == 1);
        std::array<float, g1::NUM_JOINTS> il;
        wire.jp_il(1, il.data());
        for (int j = 0; j < J; ++j)
            CHECK(il[j] == static_cast<float>(100 + j));
        CHECK(wire.root_pos(1)[2] == 0.7f && wire.root_quat(1)[0] == 1.f);
        CHECK(wire.has_twist == full && wire.has_contact == full);
        // identity anchor → ref-frame twist == the wire's world twist
        CHECK(wire.root_lin_vel_b(1)[0] == (full ? 1.f : 0.f));
        CHECK(wire.root_ang_vel_b(1)[2] == (full ? 2.f : 0.f));
        CHECK(wire.contact(1)[4] == (full ? 1.f : 0.f) && wire.contact(1)[3] == 0.f);

        if (full)
        {
            // PerLoco: the same full storage independently carries twist while
            // contact is deliberately absent, rather than inferred from width.
            auto twist_only =
                g1::Motion::from_wire(T, rows.data(), cols, true, false, 60.f);
            CHECK(twist_only.has_twist && !twist_only.has_contact);
            CHECK(twist_only.fps == 60.f);
            CHECK(twist_only.root_lin_vel_b(1)[0] == 1.f);
            CHECK(twist_only.contact(1)[4] == 0.f);
        }
    }
    // a width the C++ side does not know is a hard error, never a silent zero
    bool threw = false;
    try
    {
        std::vector<float> bad(T * 70, 0.f);
        g1::Motion::from_wire(T, bad.data(), 70);
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    CHECK(threw);

    // stand factory: 1 frame, nominal pose, identity anchor, zero (true) twist
    std::vector<float> defaults(J, 0.5f);
    auto stand = g1::Motion::stand(defaults);
    CHECK(stand.num_frames == 1 && stand.jp(0)[7] == 0.5f && stand.jv(0)[7] == 0.f);
    CHECK(stand.root_quat(0)[0] == 1.f && stand.has_twist);
    // no object in the loop: zeros are the TRUE contact command, not a fallback
    CHECK(stand.has_contact && stand.contact(0)[0] == 0.f);
    CHECK(static_cast<int>(stand.bodywise_contact.size()) == g1::NUM_CONTACT_BODIES);

    // chain groups must PARTITION the MJ table — a joint in none of them would
    // silently never be prepped; a joint in two would be double-counted.
    std::vector<int> chains = g1::LEG_JOINT_INDICES;
    chains.insert(chains.end(), g1::WAIST_JOINT_INDICES.begin(), g1::WAIST_JOINT_INDICES.end());
    chains.insert(chains.end(), g1::ARM_JOINT_INDICES.begin(), g1::ARM_JOINT_INDICES.end());
    std::sort(chains.begin(), chains.end());
    CHECK(static_cast<int>(chains.size()) == g1::NUM_JOINTS);
    for (int i = 0; i < g1::NUM_JOINTS; ++i)
        CHECK(chains[i] == i);
    CHECK(g1::LEG_JOINT_INDICES.size() == 12 && g1::WAIST_JOINT_INDICES.size() == 3 &&
          g1::ARM_JOINT_INDICES.size() == 14);

    // lead_in: the prep reference. Endpoints must be EXACT — the last frame is
    // handed to the policy as the clip's frame 0, and any error there is the
    // jerk this whole path exists to remove.
    std::vector<float> to(J, 0.5f);
    to[17] = 2.0f;  // one arm joint carries the delta, as the cube clips do
    auto ramp = g1::Motion::lead_in(defaults, to, 60, 50.0f);
    CHECK(ramp.num_frames == 60 && ramp.num_joints == J && ramp.num_bodies == 1);
    for (int j = 0; j < J; ++j)
    {
        CHECK(ramp.jp(0)[j] == defaults[j]);
        CHECK(ramp.jp(59)[j] == to[j]);
    }
    // smoothstep: monotone, and zero velocity at both ends (continuous with the
    // stand it leaves and the held pose it lands on)
    for (int f = 0; f + 1 < 60; ++f)
        CHECK(ramp.jp(f + 1)[17] >= ramp.jp(f)[17]);
    // (forward difference, so jv(0) is O(1/T^2) of peak rather than exactly 0)
    CHECK(ramp.jv(0)[17] < 0.05f * ramp.jv(30)[17] && ramp.jv(59)[17] == 0.f);
    CHECK(ramp.jv(30)[17] > 0.f);
    // stand's other channels, per frame: identity root, zero twist, zero contact
    CHECK(ramp.root_quat(59)[0] == 1.f && ramp.root_lin_vel_b(30)[0] == 0.f);
    CHECK(ramp.has_contact && ramp.contact(59)[0] == 0.f);
    CHECK(static_cast<int>(ramp.bodywise_contact.size()) == 60 * g1::NUM_CONTACT_BODIES);
    CHECK(g1::Motion::lead_in(defaults, to, 1, 50.0f).num_frames == 2);  // T clamped
    std::puts("ok  joint orders + wire/stand/lead_in");
}

// ── Tokenizer row layout (the chop) ─────────────────────────────
//
// Verifies the flat-[jp|jv]-reinterpret against a hand-built reference:
// row f of (F, 2J) covers concat elements [2Jf, 2Jf+2J) — NOT [jp_f | jv_f].

static void test_tokenizer_layout()
{
    const int F = 4, J = 3;
    std::vector<float> jp(F * J), jv(F * J);
    for (int i = 0; i < F * J; ++i)
    {
        jp[i] = static_cast<float>(i);           // frames flattened
        jv[i] = 100.f + static_cast<float>(i);
    }
    std::vector<float> flat;
    flat.insert(flat.end(), jp.begin(), jp.end());
    flat.insert(flat.end(), jv.begin(), jv.end());

    // row 0 = jp frames 0,1 ; row 2 = jv frames 0,1 (F=4, J=3 → 2J=6)
    CHECK(flat[0 * 2 * J + 0] == 0.f && flat[0 * 2 * J + 5] == 5.f);
    CHECK(flat[2 * 2 * J + 0] == 100.f && flat[2 * 2 * J + 5] == 105.f);
    std::puts("ok  tokenizer layout");
}

// ── Optional: real clip contact schedule ────────────────────────
//
// Parity reference (same clip, orcs truth):
//   ContactSchedule([mf], [T]).body_object_contacts(0, CONTACT_GRAPH_BODY_NAMES)

static void clip_report(const std::string& motion_path)
{
    auto m = g1::Motion::from_npz(motion_path, g1::MJ2IL);
    CHECK(m.has_contact);
    std::printf("clip: %s\n  %d frames @ %.0f fps, %d bodies, twist=%d\n",
                motion_path.c_str(), m.num_frames, static_cast<double>(m.fps), m.num_bodies,
                static_cast<int>(m.has_twist));
    CHECK(static_cast<int>(m.bodywise_contact.size()) ==
          m.num_frames * g1::NUM_CONTACT_BODIES);
    for (float v : m.bodywise_contact)
        CHECK(v == 0.f || v == 1.f);

    for (int k = 0; k < g1::NUM_CONTACT_BODIES; ++k)
    {
        int n = 0;
        for (int f = 0; f < m.num_frames; ++f)
            n += m.contact(f)[k] != 0.f;
        std::printf("  %-26s %4d/%d frames\n", g1::CONTACT_GRAPH_BODIES[k].c_str(), n,
                    m.num_frames);
    }
    for (int f : {0, m.num_frames / 2, m.num_frames - 1})
    {
        std::printf("  contact[%4d] =", f);
        for (int k = 0; k < g1::NUM_CONTACT_BODIES; ++k)
            std::printf(" %.0f", m.contact(f)[k]);
        std::puts("");
    }

    // Wire round-trip: pack this clip the way npz_motion_publisher does, parse it
    // back. The streamed path must command exactly what the npz path commands.
    const int J = g1::NUM_JOINTS, C = g1::WIRE_COLS_FULL;
    std::vector<float> rows(static_cast<size_t>(m.num_frames) * C, 0.f);
    for (int f = 0; f < m.num_frames; ++f)
    {
        float* row = &rows[static_cast<size_t>(f) * C];
        m.jp_il(f, row);
        m.jv_il(f, row + J);
        std::memcpy(row + 2 * J, m.root_pos(f).data(), 3 * sizeof(float));
        std::memcpy(row + 2 * J + 3, m.root_quat(f).data(), 4 * sizeof(float));
        std::memcpy(row + g1::WIRE_COLS_MIN, &m.body_lin_vel_w[static_cast<size_t>(f) *
                                                               m.num_bodies * 3],
                    3 * sizeof(float));
        std::memcpy(row + g1::WIRE_COLS_MIN + 3,
                    &m.body_ang_vel_w[static_cast<size_t>(f) * m.num_bodies * 3],
                    3 * sizeof(float));
        std::memcpy(row + g1::WIRE_COLS_MIN + 6, m.contact(f),
                    g1::NUM_CONTACT_BODIES * sizeof(float));
    }
    auto w = g1::Motion::from_wire(m.num_frames, rows.data(), C);
    for (int f = 0; f < m.num_frames; ++f)
    {
        for (int j = 0; j < J; ++j)
            CHECK(w.jp(f)[j] == m.jp(f)[j] && w.jv(f)[j] == m.jv(f)[j]);
        for (int k = 0; k < g1::NUM_CONTACT_BODIES; ++k)
            CHECK(w.contact(f)[k] == m.contact(f)[k]);
        CHECK(w.root_lin_vel_b(f) == m.root_lin_vel_b(f));
        CHECK(w.root_ang_vel_b(f) == m.root_ang_vel_b(f));
    }
    std::puts("ok  clip contact schedule + wire round-trip");
}

// ── Optional: real export smoke run ─────────────────────────────

static deploy::DeployManifest check_vibe_contract(const std::string& manifest_path)
{
    auto manifest = deploy::DeployManifest::load(manifest_path);
    if (manifest.model_class == "ExtractorSonicAdapterModel")
    {
        const auto profile = vibe::profile_from_task_id(manifest.task_id);
        vibe::validate_contract(manifest, profile);
        std::printf("task:  %s [%s]\n", manifest.task_id.c_str(), profile.family.c_str());
    }
    return manifest;
}

static void smoke_run(const std::string& onnx, const std::string& manifest_path)
{
    auto manifest = check_vibe_contract(manifest_path);
    deploy::OnnxSession session(onnx);

    std::printf("model: %s\n", manifest.model_class.c_str());
    for (const auto& port : manifest.inputs)
    {
        CHECK(session.has_input(port.name));
        CHECK(static_cast<int>(session.input_dim(port.name)) == port.dim());
        std::printf("  in  %-24s %6d floats, %zu terms\n", port.name.c_str(), port.dim(),
                    port.terms.size());
        for (const auto& t : port.terms)
            std::printf("      %-28s dim=%-5d off=%-5d hist=%d\n", t.name.c_str(), t.dim,
                        t.offset, t.history);
    }

    session.run();  // zero observations
    for (const auto& out : manifest.outputs)
    {
        const float* y = session.output(out);
        const size_t n = session.output_dim(out);
        bool finite = true;
        for (size_t i = 0; i < n; ++i)
            finite &= std::isfinite(y[i]);
        CHECK(finite);
        std::printf("  out %-24s %6zu floats, finite, [0]=%.4f\n", out.c_str(), n, y[0]);
    }
    std::puts("ok  smoke run (zero obs)");
}

// ── asset_path: one rule, two roots ─────────────────────────────
// Absolute always wins; relative is searched under $VIBE_ASSET_ROOT then the
// package's installed models/; missing and ambiguous both throw.
static bool throws(const std::string& p)
{
    try
    {
        asset_path(p);
        return false;
    }
    catch (const std::exception&)
    {
        return true;
    }
}

static void test_asset_path()
{
    namespace fs = std::filesystem;
    const char* saved_asset = std::getenv("VIBE_ASSET_ROOT");
    const char* saved_ament = std::getenv("AMENT_PREFIX_PATH");
    const std::string keep_asset = saved_asset ? saved_asset : "";
    const std::string keep_ament = saved_ament ? saved_ament : "";

    const fs::path base = fs::path(tmp_path("asset_roots"));
    fs::remove_all(base);
    const fs::path assets = base / "vibe";
    const fs::path models = base / "prefix" / "share" / "cpp_control" / "models";
    fs::create_directories(assets / "data");
    fs::create_directories(models / "locomotion");
    std::ofstream(assets / "data" / "clips.npz").put('x');
    std::ofstream(models / "locomotion" / "g1.onnx").put('x');
    std::ofstream(assets / "both.onnx").put('x');
    std::ofstream(models / "both.onnx").put('x');

    setenv("VIBE_ASSET_ROOT", assets.c_str(), 1);
    setenv("AMENT_PREFIX_PATH", (base / "prefix").c_str(), 1);

    CHECK(asset_path("") == "");                             // "" means "none"
    CHECK(asset_path("/opt/x.onnx") == "/opt/x.onnx");       // absolute, never searched
    CHECK(asset_path("data/clips.npz") == (assets / "data" / "clips.npz").string());
    CHECK(asset_path("locomotion/g1.onnx") == (models / "locomotion" / "g1.onnx").string());
    CHECK(throws("data/missing.npz"));                       // under no root
    CHECK(throws("both.onnx"));                              // under two roots

    // No roots at all: an absolute path still resolves, a relative one says why.
    unsetenv("VIBE_ASSET_ROOT");
    unsetenv("AMENT_PREFIX_PATH");
    CHECK(asset_path("/opt/x.onnx") == "/opt/x.onnx");
    CHECK(throws("data/clips.npz"));

    fs::remove_all(base);
    if (keep_asset.empty()) unsetenv("VIBE_ASSET_ROOT");
    else setenv("VIBE_ASSET_ROOT", keep_asset.c_str(), 1);
    if (keep_ament.empty()) unsetenv("AMENT_PREFIX_PATH");
    else setenv("AMENT_PREFIX_PATH", keep_ament.c_str(), 1);
    std::puts("ok  asset_path (2 roots, absolute wins, missing/ambiguous throw)");
}

int main(int argc, char** argv)
{
    test_stand_yaw();
    test_history();
    test_manifest();
    test_motion();
    test_joint_orders();
    test_tokenizer_layout();
    test_asset_path();

    std::vector<std::string> pos;
    std::string motion;
    std::string contract_manifest;
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--motion" && i + 1 < argc)
            motion = argv[++i];
        else if (std::string(argv[i]) == "--manifest" && i + 1 < argc)
            contract_manifest = argv[++i];
        else
            pos.push_back(argv[i]);
    }
    if (!motion.empty())
        clip_report(motion);
    if (!contract_manifest.empty())
    {
        check_vibe_contract(contract_manifest);
        std::puts("ok  Vibe manifest contract");
    }
    if (!pos.empty())
    {
        std::string manifest = pos.size() > 1
                                   ? pos[1]
                                   : pos[0].substr(0, pos[0].rfind(".onnx")) + ".manifest.json";
        smoke_run(pos[0], manifest);
    }

    std::puts("all checks passed");
    return 0;
}
