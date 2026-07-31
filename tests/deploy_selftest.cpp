// Self-test for the deploy infra (manifest / history / motion clip / tokenizer
// layout), plus an optional smoke run of a real export:
//
//   deploy_selftest                                  # unit checks only
//   deploy_selftest policy.onnx [policy.manifest.json]  # + load, zero-obs run
//
// Plain asserts, no gtest — run it, exit 0 means pass.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "cnpy/cnpy.h"
#include "common/deploy_manifest.hpp"
#include "common/math_utils.hpp"
#include "common/motion_clip.hpp"
#include "common/obs_terms.hpp"
#include "common/onnx_session.hpp"

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

// ── MotionClip + playback: clamp, heading alignment ─────────────

static void test_motion()
{
    const std::string path = tmp_path("g1_sonic_selftest_motion.npz");
    const int T = 6, J = 3, B = 2;
    std::vector<float> jp(T * J), jv(T * J), bq(T * B * 4, 0.f);
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
    }
    std::vector<double> fps = {50.0};
    cnpy::npz_save(path, "joint_pos", jp.data(), {(size_t)T, (size_t)J}, "w");
    cnpy::npz_save(path, "joint_vel", jv.data(), {(size_t)T, (size_t)J}, "a");
    cnpy::npz_save(path, "body_quat_w", bq.data(), {(size_t)T, (size_t)B, 4}, "a");
    cnpy::npz_save(path, "fps", fps.data(), {1}, "a");

    auto clip = MotionClip::load(path);
    CHECK(clip.num_frames == T && clip.num_joints == J && clip.num_bodies == B);
    CHECK(clip.fps == 50.f);
    CHECK(clip.jp(2)[1] == 21.f && clip.jv(1)[0] == -10.f);

    MotionPlayback pb(clip, 0);
    std::array<float, 4> robot_identity = {1.f, 0.f, 0.f, 0.f};
    pb.start(robot_identity);
    CHECK(pb.frame() == 0);
    // future clamp: mirror FutureMotionCommand.future_frames
    CHECK(pb.future_frame(3) == 3 && pb.future_frame(99) == T - 1);
    for (int i = 0; i < 10; ++i)
        pb.step(0.02);  // 50 fps * 0.02 = 1 frame/step
    CHECK(pb.frame() == 5 && pb.finished());

    // heading alignment: ref yaw 90°, robot identity → aligned ref ≈ identity
    auto q = pb.aligned_anchor_quat(0);
    CHECK(std::fabs(q[0] - 1.f) < 1e-5 && std::fabs(q[3]) < 1e-5);

    // joint permutation: reversed columns
    auto clip_perm = MotionClip::load(path, {2, 1, 0});
    CHECK(clip_perm.jp(2)[0] == 22.f && clip_perm.jp(2)[2] == 20.f);
    std::puts("ok  motion");
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

// ── Optional: real export smoke run ─────────────────────────────

static void smoke_run(const std::string& onnx, const std::string& manifest_path)
{
    auto manifest = deploy::DeployManifest::load(manifest_path);
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

int main(int argc, char** argv)
{
    test_history();
    test_manifest();
    test_motion();
    test_tokenizer_layout();

    if (argc > 1)
    {
        std::string onnx = argv[1];
        std::string manifest =
            argc > 2 ? argv[2] : onnx.substr(0, onnx.rfind(".onnx")) + ".manifest.json";
        smoke_run(onnx, manifest);
    }

    std::puts("all checks passed");
    return 0;
}
