// difftrack self-test: does this stack reproduce diffsimrl, step for step?
//
//   ./build/cpp_control/difftrack_selftest <model_dir> [tolerance]
//   <model_dir> = models/tracker/difftrack/<motion>, the exporter's output
//
// DiffTrackObsBuilder is a C++ reimplementation of
// diffsimrl/sim2mujoco/mujoco_env.py:G1AmpMujocoEnv._get_obs, and the ONNX
// graph is a second implementation of the trained actor. Reading either side by
// side against the original proves nothing -- the failure modes are quiet ones
// (a transposed rotation column, a joint permutation, an off-by-one in the
// lookahead) that produce a plausible-looking vector and a robot that falls over
// for no visible reason.
//
// So this replays the states diffsimrl actually visited and compares against the
// observation AND the action diffsimrl actually produced. The trace comes from
//
//   python scripts/export_to_drcl_cpp_control.py <run_dir> --out <dir>
//       --trace-steps 400
//
// Run it before every deployment and after every rebuild: it is the only check
// in this pipeline that can catch a silent layout change, and it needs neither
// the robot, the simulator, nor ROS.

#include <yaml-cpp/yaml.h>

#include <onnxruntime_cxx_api.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "common/g1/difftrack_obs.hpp"

using cpp_control::g1::difftrack::DiffTrackObsBuilder;
using cpp_control::g1::difftrack::DiffTrackState;
using cpp_control::g1::difftrack::MotionAnchor;

namespace
{

// Row layout written by export_trace(); mirrored from trace.json's "layout".
constexpr int kRootPos = 3;
constexpr int kRootQuat = 4;
constexpr int kRootLinVel = 3;
constexpr int kRootAngVel = 3;
constexpr int kStep = 1;

bool read_floats(const std::string& path, std::vector<float>& out)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (f.fail())
        return false;
    const std::streamsize bytes = f.tellg();
    f.seekg(0, std::ios::beg);
    out.resize(static_cast<size_t>(bytes) / sizeof(float));
    f.read(reinterpret_cast<char*>(out.data()), bytes);
    return static_cast<bool>(f);
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: " << argv[0] << " <model_dir> [tolerance]\n"
                  << "  <model_dir> holds difftrack_config.json, motion.bin,\n"
                  << "  policy.onnx and (from --trace-steps) trace.bin/.json\n";
        return 2;
    }
    const std::filesystem::path dir(argv[1]);
    const double tol = (argc > 2) ? std::stod(argv[2]) : 2e-5;
    // Bound for a LOOPING clip's second and later table repeats, away from the
    // clip's own seam -- see the long note at the scoring loop. Not a fudge
    // factor for the first repeat, which is held to `tol` like any play-once
    // clip. Measured over 30000 control steps of g1_walk: 2.07e-3, and flat.
    const double loop_tol = 3e-3;

    DiffTrackObsBuilder builder;
    std::string error;
    if (!builder.load((dir / "difftrack_config.json").string(), error))
    {
        std::cerr << "load failed: " << error << "\n";
        return 1;
    }
    const auto& cfg = builder.config();

    std::ifstream meta_file((dir / "trace.json").string());
    if (meta_file.fail())
    {
        std::cerr << "no trace.json in " << dir << " -- re-export with --trace-steps N\n";
        return 1;
    }
    const YAML::Node meta = YAML::LoadFile((dir / "trace.json").string());
    const int num_rows = meta["num_rows"].as<int>();
    const int row_floats = meta["row_floats"].as<int>();
    const bool has_actions = meta["has_actions"].as<bool>(false);

    std::vector<float> trace;
    if (!read_floats((dir / "trace.bin").string(), trace))
    {
        std::cerr << "cannot read trace.bin\n";
        return 1;
    }
    if (trace.size() != static_cast<size_t>(num_rows) * row_floats)
    {
        std::cerr << "trace.bin size disagrees with trace.json\n";
        return 1;
    }

    const int n_act = cfg.numActions;
    const int state_floats =
        kRootPos + kRootQuat + kRootLinVel + kRootAngVel + 2 * n_act + kStep;
    const int want = state_floats + cfg.numObs + (has_actions ? n_act : 0);
    if (row_floats != want)
    {
        std::cerr << "trace row is " << row_floats << " floats but the config implies "
                  << want << "\n";
        return 1;
    }

    // ── the policy, as the node will run it ──
    Ort::Env ort_env(ORT_LOGGING_LEVEL_ERROR, "difftrack_selftest");
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    Ort::Session session(ort_env, (dir / cfg.modelName).string().c_str(), opts);
    {
        const auto shape =
            session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        int64_t in_dim = 1;
        for (size_t i = 1; i < shape.size(); i++)
            in_dim *= shape[i];
        if (in_dim != cfg.numObs)
        {
            std::cerr << "policy.onnx takes " << in_dim << " inputs but the config "
                      << "assembles " << cfg.numObs << "\n";
            return 1;
        }
    }
    const auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const char* in_names[] = {"obs"};
    const char* out_names[] = {"actions"};

    std::printf("model dir     %s\n"
                "run           %s%s%s\n"
                "motion        %s (%d steps%s)\n"
                "obs           %d = %d char + %zu x %d tar\n"
                "trace         %d rows%s\n"
                "tolerance     %g\n\n",
                dir.c_str(), cfg.sourceRun.c_str(),
                cfg.variant.empty() ? "" : "  variant=", cfg.variant.c_str(),
                cfg.motionFile.c_str(), cfg.clipSteps, builder.loops() ? ", LOOPING" : "",
                cfg.numObs, cfg.charObsDim, cfg.tarObsSteps.size(), cfg.tarFeatDim,
                num_rows, has_actions ? " (with actions)" : "", tol);

    DiffTrackState state;
    state.dofPos.resize(n_act);
    state.dofVel.resize(n_act);
    std::vector<float> obs;
    std::vector<float> onnx_out(static_cast<size_t>(n_act), 0.0f);

    // Row 0 is scored separately, and only rows >= 1 are held to the tolerance.
    //
    // diffsimrl's reference-state initialisation writes the motion library's
    // root quaternion straight into qpos, and those frames are off unit by up to
    // 3e-5. MuJoCo normalises internally when it computes body poses, but
    // _quat_to_tan_norm does not -- so on that one frame diffsimrl's own
    // root_rot6 and key_pos disagree about the rotation, and no implementation
    // can match both. This builder normalises, which is the consistent choice.
    // From step 1 on the integrator has renormalised qpos and the question
    // disappears (residual 3e-8, i.e. float32 storage).
    double worst = 0.0, worst_rsi = 0.0;
    int worst_row = -1, worst_idx = -1;

    // Rows past the clip table's first repeat are scored separately, and only
    // for a LOOPING clip -- a play-once one never reaches them.
    //
    // Inside the first repeat the builder looks up the very frames the exporter
    // computed from diffsimrl's own arithmetic, so it is bit-identical. Past it,
    // the builder reads step k out of table entry k % clipSteps while diffsimrl
    // queried the motion library at the absolute time k*dt. Those are the same
    // instant, but not the same float32 computation, and the reference has two
    // kinds of place where that shows.
    //
    // 1. THE CLIP'S SEAM, at (k + lookahead) % clipSteps == 0. The table holds a
    //    whole number of cycles, so its repeats land exactly on t = m*L, where
    //    the reference jumps from the clip's last frame to its first -- g1_walk
    //    does not close perfectly, and that jump is 4.6e-2 in a 6D rotation
    //    channel. Which side of it a query lands on is decided by float32
    //    cancellation in `phase - floor(phase)`. Both answers are that instant;
    //    the reference is discontinuous there and training crossed the same
    //    discontinuity once per cycle. Reported, not scored.
    // 2. EXACT SOURCE-FRAME LANDINGS, everywhere else. dt = 0.02s against a
    //    120 fps clip puts every fifth control step exactly twelve source frames
    //    on, and the same cancellation decides whether that reads as (frame j,
    //    blend 0) or (frame j-1, blend 0.99995). Those would be identical but
    //    for MimicKit's slerp, which returns the MIDPOINT of its two frames
    //    whenever they are under ~0.1 deg apart whatever the blend. Measured
    //    over 30000 steps of g1_walk: 2.07e-3, flat, on nearly-stationary
    //    joints. Training was noised by 1e-2 rad on joint positions.
    //
    // No table-based scheme avoids either: a clip that loops forever cannot
    // store every absolute time. What the bound below IS good for is catching a
    // wrong wrap -- a missing per-repeat root offset shows up as metres.
    const bool looping = builder.loops();
    double worst_loop = 0.0;
    int worst_loop_row = -1, worst_loop_idx = -1;
    int loop_rows = 0;
    double worst_seam = 0.0;
    int seam_instants = 0;
    double worst_action = 0.0, action_rms = 0.0;
    int worst_action_row = -1;
    long action_n = 0;

    // Per-block worst error, so a failure points at the block that broke rather
    // than at one index in an 849-vector.
    struct Block
    {
        std::string name;
        int begin, end;
        double worst;
    };
    int p = 0;
    std::vector<Block> blocks;
    if (cfg.rootHeightObs)
    {
        blocks.push_back({"root_height", p, p + 1, 0.0});
        p += 1;
    }
    blocks.push_back({"root_rot6", p, p + 6, 0.0});
    p += 6;
    blocks.push_back({"root_lin_vel", p, p + 3, 0.0});
    p += 3;
    blocks.push_back({"root_ang_vel", p, p + 3, 0.0});
    p += 3;
    blocks.push_back({"joint_rot6", p, p + 6 * (cfg.numBodies - 1), 0.0});
    p += 6 * (cfg.numBodies - 1);
    blocks.push_back({"dof_vel", p, p + n_act, 0.0});
    p += n_act;
    blocks.push_back({"key_pos", p, cfg.charObsDim, 0.0});
    p = cfg.charObsDim;
    for (size_t s = 0; s < cfg.tarObsSteps.size(); s++)
    {
        blocks.push_back({"tar_obs[" + std::to_string(cfg.tarObsSteps[s]) + "]", p,
                          p + cfg.tarFeatDim, 0.0});
        p += cfg.tarFeatDim;
    }

    for (int r = 0; r < num_rows; r++)
    {
        const float* row = trace.data() + static_cast<size_t>(r) * row_floats;
        int i = 0;
        state.rootPos = Eigen::Vector3d(row[i], row[i + 1], row[i + 2]);
        i += kRootPos;
        state.rootQuat = Eigen::Quaterniond(row[i], row[i + 1], row[i + 2], row[i + 3]);
        i += kRootQuat;
        state.rootLinVelWorld = Eigen::Vector3d(row[i], row[i + 1], row[i + 2]);
        i += kRootLinVel;
        state.rootAngVelWorld = Eigen::Vector3d(row[i], row[i + 1], row[i + 2]);
        i += kRootAngVel;
        for (int j = 0; j < n_act; j++)
            state.dofPos[j] = row[i + j];
        i += n_act;
        for (int j = 0; j < n_act; j++)
            state.dofVel[j] = row[i + j];
        i += n_act;
        const int episode_step = static_cast<int>(row[i]);
        i += kStep;
        const float* expected = row + i;
        const float* expected_act = has_actions ? row + i + cfg.numObs : nullptr;

        // Identity anchor: the trace was recorded with the robot teleported onto
        // the clip, which is where the clip already is. This is also the case
        // that has to be exact -- with the anchor on, the reference is somewhere
        // the recorded rollout never went, so there is nothing to compare to.
        builder.computeObs(state, episode_step, MotionAnchor{}, obs);

        if (expected_act)
        {
            // Feed the ROLLOUT's observation, not the one just computed: this
            // half is scoring the ONNX graph against torch, and mixing in the
            // observation's own residual would hide which of the two moved.
            std::vector<float> in(expected, expected + cfg.numObs);
            std::vector<int64_t> in_shape = {1, cfg.numObs};
            std::vector<int64_t> out_shape = {1, n_act};
            Ort::Value in_t = Ort::Value::CreateTensor<float>(
                mem, in.data(), in.size(), in_shape.data(), in_shape.size());
            Ort::Value out_t = Ort::Value::CreateTensor<float>(
                mem, onnx_out.data(), onnx_out.size(), out_shape.data(), out_shape.size());
            Ort::RunOptions run_options;
            session.Run(run_options, in_names, &in_t, 1, out_names, &out_t, 1);
            for (int j = 0; j < n_act; j++)
            {
                const double err = std::fabs(onnx_out[j] - expected_act[j]);
                if (err > worst_action)
                {
                    worst_action = err;
                    worst_action_row = r;
                }
                action_rms += static_cast<double>(expected_act[j]) * expected_act[j];
                action_n++;
            }
        }

        const bool later_repeat = looping && episode_step >= cfg.clipSteps;
        if (later_repeat)
            loop_rows++;
        // Which tar_obs slots of this row sit on the clip's seam (case 1 above).
        // char_obs is the robot's own state and is never affected.
        std::vector<bool> slot_on_seam(cfg.tarObsSteps.size(), false);
        if (later_repeat)
        {
            for (size_t sl = 0; sl < cfg.tarObsSteps.size(); sl++)
            {
                slot_on_seam[sl] = ((episode_step + cfg.tarObsSteps[sl]) % cfg.clipSteps) == 0;
                if (slot_on_seam[sl])
                    seam_instants++;
            }
        }
        auto on_seam = [&](int j) {
            if (j < cfg.charObsDim)
                return false;
            const size_t sl = static_cast<size_t>((j - cfg.charObsDim) / cfg.tarFeatDim);
            return sl < slot_on_seam.size() && slot_on_seam[sl];
        };

        for (int j = 0; j < cfg.numObs; j++)
        {
            const double err = std::fabs(static_cast<double>(obs[j]) - expected[j]);
            if (r == 0)
            {
                worst_rsi = std::max(worst_rsi, err);
                continue;
            }
            if (later_repeat)
            {
                if (on_seam(j))
                {
                    worst_seam = std::max(worst_seam, err);
                }
                else if (err > worst_loop)
                {
                    worst_loop = err;
                    worst_loop_row = r;
                    worst_loop_idx = j;
                }
                continue;
            }
            if (err > worst)
            {
                worst = err;
                worst_row = r;
                worst_idx = j;
            }
            for (auto& b : blocks)
            {
                if (j >= b.begin && j < b.end)
                {
                    b.worst = std::max(b.worst, err);
                    break;
                }
            }
        }
    }

    std::printf("%-16s %12s   (steady state, rows 1..%d%s)\n", "block", "max |err|",
                num_rows - 1, looping ? ", loop repeats scored separately" : "");
    for (const auto& b : blocks)
        std::printf("%-16s %12.3e\n", b.name.c_str(), b.worst);

    std::printf("\nworst steady state  %.3e  (row %d, index %d)\n", worst, worst_row,
                worst_idx);
    if (looping)
    {
        std::printf("worst loop repeat   %.3e  (row %d, index %d; %d rows past the "
                    "clip table's first repeat, bound %g -- MimicKit's slerp at an "
                    "exact source-frame landing, see the note in this file)\n",
                    worst_loop, worst_loop_row, worst_loop_idx, loop_rows, loop_tol);
        std::printf("clip seam           %.3e  (%d lookahead instants on t = m*L; "
                    "reported, not scored -- the clip does not close, and which side "
                    "of the jump a query lands on is float32 rounding)\n",
                    worst_seam, seam_instants);
    }
    std::printf("worst RSI frame     %.3e  (row 0; excluded -- diffsimrl's RSI "
                "quaternion is not unit, see the note in this file)\n",
                worst_rsi);
    if (action_n)
        std::printf("worst action        %.3e  (row %d, against an action RMS of "
                    "%.3f -- policy.onnx vs the torch module)\n",
                    worst_action, worst_action_row, std::sqrt(action_rms / action_n));

    bool ok = true;
    if (!(worst <= tol))
    {
        std::printf("\nFAIL: observation does not match diffsimrl within %g\n", tol);
        ok = false;
    }
    if (looping && !(worst_loop <= loop_tol))
    {
        std::printf("\nFAIL: past the clip table's first repeat the observation is off "
                    "by %g, more than the %g the frame-boundary artifact can account "
                    "for -- the wrap arithmetic is wrong, not the slerp\n",
                    worst_loop, loop_tol);
        ok = false;
    }
    // The exporter measures the graph against torch at ~3e-6 in float32; a
    // deploy-side session that disagrees by more than 1e-4 is a different graph
    // or a different input, not numerics.
    if (action_n && !(worst_action <= 1e-4))
    {
        std::printf("\nFAIL: policy.onnx disagrees with the recorded actions by %g\n",
                    worst_action);
        ok = false;
    }
    std::printf(ok ? "\nPASS\n" : "\n");
    return ok ? 0 : 1;
}
