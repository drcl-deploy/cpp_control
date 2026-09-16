// Is the observation really equivariant to a yaw of the clip?
//
//   ./build/cpp_control/difftrack_yaw_selftest <model_dir> [tolerance]
//   <model_dir> = models/tracker/difftrack/<motion>
//
// The whole stand-entry path rests on one claim, made in MotionAnchor's comment
// and relied on by every clip a robot plays from a standing start: a yaw about
// gravity is an exact symmetry of the plant, the observation is exactly
// equivariant under it, and so placing the clip on the robot's heading and then
// building the observation in the clip's frame (toReferenceFrame) puts the
// policy back on the distribution it trained on, whichever way the robot
// happens to be facing.
//
// That claim is load-bearing and it is invisible when it breaks. A clip whose
// recorded heading is 165 deg from the robot's -- g1_fight is one -- would fail
// in a way that looks exactly like a policy that cannot do the motion, and the
// only thing separating the two is this test.
//
// So: take a robot state near a clip frame, build its observation with the clip
// where it was recorded. Then turn the ENTIRE configuration, robot and clip
// together, about gravity, and build the observation the deploy path builds --
// toReferenceFrame through the anchor, then computeObs with the identity. The
// two must agree to the bit. They do, at every angle tried, which is why the
// blends in difftrack_obs.cpp are built in the clip's frame: a frame that is
// anchored on the way out is readable through either path.
//
// Needs neither the robot, the simulator, ROS nor ONNX Runtime.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "common/g1/difftrack_obs.hpp"

using cpp_control::g1::difftrack::DiffTrackObsBuilder;
using cpp_control::g1::difftrack::DiffTrackState;
using cpp_control::g1::difftrack::MotionAnchor;
using cpp_control::g1::difftrack::toReferenceFrame;

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: %s <model_dir> [tolerance]\n", argv[0]);
        return 2;
    }
    const std::string model_dir = argv[1];
    // Zero by default, and that is not optimism: the transform is a rotation by
    // -yaw applied to a state and by +yaw applied to the clip, in the same
    // double precision, so the two cancel exactly rather than nearly. A
    // tolerance here would hide the only kind of bug this test can find.
    const double tol = argc > 2 ? std::atof(argv[2]) : 0.0;

    DiffTrackObsBuilder builder;
    std::string error;
    if (!builder.load(model_dir + "/difftrack_config.json", error))
    {
        std::printf("FAIL: %s\n", error.c_str());
        return 1;
    }
    const auto& cfg = builder.config();

    std::mt19937 rng(7);
    std::uniform_real_distribution<double> jitter(-0.3, 0.3);

    // Spread over the clip, so a step that happens to sit at yaw 0 cannot carry
    // the result on its own.
    const std::vector<int> steps = {0, cfg.clipSteps / 8, cfg.clipSteps / 3,
                                    cfg.clipSteps / 2, cfg.clipSteps - 1};
    const std::vector<double> angles = {5.0, 23.0, 90.0, 165.0, -165.0, 180.0};

    double worst = 0.0;
    for (const int step : steps)
    {
        // A robot NEAR the reference, not on it: an exactly-on-reference state
        // zeroes several channels and would pass a transform that mangles them.
        MotionAnchor identity;
        Eigen::Vector3d refPos;
        Eigen::Quaterniond refQuat;
        Eigen::VectorXd refDof;
        builder.referencePose(step, identity, refPos, refQuat, refDof);

        DiffTrackState s;
        s.rootPos = refPos + Eigen::Vector3d(jitter(rng), jitter(rng), 0.02);
        s.rootQuat = refQuat * Eigen::Quaterniond(Eigen::AngleAxisd(
                                   0.05, Eigen::Vector3d(0.3, 0.5, 0.1).normalized()));
        s.rootQuat.normalize();
        s.rootLinVelWorld = Eigen::Vector3d(0.7, -0.2, 0.05);
        s.rootAngVelWorld = Eigen::Vector3d(0.1, 0.2, -0.3);
        s.dofPos = refDof;
        s.dofVel.resize(cfg.numActions);
        for (int i = 0; i < cfg.numActions; ++i)
        {
            s.dofPos[i] += 0.2 * jitter(rng);
            s.dofVel[i] = jitter(rng);
        }

        std::vector<float> obs_ref;
        builder.computeObs(s, step, identity, obs_ref);

        for (const double deg : angles)
        {
            const double yaw = deg * M_PI / 180.0;
            const Eigen::Quaterniond Rz(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

            // The same physical configuration, turned bodily about gravity.
            DiffTrackState turned = s;
            turned.rootPos = Rz * s.rootPos;
            turned.rootQuat = Rz * s.rootQuat;
            turned.rootLinVelWorld = Rz * s.rootLinVelWorld;
            turned.rootAngVelWorld = Rz * s.rootAngVelWorld;

            MotionAnchor anchor;
            anchor.yaw = yaw;
            anchor.identity = false;
            anchor.translation = Eigen::Vector2d::Zero();   // turned about the origin

            std::vector<float> obs_turned;
            builder.computeObs(toReferenceFrame(turned, anchor), step, identity, obs_turned);

            double worst_here = 0.0;
            int at = -1;
            for (size_t i = 0; i < obs_ref.size(); ++i)
            {
                const double d = std::fabs(static_cast<double>(obs_ref[i]) -
                                           static_cast<double>(obs_turned[i]));
                if (d > worst_here)
                {
                    worst_here = d;
                    at = static_cast<int>(i);
                }
            }
            worst = std::max(worst, worst_here);
            if (worst_here > tol)
                std::printf("  step %5d  yaw %+7.1f deg   max |dobs| %.3e at channel %d\n",
                            step, deg, worst_here, at);
        }
    }

    std::printf("%s: %zu steps x %zu angles, worst |dobs| %.3e (tolerance %.3e)\n",
                worst <= tol ? "PASS" : "FAIL", steps.size(), angles.size(), worst, tol);
    return worst <= tol ? 0 : 1;
}
