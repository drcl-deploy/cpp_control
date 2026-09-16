// Does WorldStateGuard keep an estimator glitch out without touching real motion?
//
//   ./build/cpp_control/difftrack_guard_selftest
//
// Synthetic, and on purpose: the two properties it holds the guard to are
// statements about motion, not about any one clip.
//
//   1. GENUINE MOTION PASSES UNTOUCHED. A reference running at 3.75 m/s -- the
//      fastest robot speed any shipped clip reaches on ground truth
//      (g1_run_bundle) -- with the robot weaving 0.3 m around it, fast enough
//      that its motion beyond the reference reaches 0.42 m per 0.2 s: above the
//      0.39 m that g1_run_bundle, the worst shipped clip, measures on ground
//      truth. No rejection, no correction, no velocity change.
//   2. A GLITCH IS HELD OUT. The same run with the estimate teleporting 2.0 m
//      in 0.1 s and a 2 m/s velocity error on top: at most maxUnexplainedM
//      leaks into the corrected position, the rest becomes the correction, and
//      the velocity stays within velDeviationMps of the reference while the
//      correction is fresh.
//   3. A SNAP-BACK UNDOES IT, gate=false takes no rejections, reset() forgets.
//
// Needs neither the robot, the simulator, ROS nor ONNX Runtime.

#include <cmath>
#include <cstdio>
#include <string>

#include <Eigen/Dense>

#include "common/g1/difftrack_obs.hpp"

using cpp_control::g1::difftrack::WorldStateGuard;
using cpp_control::g1::difftrack::WorldStateGuardConfig;

namespace {

int failures = 0;

void check(bool ok, const std::string& what, double value, double limit)
{
    std::printf("  %-4s %-58s %9.4f  (limit %.4f)\n", ok ? "ok" : "FAIL", what.c_str(), value, limit);
    if (!ok)
        ++failures;
}

struct Truth
{
    Eigen::Vector3d pos, vel, ref, ref_vel;
};

// The robot 0.3 m around a reference running along a curve at `speed`.
Truth truth_at(int k, double dt, double speed)
{
    const double t = k * dt;
    const double w = 0.4;   // rad/s: a gentle turn, so both axes move
    Truth s;
    s.ref = Eigen::Vector3d(speed / w * std::sin(w * t), speed / w * (1.0 - std::cos(w * t)), 0.78);
    s.ref_vel = Eigen::Vector3d(speed * std::cos(w * t), speed * std::sin(w * t), 0.0);
    // 1.0 Hz in x and 0.5 Hz in y at 0.3 m: the weave's own speed peaks at
    // 2.1 m/s, i.e. 0.42 m per 0.2 s beyond the reference.
    const double fx = 2.0 * M_PI * 1.0, fy = 2.0 * M_PI * 0.5;
    const Eigen::Vector3d weave(0.3 * std::sin(fx * t), 0.3 * std::cos(fy * t), 0.0);
    const Eigen::Vector3d weave_vel(0.3 * fx * std::cos(fx * t), -0.3 * fy * std::sin(fy * t), 0.0);
    s.pos = s.ref + weave;
    s.vel = s.ref_vel + weave_vel;
    return s;
}

// The reference velocity exactly as the guard sees it: the reference's own step
// over one control period, not the analytic derivative.
Eigen::Vector3d ref_step_vel(int k, double dt, double speed)
{
    if (k == 0)
        return Eigen::Vector3d::Zero();
    return (truth_at(k, dt, speed).ref - truth_at(k - 1, dt, speed).ref) / dt;
}

}  // namespace

int main()
{
    WorldStateGuardConfig cfg;
    const double dt = cfg.dt;
    const double speed = 3.75;

    // ── 1. genuine motion ──
    std::printf("genuine motion: %.2f m/s reference, robot weaving 0.3 m around it\n", speed);
    {
        WorldStateGuard g;
        g.configure(cfg);
        double worst_pos = 0.0, worst_vel = 0.0;
        for (int k = 0; k < 750; ++k)
        {
            Truth s = truth_at(k, dt, speed);
            Eigen::Vector3d p = s.pos, v = s.vel;
            g.apply(p, v, s.ref);
            worst_pos = std::max(worst_pos, (p - s.pos).norm());
            worst_vel = std::max(worst_vel, (v - s.vel).norm());
        }
        check(g.rejectTicks() == 0, "no rejected tick", g.rejectTicks(), 0);
        check(worst_pos == 0.0, "position untouched (m)", worst_pos, 0.0);
        check(worst_vel == 0.0, "velocity untouched (m/s)", worst_vel, 0.0);
    }

    // ── 2. a glitch ──
    std::printf("glitch: 2.0 m in 0.1 s at t=4 s, 2 m/s velocity error for 0.4 s, error persists\n");
    WorldStateGuard g;
    g.configure(cfg);
    const Eigen::Vector3d jump_dir = Eigen::Vector3d(0.6, -0.8, 0.0);
    const int k0 = 200, ramp = 5, vel_ticks = 20;
    double leak_max = 0.0, vel_dev_max = 0.0;
    for (int k = 0; k < 500; ++k)
    {
        Truth s = truth_at(k, dt, speed);
        double frac = std::min(1.0, std::max(0.0, (k - k0 + 1) / static_cast<double>(ramp)));
        Eigen::Vector3d p = s.pos + 2.0 * frac * jump_dir;
        Eigen::Vector3d v = s.vel;
        if (k >= k0 && k < k0 + vel_ticks)
            v += 2.0 * jump_dir;
        g.apply(p, v, s.ref);
        if (k >= k0)
            leak_max = std::max(leak_max, (p - s.pos).head<2>().norm());
        if (k >= k0 && k < k0 + vel_ticks)
            vel_dev_max = std::max(vel_dev_max, (v - ref_step_vel(k, dt, speed)).head<2>().norm());
    }
    check(g.rejectTicks() > 0, "the jump was rejected (ticks)", g.rejectTicks(), 1);
    check(leak_max <= cfg.maxUnexplainedM + 1e-9, "position error let through (m)", leak_max, cfg.maxUnexplainedM);
    check(g.offset().norm() >= 2.0 - cfg.maxUnexplainedM - 1e-9, "held-out correction (m)", g.offset().norm(),
          2.0 - cfg.maxUnexplainedM);
    check(vel_dev_max <= cfg.velDeviationMps + 1e-9, "velocity deviation while fresh (m/s)", vel_dev_max,
          cfg.velDeviationMps);

    // ── 3. snap-back, gate, reset ──
    std::printf("snap-back, gate=false, reset\n");
    {
        const double before = g.offset().norm();
        double worst = 0.0;
        for (int k = 500; k < 700; ++k)
        {
            Truth s = truth_at(k, dt, speed);
            double frac = std::min(1.0, std::max(0.0, (k - 600 + 1) / static_cast<double>(ramp)));
            Eigen::Vector3d p = s.pos + 2.0 * (1.0 - frac) * jump_dir;   // the estimate snaps back
            Eigen::Vector3d v = s.vel;
            g.apply(p, v, s.ref);
            if (k >= 650)
                worst = std::max(worst, (p - s.pos).head<2>().norm());
        }
        check(worst <= cfg.maxUnexplainedM + 1e-9, "error after the estimate snaps back (m)", worst,
              cfg.maxUnexplainedM);
        check(g.offset().norm() < before, "correction shrank on the snap-back (m)", g.offset().norm(), before);

        WorldStateGuard h;
        h.configure(cfg);
        Truth s0 = truth_at(0, dt, speed);
        Eigen::Vector3d p = s0.pos, v = s0.vel;
        h.apply(p, v, s0.ref, false);
        Eigen::Vector3d p1 = s0.pos + Eigen::Vector3d(3.0, 0.0, 0.0), v1 = s0.vel;
        h.apply(p1, v1, s0.ref, false);
        check(h.rejectTicks() == 0, "gate=false takes no rejection", h.rejectTicks(), 0);

        g.reset();
        check(g.offset().norm() == 0.0 && g.rejectTicks() == 0, "reset() clears the correction", g.offset().norm(),
              0.0);
    }

    std::printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
