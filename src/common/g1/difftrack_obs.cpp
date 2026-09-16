#include "common/g1/difftrack_obs.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace cpp_control::g1::difftrack {

    namespace {

        // ------------------------------------------------------------------
        // Quaternion helpers
        // ------------------------------------------------------------------
        //
        // Written out rather than delegated to Eigen because the encoding below
        // is NOT Eigen's. tanNorm takes the first and THIRD columns of the
        // rotation matrix (forward + up), matching MimicKit's quat_to_tan_norm;
        // taking Eigen's columns 0 and 1 would silently produce a different
        // 6-vector and a policy fed nonsense.

        /** Columns 0 and 2 of the rotation matrix of @p q, as one 6-vector. */
        inline void tanNorm(const Eigen::Quaterniond &q, double *out) {
            const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
            out[0] = 1.0 - 2.0 * (y * y + z * z);
            out[1] = 2.0 * (x * y + w * z);
            out[2] = 2.0 * (x * z - w * y);
            out[3] = 2.0 * (x * z + w * y);
            out[4] = 2.0 * (y * z - w * x);
            out[5] = 1.0 - 2.0 * (x * x + y * y);
        }

        /** Rotation of @p angle about @p axis, matching MimicKit axis_angle_to_quat. */
        inline Eigen::Quaterniond axisAngle(const Eigen::Vector3d &axis, double angle) {
            const double n = axis.norm();
            if (n < 1e-12) {
                return Eigen::Quaterniond::Identity();
            }
            const double half = 0.5 * angle;
            const Eigen::Vector3d u = (axis / n) * std::sin(half);
            Eigen::Quaterniond q(std::cos(half), u.x(), u.y(), u.z());
            q.normalize();  // MimicKit's quat_unit
            return q;
        }

        /** Yaw of a quaternion in the Z-up convention MimicKit uses. */
        inline double headingYaw(const Eigen::Quaterniond &q) {
            const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
            return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        }

        // ------------------------------------------------------------------
        // Rotation-vector exp/log, for the lead-in's orientation blend
        // ------------------------------------------------------------------

        /** Rotation vector (axis * angle) of @p q, taking the shorter arc. */
        inline Eigen::Vector3d quatLog(const Eigen::Quaterniond &qIn) {
            Eigen::Quaterniond q = qIn;
            q.normalize();
            if (q.w() < 0.0) {
                // -q is the same rotation; picking it keeps the angle under pi, so
                // the blend takes the short way round instead of the long way.
                q.coeffs() *= -1.0;
            }
            const Eigen::Vector3d v = q.vec();
            const double n = v.norm();
            if (n < 1e-12) {
                return Eigen::Vector3d::Zero();
            }
            return v * (2.0 * std::atan2(n, q.w()) / n);
        }

        /** Inverse of quatLog. */
        inline Eigen::Quaterniond quatExp(const Eigen::Vector3d &r) {
            const double angle = r.norm();
            if (angle < 1e-12) {
                return Eigen::Quaterniond::Identity();
            }
            return Eigen::Quaterniond(Eigen::AngleAxisd(angle, r / angle));
        }

        /**
         * Cubic Hermite on [0, T]: value @p a with derivative @p da at t=0, value
         * @p b with derivative @p db at t=1, evaluated at normalised time @p s.
         *
         * Templated over the value type so the root position, the joint vector and
         * the orientation's rotation vector all share one blend -- they have to,
         * or the reference would be internally inconsistent partway through.
         */
        template <typename T>
        inline T hermite(const T &a, const T &da, const T &b, const T &db,
                         double s, double duration) {
            const double s2 = s * s, s3 = s2 * s;
            const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
            const double h10 = s3 - 2.0 * s2 + s;
            const double h01 = -2.0 * s3 + 3.0 * s2;
            const double h11 = s3 - s2;
            return h00 * a + (h10 * duration) * da + h01 * b + (h11 * duration) * db;
        }

        // ------------------------------------------------------------------
        // JSON reading, through yaml-cpp
        // ------------------------------------------------------------------
        //
        // difftrack_config.json is JSON, and JSON is a subset of YAML, so
        // yaml-cpp reads it as-is -- which is what deploy_manifest.cpp does with
        // the vibe manifest, and keeps this package's dependency list where it
        // was. The only wrinkle is that a missing key yields an undefined node
        // rather than throwing, so `req()` puts the throw back: an absent field
        // has to be an error, not a silent zero fed to the policy.

        YAML::Node req(const YAML::Node &parent, const char *key) {
            const YAML::Node n = parent[key];
            if (!n) {
                throw std::runtime_error(std::string("missing key '") + key + "'");
            }
            return n;
        }

        Eigen::VectorXd toVector(const YAML::Node &j) {
            Eigen::VectorXd v(static_cast<int>(j.size()));
            for (size_t i = 0; i < j.size(); i++) {
                v[static_cast<int>(i)] = j[i].as<double>();
            }
            return v;
        }

        std::vector<Eigen::Vector3d> toVec3List(const YAML::Node &j) {
            std::vector<Eigen::Vector3d> out;
            out.reserve(j.size());
            for (const auto &row : j) {
                out.emplace_back(row[0].as<double>(), row[1].as<double>(), row[2].as<double>());
            }
            return out;
        }

        std::vector<std::string> toStringList(const YAML::Node &j) {
            std::vector<std::string> out;
            out.reserve(j.size());
            for (const auto &v : j) {
                out.push_back(v.as<std::string>());
            }
            return out;
        }

        std::vector<int> toIntList(const YAML::Node &j) {
            std::vector<int> out;
            out.reserve(j.size());
            for (const auto &v : j) {
                out.push_back(v.as<int>());
            }
            return out;
        }

    }  // namespace

    // ----------------------------------------------------------------------
    // Loading
    // ----------------------------------------------------------------------

    bool DiffTrackObsBuilder::load(const std::string &configPath, std::string &error) {
        if (!std::filesystem::exists(configPath)) {
            error = "cannot open " + configPath;
            return false;
        }

        YAML::Node conf;
        try {
            conf = YAML::LoadFile(configPath);
        } catch (const std::exception &e) {
            error = std::string("cannot parse ") + configPath + ": " + e.what();
            return false;
        }

        try {
            cfg_.sourceRun = conf["source_run"].as<std::string>(std::string());
            cfg_.motionFile = conf["motion_file"].as<std::string>(std::string());
            cfg_.trainRun = conf["train_run"].as<std::string>(std::string());
            cfg_.variant = conf["variant"].as<std::string>(std::string());
            cfg_.trainAlg = conf["train_alg"].as<std::string>(std::string("forl_shac"));
            cfg_.checkpointIter = conf["checkpoint_iter"].as<int>(-1);
            cfg_.modelName = req(conf, "model_name").as<std::string>();
            cfg_.motionName = req(conf, "motion_name").as<std::string>();

            cfg_.numObs = req(conf, "num_obs").as<int>();
            cfg_.numActions = req(conf, "num_actions").as<int>();
            cfg_.controlDt = req(conf, "control_dt").as<double>();

            cfg_.policyJointNames = toStringList(req(conf, "policy_joint_names"));
            cfg_.defaultAngles = toVector(req(conf, "default_angles"));
            cfg_.actionScale = req(conf, "action_scale").as<double>();
            cfg_.actionClipping = req(conf, "action_clipping").as<bool>();
            cfg_.actionClipRange = req(conf, "action_clip_range").as<double>();

            cfg_.jointStiffness = toVector(req(conf, "joint_stiffness"));
            cfg_.jointDamping = toVector(req(conf, "joint_damping"));
            cfg_.jointTorqueLimit = toVector(req(conf, "joint_torque_limit"));
            cfg_.terminationHeight = req(conf, "plant")["termination_height"].as<double>(0.0);

            const YAML::Node obs = req(conf, "obs");
            cfg_.globalObs = req(obs, "global_obs").as<bool>();
            cfg_.rootHeightObs = req(obs, "root_height_obs").as<bool>();
            cfg_.charObsDim = req(obs, "char_obs_dim").as<int>();
            cfg_.tarFeatDim = req(obs, "tar_feat_dim").as<int>();
            cfg_.clipSteps = req(obs, "clip_steps").as<int>();
            cfg_.motionLengthS = req(obs, "motion_length_s").as<double>();
            // Absent from clips exported before looping was supported, which are
            // all play-once -- so the defaults are the old behaviour exactly.
            cfg_.loopWrap = obs["loop_wrap"].as<bool>(false);
            cfg_.wrapCycles = obs["wrap_cycles"].as<int>(1);
            if (obs["wrap_delta"]) {
                const auto wd = toVector(obs["wrap_delta"]);
                cfg_.wrapDelta = Eigen::Vector3d(wd[0], wd[1], wd[2]);
            }
            cfg_.numBodies = req(obs, "num_bodies").as<int>();
            cfg_.tarObsSteps = toIntList(req(obs, "tar_obs_steps"));
            cfg_.keyBodyIds = toIntList(req(obs, "key_body_ids"));
            cfg_.keyBodyOffsets = toVec3List(req(obs, "key_body_offsets"));
            cfg_.dofAxes = toVec3List(req(obs, "dof_axes"));
            cfg_.dofIndex = toIntList(req(obs, "dof_index"));
            cfg_.numKeyPoints = static_cast<int>(cfg_.keyBodyIds.size());

            const YAML::Node fk = req(conf, "fk");
            cfg_.bodyNames = toStringList(req(fk, "body_names"));
            cfg_.parent = toIntList(req(fk, "parent"));
            cfg_.bodyPos = toVec3List(req(fk, "body_pos"));
            cfg_.jointAxis = toVec3List(req(fk, "joint_axis"));
            cfg_.jointDof = toIntList(req(fk, "joint_dof"));
            cfg_.fixedOffset = toVec3List(req(fk, "fixed_offset"));
            cfg_.bodyQuat.clear();
            for (const auto &q : req(fk, "body_quat")) {
                cfg_.bodyQuat.emplace_back(q[0].as<double>(), q[1].as<double>(),
                                           q[2].as<double>(), q[3].as<double>());
            }

            const YAML::Node rsi = req(conf, "rsi");
            const auto rp = toVector(req(rsi, "root_pos"));
            const auto rq = toVector(req(rsi, "root_quat_wxyz"));
            cfg_.rsi.rootPos = Eigen::Vector3d(rp[0], rp[1], rp[2]);
            cfg_.rsi.rootQuat = Eigen::Quaterniond(rq[0], rq[1], rq[2], rq[3]);
            const auto rv = toVector(req(rsi, "root_lin_vel"));
            const auto ra = toVector(req(rsi, "root_ang_vel_world"));
            cfg_.rsi.rootLinVel = Eigen::Vector3d(rv[0], rv[1], rv[2]);
            cfg_.rsi.rootAngVelWorld = Eigen::Vector3d(ra[0], ra[1], ra[2]);
            cfg_.rsi.dofPos = toVector(req(rsi, "dof_pos"));
            cfg_.rsi.dofVel = toVector(req(rsi, "dof_vel"));
        } catch (const std::exception &e) {
            error = std::string("malformed config ") + configPath + ": " + e.what();
            return false;
        }

        // -- consistency: the observation the exporter described must be the one
        //    the network expects, and the tables must be the sizes it declared.
        const int expected = cfg_.charObsDim +
                             static_cast<int>(cfg_.tarObsSteps.size()) * cfg_.tarFeatDim;
        if (expected != cfg_.numObs) {
            error = "config declares num_obs=" + std::to_string(cfg_.numObs) +
                    " but its layout assembles " + std::to_string(expected);
            return false;
        }
        if (!cfg_.globalObs) {
            error = "global_obs=false is not supported: tar_obs would depend on the "
                    "agent's heading, which the exported table does not carry";
            return false;
        }
        // tar slot = root_pos(3 with height, else 2) + root_rot6(6)
        //            + joint_rot6(6*(nb-1)) + key(3K)
        const int tarExpected = (cfg_.rootHeightObs ? 3 : 2) + 6 +
                                6 * (cfg_.numBodies - 1) + 3 * cfg_.numKeyPoints;
        if (tarExpected != cfg_.tarFeatDim) {
            error = "tar_feat_dim=" + std::to_string(cfg_.tarFeatDim) +
                    " but the declared layout assembles " + std::to_string(tarExpected);
            return false;
        }
        const size_t nb = static_cast<size_t>(cfg_.numBodies);
        if (cfg_.bodyNames.size() != nb || cfg_.parent.size() != nb ||
            cfg_.bodyPos.size() != nb || cfg_.bodyQuat.size() != nb ||
            cfg_.jointAxis.size() != nb || cfg_.jointDof.size() != nb ||
            cfg_.fixedOffset.size() != nb) {
            error = "fk table is not " + std::to_string(nb) + " entries wide";
            return false;
        }
        if (cfg_.dofAxes.size() != nb - 1 || cfg_.dofIndex.size() != nb - 1) {
            error = "dof_axes/dof_index must have num_bodies-1 entries";
            return false;
        }
        if (cfg_.keyBodyIds.size() != cfg_.keyBodyOffsets.size()) {
            error = "key_body_ids and key_body_offsets disagree in length";
            return false;
        }
        if (cfg_.defaultAngles.size() != cfg_.numActions ||
            cfg_.jointStiffness.size() != cfg_.numActions ||
            cfg_.jointDamping.size() != cfg_.numActions ||
            cfg_.jointTorqueLimit.size() != cfg_.numActions ||
            static_cast<int>(cfg_.policyJointNames.size()) != cfg_.numActions) {
            error = "per-joint arrays must all have num_actions entries";
            return false;
        }
        // A body must come after its parent, or the single forward sweep in
        // forwardKinematics would read a pose that has not been written yet.
        for (size_t i = 1; i < nb; i++) {
            if (cfg_.parent[i] < 0 || cfg_.parent[i] >= static_cast<int>(i)) {
                error = "fk parent[" + std::to_string(i) + "] is not a lower index";
                return false;
            }
        }

        const std::filesystem::path dir = std::filesystem::path(configPath).parent_path();
        return loadMotion((dir / cfg_.motionName).string(), error);
    }

    bool DiffTrackObsBuilder::loadMotion(const std::string &path, std::string &error) {
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (f.fail()) {
            error = "cannot open motion file " + path;
            return false;
        }
        const std::streamsize bytes = f.tellg();
        f.seekg(0, std::ios::beg);

        const size_t S = cfg_.tarObsSteps.size();
        const size_t K = static_cast<size_t>(cfg_.clipSteps);
        const size_t rot6 = 6 * static_cast<size_t>(cfg_.numBodies - 1);
        const size_t key3 = 3 * static_cast<size_t>(cfg_.numKeyPoints);

        const size_t nTarPos = S * K * 3;
        const size_t nTarQuat = S * K * 4;
        const size_t nTarRot6 = S * K * rot6;
        const size_t nTarKey = S * K * key3;
        const size_t nRefPos = K * 3;
        const size_t nRefQuat = K * 4;
        const size_t nRefDof = K * static_cast<size_t>(cfg_.numActions);
        const size_t total =
            nTarPos + nTarQuat + nTarRot6 + nTarKey + nRefPos + nRefQuat + nRefDof;

        if (static_cast<size_t>(bytes) != total * sizeof(float)) {
            std::ostringstream os;
            os << path << " is " << bytes << " bytes but the config describes "
               << total * sizeof(float) << " (" << total << " floats)";
            error = os.str();
            return false;
        }

        auto read = [&](std::vector<float> &dst, size_t n) {
            dst.resize(n);
            f.read(reinterpret_cast<char *>(dst.data()),
                   static_cast<std::streamsize>(n * sizeof(float)));
        };
        read(tarRootPos_, nTarPos);
        read(tarRootQuat_, nTarQuat);
        read(tarJointRot6_, nTarRot6);
        read(tarKeyRel_, nTarKey);
        read(refRootPos_, nRefPos);
        read(refRootQuat_, nRefQuat);
        read(refDofPos_, nRefDof);

        if (!f) {
            error = "short read on " + path;
            return false;
        }
        return true;
    }

    int DiffTrackObsBuilder::clampStep(int clipStep) const {
        return std::max(0, std::min(clipStep, cfg_.clipSteps - 1));
    }

    int DiffTrackObsBuilder::resolveStep(int clipStep, Eigen::Vector3d &rootOffset) const {
        rootOffset.setZero();
        if (!cfg_.loopWrap) {
            return clampStep(clipStep);
        }
        // Floor division, so a step taken from inside a lead-in (which counts
        // backwards from clip frame 0) reads the tail of the previous repeat
        // rather than folding onto the wrong frame.
        const int n = cfg_.clipSteps;
        int loops = clipStep / n;
        int idx = clipStep - loops * n;
        if (idx < 0) {
            idx += n;
            loops -= 1;
        }
        rootOffset = static_cast<double>(loops) * cfg_.wrapDelta;
        return idx;
    }

    void DiffTrackObsBuilder::clipFrame(int k, Eigen::Vector3d &rootPos,
                                        Eigen::Quaterniond &rootQuat,
                                        Eigen::VectorXd &dofPos) const {
        Eigen::Vector3d wrapOffset;
        const int c = resolveStep(k, wrapOffset);
        rootPos = Eigen::Vector3d(refRootPos_[3 * c], refRootPos_[3 * c + 1],
                                  refRootPos_[3 * c + 2]) + wrapOffset;
        rootQuat = Eigen::Quaterniond(refRootQuat_[4 * c], refRootQuat_[4 * c + 1],
                                      refRootQuat_[4 * c + 2], refRootQuat_[4 * c + 3]);
        rootQuat.normalize();
        dofPos.resize(cfg_.numActions);
        for (int i = 0; i < cfg_.numActions; i++) {
            dofPos[i] = refDofPos_[static_cast<size_t>(c) * cfg_.numActions + i];
        }
    }

    // ----------------------------------------------------------------------
    // Anchoring
    // ----------------------------------------------------------------------

    MotionAnchor DiffTrackObsBuilder::makeAnchor(const Eigen::Vector3d &robotPos,
                                                 const Eigen::Quaterniond &robotQuat,
                                                 int episodeStep, bool matchYaw) const {
        Eigen::Vector3d wrapOffset;
        const int k = resolveStep(episodeStep, wrapOffset);
        const Eigen::Vector3d clipPos =
            Eigen::Vector3d(refRootPos_[3 * k], refRootPos_[3 * k + 1],
                            refRootPos_[3 * k + 2]) + wrapOffset;
        Eigen::Quaterniond clipQuat(refRootQuat_[4 * k], refRootQuat_[4 * k + 1],
                                    refRootQuat_[4 * k + 2], refRootQuat_[4 * k + 3]);
        clipQuat.normalize();
        Eigen::Quaterniond rq = robotQuat;
        rq.normalize();

        MotionAnchor a;
        a.yaw = matchYaw ? headingYaw(rq) - headingYaw(clipQuat) : 0.0;
        const Eigen::Matrix3d R = a.rotation().toRotationMatrix();
        const Eigen::Vector2d rotatedClipXy = (R * clipPos).head<2>();
        a.translation = robotPos.head<2>() - rotatedClipXy;
        a.identity = false;
        return a;
    }

    DiffTrackState toReferenceFrame(const DiffTrackState &world, const MotionAnchor &anchor) {
        if (anchor.identity) {
            return world;
        }
        // Rz(-yaw) rather than anchor.rotation().conjugate(): the anchor is a
        // pure yaw, so the inverse is the same rotation about -yaw, and building
        // it directly keeps this free of any accumulated normalisation error.
        const Eigen::Quaterniond qi(
            Eigen::AngleAxisd(-anchor.yaw, Eigen::Vector3d::UnitZ()));

        DiffTrackState s = world;   // joint positions and velocities carry no frame
        Eigen::Vector3d p = world.rootPos;
        p.head<2>() -= anchor.translation;
        s.rootPos = qi * p;
        s.rootQuat = qi * world.rootQuat;
        s.rootQuat.normalize();
        s.rootLinVelWorld = qi * world.rootLinVelWorld;
        s.rootAngVelWorld = qi * world.rootAngVelWorld;
        return s;
    }

    // ----------------------------------------------------------------------
    // Lead-in
    // ----------------------------------------------------------------------

    void DiffTrackObsBuilder::encodeFrame(SynthFrame &frame) const {
        double buf6[6];
        frame.jointRot6.assign(6 * static_cast<size_t>(cfg_.numBodies - 1), 0.0f);
        size_t o = 0;
        for (size_t j = 0; j < cfg_.dofIndex.size(); j++) {
            const int d = cfg_.dofIndex[j];
            const Eigen::Quaterniond q =
                (d < 0) ? Eigen::Quaterniond::Identity()
                        : axisAngle(cfg_.dofAxes[j], frame.dofPos[d]);
            tanNorm(q, buf6);
            for (int i = 0; i < 6; i++) {
                frame.jointRot6[o++] = static_cast<float>(buf6[i]);
            }
        }

        std::vector<Eigen::Vector3d> bodyPos;
        std::vector<Eigen::Quaterniond> bodyQuat;
        forwardKinematics(frame.rootPos, frame.rootQuat, frame.dofPos, bodyPos, bodyQuat);
        frame.keyRel.assign(3 * static_cast<size_t>(cfg_.numKeyPoints), 0.0f);
        for (size_t k = 0; k < cfg_.keyBodyIds.size(); k++) {
            const int b = cfg_.keyBodyIds[k];
            const Eigen::Vector3d p =
                bodyPos[b] + bodyQuat[b] * cfg_.keyBodyOffsets[k] - frame.rootPos;
            for (int i = 0; i < 3; i++) {
                frame.keyRel[3 * k + i] = static_cast<float>(p[i]);
            }
        }
    }

    MotionAnchor DiffTrackObsBuilder::buildLeadIn(const DiffTrackState &robot,
                                                  double durationS, bool matchYaw) {
        leadIn_.clear();

        Eigen::Quaterniond q0 = robot.rootQuat;
        q0.normalize();
        if (durationS <= 0.0) {
            return makeAnchor(robot.rootPos, q0, 0, matchYaw);
        }

        // -- where the clip has to end up -------------------------------------
        // Only the yaw is known before the translation is solved for, but the
        // translation depends on the clip's entry velocity, which the yaw already
        // fixes -- so the two are resolved in that order, not simultaneously.
        Eigen::Vector3d clipPos;
        Eigen::Quaterniond clipQuat;
        Eigen::VectorXd clipDof;
        clipFrame(0, clipPos, clipQuat, clipDof);

        MotionAnchor anchor;
        anchor.yaw = matchYaw ? headingYaw(q0) - headingYaw(clipQuat) : 0.0;
        anchor.identity = false;
        const Eigen::Quaterniond qa = anchor.rotation();

        const Eigen::Vector3d v1 = qa * cfg_.rsi.rootLinVel;      // clip entry velocity
        const Eigen::Vector3d &v0 = robot.rootLinVelWorld;

        // Travel of a constant-acceleration ramp from v0 to v1 over the lead-in.
        // Solving the anchor's translation for it is what makes the reference
        // accelerate into the clip instead of teleporting up to its entry speed.
        const Eigen::Vector2d travel = 0.5 * durationS * (v0 + v1).head<2>();
        anchor.translation = robot.rootPos.head<2>() + travel - (qa * clipPos).head<2>();

        // -- the approach, in the CLIP's own frame ------------------------------
        // The segment used to be laid out in the world, which made it unreadable
        // through the identity anchor `observe_in_reference_frame` uses and so
        // confined the lead-in to world-frame observations -- i.e. to RSI, the one
        // entry that needs it least. Built here instead and anchored on the way
        // out, exactly as the clip's own table is, it works from a stand too.
        //
        // The robot goes into that frame with it: the anchor is solved above, so
        // mapping through it is exact.
        const DiffTrackState r = toReferenceFrame(robot, anchor);
        const Eigen::Vector3d &v0r = r.rootLinVelWorld;
        const Eigen::Vector3d &w0r = r.rootAngVelWorld;

        // -- the blend ---------------------------------------------------------
        // Orientation rides on the rotation vector that takes the robot to clip
        // frame 0, so the same Hermite that moves the root also turns it, and both
        // are C1.
        const Eigen::Vector3d dRot = quatLog(clipQuat * r.rootQuat.conjugate());
        const int steps = std::max(1, static_cast<int>(std::lround(durationS / cfg_.controlDt)));
        const double duration = steps * cfg_.controlDt;

        leadIn_.resize(static_cast<size_t>(steps) + 1);
        for (int j = 0; j <= steps; j++) {
            const double s = static_cast<double>(j) / static_cast<double>(steps);
            SynthFrame &f = leadIn_[static_cast<size_t>(j)];
            f.anchored = false;
            if (j == steps) {
                // Close the segment on the clip's own first frame exactly rather
                // than on the Hermite's value there, so the handover carries no
                // round-off step at all.
                f.rootPos = clipPos;
                f.rootQuat = clipQuat;
                f.dofPos = clipDof;
            } else {
                f.rootPos = hermite(r.rootPos, v0r, clipPos, cfg_.rsi.rootLinVel, s, duration);
                f.rootQuat = quatExp(hermite(Eigen::Vector3d::Zero().eval(), w0r, dRot,
                                             cfg_.rsi.rootAngVelWorld, s, duration)) *
                             r.rootQuat;
                f.rootQuat.normalize();
                f.dofPos = hermite(r.dofPos, r.dofVel, clipDof, cfg_.rsi.dofVel,
                                   s, duration);
            }
            encodeFrame(f);
        }
        return anchor;
    }

    // ----------------------------------------------------------------------
    // Blends
    // ----------------------------------------------------------------------

    const SynthFrame *DiffTrackObsBuilder::synthFrame(int episodeStep) const {
        if (episodeStep < 0) {
            episodeStep = 0;
        }
        // The lead-in comes first and owns steps 0..leadInSteps(); its last entry
        // IS clip frame 0, so a blend-in built alongside one would be fighting it
        // for that step. The node refuses the combination; this is the tie-break
        // if it ever gets here anyway.
        const int lead = leadInSteps();
        if (!leadIn_.empty() && episodeStep <= lead) {
            return &leadIn_[static_cast<size_t>(episodeStep)];
        }
        if (!blendIn_.empty() && episodeStep <= blendInSteps()) {
            return &blendIn_[static_cast<size_t>(episodeStep)];
        }
        if (blendOutFirst_ >= 0 && episodeStep >= blendOutFirst_) {
            // Past the end the standing frame is held, which is what a lookahead
            // taken on the last played step reads.
            const size_t j = std::min(static_cast<size_t>(episodeStep - blendOutFirst_),
                                      blendOut_.size() - 1);
            return &blendOut_[j];
        }
        return nullptr;
    }

    double DiffTrackObsBuilder::standRootHeight() const {
        std::vector<Eigen::Vector3d> bodyPos;
        std::vector<Eigen::Quaterniond> bodyQuat;
        auto lowest = [&](const Eigen::Vector3d &p, const Eigen::Quaterniond &q,
                          const Eigen::VectorXd &dof) {
            forwardKinematics(p, q, dof, bodyPos, bodyQuat);
            double low = bodyPos[0].z();
            for (const auto &b : bodyPos) {
                low = std::min(low, b.z());
            }
            return low;
        };

        // WHERE THE FLOOR IS, as the clip sees it: the lowest any body gets over
        // the whole clip, which is a planted foot. The clip's tables are not
        // referenced to z = 0 -- the G1's ankle_roll body origin sits about 5 cm
        // above the sole -- so the offset has to come out of the clip rather than
        // be assumed, and it has to come out of the WHOLE clip rather than off the
        // frame the run happens to end on. Measured over the shipped clips, the
        // last frame sits 0 to 6.8 cm above the clip's own floor (g1_run is the
        // worst), and reading the height off it would ask the rest state to catch
        // a robot standing that far up on its toes. Whole-clip: 0.788 to 0.800 m
        // across all five, which is what a G1 in this default pose stands at.
        double floor = std::numeric_limits<double>::max();
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        Eigen::VectorXd dof;
        for (int k = 0; k < cfg_.clipSteps; k++) {
            clipFrame(k, p, q, dof);
            floor = std::min(floor, lowest(p, q, dof));
        }

        // ... and where the default pose's lowest body would be with the root at
        // the origin. Put the two together and the pose stands on that floor.
        const double standLow = lowest(Eigen::Vector3d::Zero(),
                                       Eigen::Quaterniond::Identity(), cfg_.defaultAngles);
        return floor - standLow;
    }

    MotionAnchor DiffTrackObsBuilder::buildBlendIn(const DiffTrackState &robot,
                                                   double durationS, bool matchYaw) {
        blendIn_.clear();

        Eigen::Quaterniond q0 = robot.rootQuat;
        q0.normalize();
        if (durationS <= 0.0) {
            return makeAnchor(robot.rootPos, q0, 0, matchYaw);
        }

        const int steps =
            std::max(1, static_cast<int>(std::lround(durationS / cfg_.controlDt)));

        // The clip is placed exactly as it is without a blend: frame 0 on the
        // robot, in yaw and horizontal position.
        //
        // It is worth saying what is NOT done here, because it is the obvious
        // idea and it is wrong. A standing robot cannot have the clip's entry
        // velocity, so the reference looks like it ought to be set back by the
        // travel the robot gives up while it accelerates -- the lead-in's
        // trapezoid, half a metre on g1_fight. Measured, that makes it worse: the
        // robot does not chase the reference root, it EXECUTES THE MOTION, and the
        // motion carries the root the clip's own distance whatever the root
        // reference says. Setting the clip back therefore does not close the
        // error, it opens a permanent one -- 0.47 m of standing offset for the
        // rest of the run, where the same clip with the plain anchor settles to
        // 0.1 m. What a standing start actually costs is one step of
        // acceleration, and the policy pays that by itself.
        MotionAnchor anchor = makeAnchor(robot.rootPos, q0, 0, matchYaw);

        Eigen::Vector3d clip0Pos;
        Eigen::Quaterniond clip0Quat;
        Eigen::VectorXd clip0Dof;
        clipFrame(0, clip0Pos, clip0Quat, clip0Dof);

        // -- the offsets that are faded out ------------------------------------
        // Built in the CLIP's own frame, not the world: the frames then read back
        // correctly through the identity anchor that `observe_in_reference_frame`
        // uses AND through the real one that a world-frame observation uses.
        //
        // Everything below is a DIFFERENCE from the clip, so the clip's own motion
        // is played unaltered underneath and only the robot's disagreement with
        // frame 0 is interpolated away.
        const DiffTrackState r = toReferenceFrame(robot, anchor);

        const Eigen::Vector3d dPos = r.rootPos - clip0Pos;
        const Eigen::Vector3d dRot = quatLog(r.rootQuat * clip0Quat.conjugate());
        const Eigen::VectorXd dDof = r.dofPos - clip0Dof;

        blendIn_.resize(static_cast<size_t>(steps) + 1);
        for (int j = 0; j <= steps; j++) {
            const double s = static_cast<double>(j) / static_cast<double>(steps);
            SynthFrame &f = blendIn_[static_cast<size_t>(j)];
            f.anchored = false;

            Eigen::Vector3d cPos;
            Eigen::Quaterniond cQuat;
            Eigen::VectorXd cDof;
            clipFrame(j, cPos, cQuat, cDof);

            if (j == steps) {
                // Close on the clip exactly, so the step where the table runs out
                // and the clip takes over carries no round-off at all.
                f.rootPos = cPos;
                f.rootQuat = cQuat;
                f.dofPos = cDof;
            } else {
                // One smoothstep for all three, so the frame stays a consistent
                // pose the whole way across: zero slope at both ends, so neither
                // the first step of the blend nor the handover to the clip steps
                // the reference's velocity.
                const double a = 1.0 - s * s * (3.0 - 2.0 * s);
                f.rootPos = cPos + a * dPos;
                f.rootQuat = quatExp(a * dRot) * cQuat;
                f.rootQuat.normalize();
                f.dofPos = cDof + a * dDof;
            }
            encodeFrame(f);
        }
        return anchor;
    }

    void DiffTrackObsBuilder::buildBlendOut(int lastEpisodeStep, double durationS) {
        blendOut_.clear();
        blendOutFirst_ = -1;
        if (durationS <= 0.0) {
            return;
        }

        const int lead = leadInSteps();
        int steps = std::max(1, static_cast<int>(std::lround(durationS / cfg_.controlDt)));
        // Never eat into the blend-in, and never start before the episode does: a
        // blend-out longer than what is left of the run would have the reference
        // walking off the clip before the robot is on it.
        const int earliest = std::max(std::max(lead, blendInSteps()), 0);
        if (lastEpisodeStep - steps < earliest) {
            steps = lastEpisodeStep - earliest;
        }
        if (steps < 1) {
            return;
        }
        blendOutFirst_ = lastEpisodeStep - steps;

        // -- the frame the run is to end on, in the CLIP's own frame ------------
        // As the blend-in, and for the same reason: the anchor is applied on the
        // way out, so one table serves both observation frames.
        const int lastClip = clampStep(lastEpisodeStep - lead);
        Eigen::Vector3d endPos;
        Eigen::Quaterniond endQuat;
        Eigen::VectorXd endDof;
        clipFrame(lastClip, endPos, endQuat, endDof);

        // Standing where the clip ends: the clip's own last position, at the
        // height the default pose stands at with its feet on the floor, upright on
        // the heading the clip has there -- the yaw is kept and the lean is taken
        // out, because a rest state cannot catch a robot it is handed at 20
        // degrees of pitch. Keeping the horizontal position means the reference
        // ends where it would have ended anyway, so what this changes is the POSE
        // the run finishes in and not how far it got.
        Eigen::Vector3d standPos = endPos;
        standPos.z() = standRootHeight();
        const Eigen::Quaterniond standQuat(
            Eigen::AngleAxisd(headingYaw(endQuat), Eigen::Vector3d::UnitZ()));
        const Eigen::VectorXd &standDof = cfg_.defaultAngles;

        blendOut_.resize(static_cast<size_t>(steps) + 1);
        for (int j = 0; j <= steps; j++) {
            const double s = static_cast<double>(j) / static_cast<double>(steps);
            SynthFrame &f = blendOut_[static_cast<size_t>(j)];
            f.anchored = false;

            const int e = blendOutFirst_ + j;
            Eigen::Vector3d cPos;
            Eigen::Quaterniond cQuat;
            Eigen::VectorXd cDof;
            clipFrame(e - lead, cPos, cQuat, cDof);

            // An INTERPOLATION onto the standing frame, not an offset added to the
            // clip -- which is the one asymmetry with the blend-in, and it is the
            // point of the thing. An offset would leave the clip's full amplitude
            // running underneath right up to the last step and then require the
            // reference to reach the standing pose in one of them; interpolating
            // damps the motion out as it goes, which is what "comes to rest" means.
            const double a = s * s * (3.0 - 2.0 * s);
            f.rootPos = (1.0 - a) * cPos + a * standPos;
            f.rootQuat = cQuat.slerp(a, standQuat);
            f.rootQuat.normalize();
            f.dofPos = (1.0 - a) * cDof + a * standDof;
            encodeFrame(f);
        }
    }

    // ----------------------------------------------------------------------
    // Kinematics
    // ----------------------------------------------------------------------

    void DiffTrackObsBuilder::forwardKinematics(const Eigen::Vector3d &rootPos,
                                                const Eigen::Quaterniond &rootQuat,
                                                const Eigen::VectorXd &dofPos,
                                                std::vector<Eigen::Vector3d> &bodyPos,
                                                std::vector<Eigen::Quaterniond> &bodyQuat) const {
        const size_t nb = static_cast<size_t>(cfg_.numBodies);
        bodyPos.resize(nb);
        bodyQuat.resize(nb);
        bodyPos[0] = rootPos;
        bodyQuat[0] = rootQuat;

        // Single forward sweep; parents are guaranteed to precede children (checked
        // in load()). This is mj_kinematics for a tree of body-origin hinges, which
        // the exporter asserts the G1 is.
        for (size_t i = 1; i < nb; i++) {
            const int p = cfg_.parent[i];
            const Eigen::Quaterniond &pq = bodyQuat[p];
            if (cfg_.jointDof[i] < 0) {
                // Welded to the parent (head_link): a fixed offset, no rotation.
                bodyQuat[i] = pq;
                bodyPos[i] = bodyPos[p] + pq * cfg_.fixedOffset[i];
                continue;
            }
            bodyPos[i] = bodyPos[p] + pq * cfg_.bodyPos[i];
            bodyQuat[i] = pq * cfg_.bodyQuat[i] *
                          axisAngle(cfg_.jointAxis[i], dofPos[cfg_.jointDof[i]]);
        }
    }

    void DiffTrackObsBuilder::referencePose(int episodeStep, const MotionAnchor &anchor,
                                            Eigen::Vector3d &rootPos,
                                            Eigen::Quaterniond &rootQuat,
                                            Eigen::VectorXd &dofPos) const {
        const int lead = leadInSteps();
        // A synthesised frame -- lead-in, blend-in or blend-out -- is what the
        // policy was shown for this step, so it is also what the tracking error is
        // against. They are stored in the clip's own frame and anchored here, the
        // same way the clip's table is.
        if (const SynthFrame *f = synthFrame(episodeStep)) {
            rootPos = f->rootPos;
            rootQuat = f->rootQuat;
            dofPos = f->dofPos;
            if (!f->anchored && !anchor.identity) {
                const Eigen::Quaterniond qa = anchor.rotation();
                rootPos = qa * rootPos;
                rootPos.head<2>() += anchor.translation;
                rootQuat = qa * rootQuat;
            }
            return;
        }

        Eigen::Vector3d wrapOffset;
        const int k = resolveStep(episodeStep - lead, wrapOffset);
        Eigen::Vector3d p = Eigen::Vector3d(refRootPos_[3 * k], refRootPos_[3 * k + 1],
                                            refRootPos_[3 * k + 2]) + wrapOffset;
        Eigen::Quaterniond q(refRootQuat_[4 * k], refRootQuat_[4 * k + 1],
                             refRootQuat_[4 * k + 2], refRootQuat_[4 * k + 3]);
        if (!anchor.identity) {
            const Eigen::Quaterniond qa = anchor.rotation();
            p = qa * p;
            p.head<2>() += anchor.translation;
            q = qa * q;
        }
        rootPos = p;
        rootQuat = q;
        dofPos.resize(cfg_.numActions);
        for (int i = 0; i < cfg_.numActions; i++) {
            dofPos[i] = refDofPos_[static_cast<size_t>(k) * cfg_.numActions + i];
        }
    }

    // ----------------------------------------------------------------------
    // Observation
    // ----------------------------------------------------------------------

    void DiffTrackObsBuilder::computeObs(const DiffTrackState &state, int episodeStep,
                                         const MotionAnchor &anchor,
                                         std::vector<float> &obs) const {
        obs.assign(static_cast<size_t>(cfg_.numObs), 0.0f);
        size_t o = 0;
        double buf6[6];

        // Normalise rather than trust the caller. tanNorm and the FK chain both
        // assume a unit quaternion, and a state estimator has no obligation to
        // supply one -- the reference clip's own frames are off by up to 3e-5,
        // and MuJoCo hides that by normalising inside mj_kinematics. Skipping it
        // shows up as a sub-millimetre error on the key points, which is small
        // enough to look like round-off and not small enough to be it.
        Eigen::Quaterniond rootQuat = state.rootQuat;
        rootQuat.normalize();

        // ---- char_obs: the robot's own state, world frame --------------------
        //
        // Layout mirrors sim2mujoco/mujoco_env.py:_compute_char_obs exactly:
        //   root_height, root_rot(6), root_vel(3), root_ang_vel(3),
        //   joint_rot(6 per non-root body), dof_vel, key_pos relative to root.

        if (cfg_.rootHeightObs) {
            obs[o++] = static_cast<float>(state.rootPos.z());
        }

        tanNorm(rootQuat, buf6);
        for (int i = 0; i < 6; i++) {
            obs[o++] = static_cast<float>(buf6[i]);
        }
        for (int i = 0; i < 3; i++) {
            obs[o++] = static_cast<float>(state.rootLinVelWorld[i]);
        }
        for (int i = 0; i < 3; i++) {
            obs[o++] = static_cast<float>(state.rootAngVelWorld[i]);
        }

        // Local joint rotations, rebuilt from the encoders. A welded joint
        // contributes the identity, whose tan-norm is the identity basis.
        for (size_t j = 0; j < cfg_.dofIndex.size(); j++) {
            const int d = cfg_.dofIndex[j];
            const Eigen::Quaterniond q =
                (d < 0) ? Eigen::Quaterniond::Identity()
                        : axisAngle(cfg_.dofAxes[j], state.dofPos[d]);
            tanNorm(q, buf6);
            for (int i = 0; i < 6; i++) {
                obs[o++] = static_cast<float>(buf6[i]);
            }
        }

        for (int i = 0; i < cfg_.numActions; i++) {
            obs[o++] = static_cast<float>(state.dofVel[i]);
        }

        std::vector<Eigen::Vector3d> bodyPos;
        std::vector<Eigen::Quaterniond> bodyQuat;
        forwardKinematics(state.rootPos, rootQuat, state.dofPos, bodyPos, bodyQuat);
        for (size_t k = 0; k < cfg_.keyBodyIds.size(); k++) {
            const int b = cfg_.keyBodyIds[k];
            const Eigen::Vector3d p =
                bodyPos[b] + bodyQuat[b] * cfg_.keyBodyOffsets[k] - state.rootPos;
            for (int i = 0; i < 3; i++) {
                obs[o++] = static_cast<float>(p[i]);
            }
        }

        // ---- tar_obs: future reference frames --------------------------------
        //
        // The clip's joint rotations are local and survive the anchor untouched,
        // so they come straight out of the table. Everything world-frame has the
        // anchor applied first and is encoded afterwards: tan-norm does not
        // commute with a rotation, so the 6D block cannot be rotated after it has
        // been formed. With the identity anchor the arithmetic collapses to a
        // copy plus the agent-relative XY, which is what the exporter's own
        // reduction produced.
        const int lead = leadInSteps();
        Eigen::Vector3d wrapOffset;
        const int k = resolveStep(episodeStep - lead, wrapOffset);
        const size_t rot6 = 6 * static_cast<size_t>(cfg_.numBodies - 1);
        const size_t nKey = static_cast<size_t>(cfg_.numKeyPoints);
        const size_t clip = static_cast<size_t>(cfg_.clipSteps);
        const Eigen::Quaterniond qa =
            anchor.identity ? Eigen::Quaterniond::Identity() : anchor.rotation();

        for (size_t s = 0; s < cfg_.tarObsSteps.size(); s++) {
            // A lookahead that lands on a synthesised step reads that frame, and
            // one that steps off the end of a synthesised segment into the clip's
            // own table cannot use slot s's -- slot 0 is one step ahead, so clip
            // frame c is that slot at c-1, and reading it there is the only way to
            // land on a frame the segment does not cover. The lead-in's last entry
            // IS clip frame 0, which is what covers c == 0.
            const int ahead = episodeStep + cfg_.tarObsSteps[s];
            const SynthFrame *leadFrame = synthFrame(ahead);
            size_t idx;
            // Root offset carried by however many times the clip has looped.
            // Slot 0's table already holds the reference one step ahead, so a
            // lookahead read out of it wraps on its own step, not on k's.
            Eigen::Vector3d slotOffset = wrapOffset;
            if (leadFrame) {
                idx = 0;
            } else if (hasSynthFrames()) {
                idx = 0 * clip + static_cast<size_t>(resolveStep(ahead - lead - 1, slotOffset));
            } else {
                idx = s * clip + static_cast<size_t>(k);
            }

            Eigen::Vector3d p;
            Eigen::Quaterniond q;
            if (leadFrame) {
                p = leadFrame->rootPos;
                q = leadFrame->rootQuat;
                if (!leadFrame->anchored && !anchor.identity) {
                    // A blend frame: in the clip's frame, so it takes the anchor
                    // the clip's own table takes.
                    p = qa * p;
                    p.head<2>() += anchor.translation;
                    q = qa * q;
                }
            } else {
                p = Eigen::Vector3d(tarRootPos_[3 * idx], tarRootPos_[3 * idx + 1],
                                    tarRootPos_[3 * idx + 2]) + slotOffset;
                q = Eigen::Quaterniond(tarRootQuat_[4 * idx], tarRootQuat_[4 * idx + 1],
                                       tarRootQuat_[4 * idx + 2], tarRootQuat_[4 * idx + 3]);
                if (!anchor.identity) {
                    p = qa * p;
                    p.head<2>() += anchor.translation;
                    q = qa * q;
                }
            }

            // Reference root position, relative to the agent in XY. Height stays
            // absolute -- it is measured against the floor, not the robot.
            obs[o++] = static_cast<float>(p.x() - state.rootPos.x());
            obs[o++] = static_cast<float>(p.y() - state.rootPos.y());
            if (cfg_.rootHeightObs) {
                obs[o++] = static_cast<float>(p.z());
            }

            tanNorm(q, buf6);
            for (int i = 0; i < 6; i++) {
                obs[o++] = static_cast<float>(buf6[i]);
            }

            const float *rot = leadFrame ? leadFrame->jointRot6.data() : &tarJointRot6_[idx * rot6];
            for (size_t i = 0; i < rot6; i++) {
                obs[o++] = rot[i];
            }

            const float *key = leadFrame ? leadFrame->keyRel.data() : &tarKeyRel_[idx * 3 * nKey];
            for (size_t i = 0; i < nKey; i++) {
                if (anchor.identity || (leadFrame && leadFrame->anchored)) {
                    obs[o++] = key[3 * i + 0];
                    obs[o++] = key[3 * i + 1];
                    obs[o++] = key[3 * i + 2];
                } else {
                    const Eigen::Vector3d kp =
                        qa * Eigen::Vector3d(key[3 * i], key[3 * i + 1], key[3 * i + 2]);
                    obs[o++] = static_cast<float>(kp.x());
                    obs[o++] = static_cast<float>(kp.y());
                    obs[o++] = static_cast<float>(kp.z());
                }
            }
        }
    }

    void DiffTrackObsBuilder::actionToJointTarget(const Eigen::VectorXd &action,
                                                  Eigen::VectorXd &target) const {
        target.resize(cfg_.numActions);
        for (int i = 0; i < cfg_.numActions; i++) {
            double delta = cfg_.actionScale * action[i];
            if (cfg_.actionClipping) {
                delta = std::max(-cfg_.actionClipRange,
                                 std::min(cfg_.actionClipRange, delta));
            }
            target[i] = cfg_.defaultAngles[i] + delta;
        }
    }

    // ----------------------------------------------------------------------
    // WorldStateGuard
    // ----------------------------------------------------------------------

    void WorldStateGuard::configure(const WorldStateGuardConfig &cfg) {
        cfg_ = cfg;
        const double dt = cfg_.dt > 0.0 ? cfg_.dt : 0.02;
        window_ticks_ = std::max(1, static_cast<int>(std::lround(cfg_.windowS / dt)));
        hold_ticks_ = std::max(0, static_cast<int>(std::lround(cfg_.holdS / dt)));
        reset();
    }

    void WorldStateGuard::reset() {
        accepted_.assign(static_cast<size_t>(window_ticks_), 0.0);
        head_ = 0;
        have_prev_ = false;
        prev_raw_.setZero();
        prev_ref_.setZero();
        offset_.setZero();
        ticks_since_reject_ = std::numeric_limits<int>::max() / 2;
        ever_rejected_ = false;
        rejected_now_ = 0.0;
        vel_clamped_now_ = false;
        max_offset_ = 0.0;
        reject_ticks_ = 0;
        vel_clamp_ticks_ = 0;
    }

    void WorldStateGuard::apply(Eigen::Vector3d &rootPos, Eigen::Vector3d &rootLinVelWorld,
                                const Eigen::Vector3d &refPos, bool gate) {
        rejected_now_ = 0.0;
        vel_clamped_now_ = false;
        if (!cfg_.enabled)
            return;

        const Eigen::Vector2d raw = rootPos.head<2>();
        const Eigen::Vector2d ref = refPos.head<2>();
        const double dt = cfg_.dt > 0.0 ? cfg_.dt : 0.02;

        if (!have_prev_) {
            // First tick after a reset: nothing to difference against. The
            // anchor was resolved from this very state, so it is the baseline.
            have_prev_ = true;
            prev_raw_ = raw;
            prev_ref_ = ref;
            rootPos.head<2>() = raw - offset_;
            return;
        }

        const Eigen::Vector2d ref_step = ref - prev_ref_;
        const Eigen::Vector2d ref_vel = ref_step / dt;
        // Horizontal motion this tick that the reference does not account for.
        const Eigen::Vector2d unexplained = (raw - prev_raw_) - ref_step;

        Eigen::Vector2d accepted = unexplained;
        if (gate) {
            // The entry about to be overwritten leaves the window this tick, so
            // it does not count against the budget: any window_ticks_
            // consecutive ticks then pass at most maxUnexplainedM between them.
            double used = 0.0;
            for (int i = 0; i < window_ticks_; ++i)
                if (i != head_)
                    used += accepted_[static_cast<size_t>(i)];
            const double allowed = std::max(0.0, cfg_.maxUnexplainedM - used);
            const double n = unexplained.norm();
            if (n > allowed) {
                accepted = unexplained * (allowed / n);
                const Eigen::Vector2d excess = unexplained - accepted;
                offset_ += excess;
                rejected_now_ = excess.norm();
                ++reject_ticks_;
                ticks_since_reject_ = 0;
                ever_rejected_ = true;
            }
        }
        accepted_[static_cast<size_t>(head_)] = accepted.norm();
        head_ = (head_ + 1) % window_ticks_;
        prev_raw_ = raw;
        prev_ref_ = ref;

        rootPos.head<2>() = raw - offset_;
        max_offset_ = std::max(max_offset_, offset_.norm());

        if (ever_rejected_ && ticks_since_reject_ <= hold_ticks_) {
            const Eigen::Vector2d dv = rootLinVelWorld.head<2>() - ref_vel;
            const double n = dv.norm();
            if (n > cfg_.velDeviationMps) {
                rootLinVelWorld.head<2>() = ref_vel + dv * (cfg_.velDeviationMps / n);
                vel_clamped_now_ = true;
                ++vel_clamp_ticks_;
            }
        }
        if (ticks_since_reject_ < std::numeric_limits<int>::max() / 2)
            ++ticks_since_reject_;
    }

}  // namespace cpp_control::g1::difftrack
