#include "common/g1/difftrack_obs.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
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

    void DiffTrackObsBuilder::encodeFrame(LeadInFrame &frame) const {
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
        const Eigen::Vector3d w1 = qa * cfg_.rsi.rootAngVelWorld;
        const Eigen::Vector3d &v0 = robot.rootLinVelWorld;
        const Eigen::Vector3d &w0 = robot.rootAngVelWorld;

        // Travel of a constant-acceleration ramp from v0 to v1 over the lead-in.
        // Solving the anchor's translation for it is what makes the reference
        // accelerate into the clip instead of teleporting up to its entry speed.
        const Eigen::Vector2d travel = 0.5 * durationS * (v0 + v1).head<2>();
        anchor.translation = robot.rootPos.head<2>() + travel - (qa * clipPos).head<2>();

        Eigen::Vector3d p1 = qa * clipPos;
        p1.head<2>() += anchor.translation;
        const Eigen::Quaterniond q1 = qa * clipQuat;

        // -- the blend ---------------------------------------------------------
        // Orientation rides on the rotation vector that takes q0 to q1, so the
        // same Hermite that moves the root also turns it, and both are C1.
        const Eigen::Vector3d dRot = quatLog(q1 * q0.conjugate());
        const int steps = std::max(1, static_cast<int>(std::lround(durationS / cfg_.controlDt)));
        const double duration = steps * cfg_.controlDt;

        leadIn_.resize(static_cast<size_t>(steps) + 1);
        for (int j = 0; j <= steps; j++) {
            const double s = static_cast<double>(j) / static_cast<double>(steps);
            LeadInFrame &f = leadIn_[static_cast<size_t>(j)];
            if (j == steps) {
                // Close the segment on the clip's own first frame exactly rather
                // than on the Hermite's value there, so the handover carries no
                // round-off step at all.
                f.rootPos = p1;
                f.rootQuat = q1;
                f.dofPos = clipDof;
            } else {
                f.rootPos = hermite(robot.rootPos, v0, p1, v1, s, duration);
                f.rootQuat = quatExp(hermite(Eigen::Vector3d::Zero().eval(), w0, dRot, w1,
                                             s, duration)) * q0;
                f.rootQuat.normalize();
                f.dofPos = hermite(robot.dofPos, robot.dofVel, clipDof, cfg_.rsi.dofVel,
                                   s, duration);
            }
            encodeFrame(f);
        }
        return anchor;
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
        if (episodeStep < lead) {
            // Lead-in frames are stored already placed in the world.
            const LeadInFrame &f = leadIn_[static_cast<size_t>(std::max(0, episodeStep))];
            rootPos = f.rootPos;
            rootQuat = f.rootQuat;
            dofPos = f.dofPos;
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
            // A lookahead taken from inside the lead-in reads the synthesised
            // frames, and steps off the end of them into the clip's own table --
            // slot 0 is one step ahead, so clip frame c is that slot at c-1. The
            // lead-in's last entry IS clip frame 0, which is what covers c == 0.
            const int ahead = episodeStep + cfg_.tarObsSteps[s];
            const LeadInFrame *leadFrame = nullptr;
            size_t idx;
            // Root offset carried by however many times the clip has looped.
            // Slot 0's table already holds the reference one step ahead, so a
            // lookahead read out of it wraps on its own step, not on k's.
            Eigen::Vector3d slotOffset = wrapOffset;
            if (episodeStep < lead && ahead <= lead) {
                leadFrame = &leadIn_[static_cast<size_t>(ahead)];
                idx = 0;
            } else if (episodeStep < lead) {
                idx = 0 * clip + static_cast<size_t>(resolveStep(ahead - lead - 1, slotOffset));
            } else {
                idx = s * clip + static_cast<size_t>(k);
            }

            Eigen::Vector3d p;
            Eigen::Quaterniond q;
            if (leadFrame) {
                p = leadFrame->rootPos;   // already placed in the world
                q = leadFrame->rootQuat;
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
                if (anchor.identity || leadFrame) {
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

}  // namespace cpp_control::g1::difftrack
