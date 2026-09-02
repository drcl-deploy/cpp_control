#pragma once

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

struct Config
{
    // Control parameters
    double control_dt = 0.02;

    // Pace the control loop off the state stream instead of the wall clock:
    // one control step every `state_decimation` state messages. 0 (default)
    // keeps the wall timer.
    //
    // A wall timer assumes the plant runs in real time. mj_sim does not — it
    // steps as fast as its loop goes (measured ~1.7x real time here, and it
    // moves with machine load), so a 50 Hz wall-clock controller hands the
    // robot 40 ms of physics per 20 ms control step and the policy is being
    // asked to control a plant running 1.7x fast. Ticking off the state stream
    // fixes the ratio exactly: mj_sim publishes one message per physics step,
    // so `state_decimation = control_dt / sim_timestep` is the decimation the
    // policy trained with, whatever the wall clock is doing.
    //
    // On hardware set it to (state publish rate x control_dt), or leave it 0
    // and let the wall timer run — there the plant IS real time.
    int state_decimation = 0;
    std::string msg_type = "hg";
    std::string imu_type = "pelvis";
    std::string workflow = "unitree";  // "unitree" or "drcl_deploy"

    // Topics
    std::string lowcmd_topic = "/lowcmd";
    std::string lowstate_topic = "/lowstate";

    // Model paths
    std::string policy_path;
    std::string onnx_path;       // WBC (or single-policy tasks)
    std::string hlc_onnx_path;   // HLC (vibe residual)
    std::string motion_path;
    std::string stand_onnx_path; // robot-level SONIC stand (ControlMode::STAND)

    // Control gains for 29DOF
    std::vector<double> kps;  // [29]
    std::vector<double> kds;  // [29]

    // Default positions for 29DOF
    std::vector<double> default_angles;  // [29]

    // Scaling factors
    std::vector<double> action_scale;  // [29]

    // Model dimensions
    int num_actions = 29;
    int num_obs = 99;

    // Joint names for 29DOF (mujoco order)
    std::vector<std::string> joint_names;

    // Motor-to-action index mapping
    std::vector<int> motor2action_id;

    // Motion padding for smooth stand ↔ motion transitions
    double motion_pad_length = 0.5;  // seconds
    bool pre_motion_pad  = true;
    bool post_motion_pad = true;

    // Stiffness and damping parameters
    std::vector<double> stiffness_params;
    std::vector<double> damping_params;

    Config() = default;
    explicit Config(const std::string& config_path);
};
