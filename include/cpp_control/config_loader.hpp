#pragma once

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

struct Config
{
    // Control parameters
    double control_dt = 0.02;
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
