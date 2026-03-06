#include "cpp_control/config_loader.hpp"

#include <fstream>
#include <iostream>

Config::Config(const std::string& config_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(config_path);

        // Control parameters
        if (config["control_dt"])
        {
            control_dt = config["control_dt"].as<double>();
        }
        if (config["msg_type"])
        {
            msg_type = config["msg_type"].as<std::string>();
        }
        if (config["imu_type"])
        {
            imu_type = config["imu_type"].as<std::string>();
        }
        if (config["workflow"])
        {
            workflow = config["workflow"].as<std::string>();
        }

        // Topics
        if (config["lowcmd_topic"])
        {
            lowcmd_topic = config["lowcmd_topic"].as<std::string>();
        }
        if (config["lowstate_topic"])
        {
            lowstate_topic = config["lowstate_topic"].as<std::string>();
        }

        // Model paths
        if (config["policy_path"])
        {
            policy_path = config["policy_path"].as<std::string>();
        }
        if (config["onnx_path"])
        {
            onnx_path = config["onnx_path"].as<std::string>();
        }
        if (config["motion_path"])
        {
            motion_path = config["motion_path"].as<std::string>();
        }

        // Control gains for 29DOF
        if (config["kps"])
        {
            kps = config["kps"].as<std::vector<double>>();
        }
        else
        {
            // Default stiffness values
            kps = std::vector<double>(29, 100.0);
        }

        if (config["kds"])
        {
            kds = config["kds"].as<std::vector<double>>();
        }
        else
        {
            // Default damping values
            kds = std::vector<double>(29, 3.0);
        }

        // Default positions for 29DOF
        if (config["default_angles"])
        {
            default_angles = config["default_angles"].as<std::vector<double>>();
        }
        else
        {
            // Default joint positions
            default_angles = {
                -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,   // left leg
                -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,   // right leg
                0.0, 0.0, 0.0,                     // waist
                0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0, // left arm
                0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0 // right arm
            };
        }

        // Action scaling for 29DOF
        if (config["action_scale"])
        {
            action_scale = config["action_scale"].as<std::vector<double>>();
        }
        else
        {
            // Default action scales
            action_scale = std::vector<double>(29, 0.5);
        }

        // Model dimensions
        if (config["num_actions"])
        {
            num_actions = config["num_actions"].as<int>();
        }
        if (config["num_obs"])
        {
            num_obs = config["num_obs"].as<int>();
        }

        // Joint names for 29DOF (mujoco order)
        joint_names = {
            "left_hip_pitch_joint",      "left_hip_roll_joint",        "left_hip_yaw_joint",
            "left_knee_joint",           "left_ankle_pitch_joint",     "left_ankle_roll_joint",
            "right_hip_pitch_joint",     "right_hip_roll_joint",       "right_hip_yaw_joint",
            "right_knee_joint",          "right_ankle_pitch_joint",    "right_ankle_roll_joint",
            "waist_yaw_joint",           "waist_roll_joint",           "waist_pitch_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",   "left_shoulder_yaw_joint",
            "left_elbow_joint",          "left_wrist_roll_joint",      "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",      "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",  "right_elbow_joint",          "right_wrist_roll_joint",
            "right_wrist_pitch_joint",   "right_wrist_yaw_joint"};

        std::cout << "Config loaded successfully for 29DOF robot" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error loading config file: " << e.what() << std::endl;
        throw;
    }
}
