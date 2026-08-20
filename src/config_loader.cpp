#include "cpp_control/config_loader.hpp"

#include "common/asset_path.hpp"

#include <fstream>
#include <iostream>

using cpp_control::asset_path;

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

        // Model paths. Resolved here, once, so a task node never sees a
        // relative string: launch passes these as parameters and the yaml is
        // only read when it does not, which used to hand ORT "locomotion/g1.onnx".
        if (config["policy_path"])
        {
            policy_path = asset_path(config["policy_path"].as<std::string>());
        }
        if (config["onnx_path"])
        {
            onnx_path = asset_path(config["onnx_path"].as<std::string>());
        }
        if (config["hlc_onnx_path"])
        {
            hlc_onnx_path = asset_path(config["hlc_onnx_path"].as<std::string>());
        }
        if (config["motion_path"])
        {
            motion_path = asset_path(config["motion_path"].as<std::string>());
        }
        if (config["stand_onnx_path"])
        {
            stand_onnx_path = asset_path(config["stand_onnx_path"].as<std::string>());
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

        // Motor-to-action index mapping
        if (config["motor2action_id"])
        {
            motor2action_id = config["motor2action_id"].as<std::vector<int>>();
        }

        // Motion padding
        if (config["motion_pad_length"])
            motion_pad_length = config["motion_pad_length"].as<double>();
        if (config["pre_motion_pad"])
            pre_motion_pad = config["pre_motion_pad"].as<bool>();
        if (config["post_motion_pad"])
            post_motion_pad = config["post_motion_pad"].as<bool>();

        // Joint names
        if (config["joint_names"])
        {
            joint_names = config["joint_names"].as<std::vector<std::string>>();
        }

        std::cout << "Config loaded successfully (" << joint_names.size() << " joints)" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error loading config file: " << e.what() << std::endl;
        throw;
    }
}
