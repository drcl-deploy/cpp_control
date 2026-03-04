#include "cpp_control/tasks/locomotion/g1.hpp"

namespace cpp_control
{

G1LocomotionNode::G1LocomotionNode() : LocomotionBase("g1_locomotion_controller") {}

}  // namespace cpp_control

// ── Entry point ───────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1LocomotionNode>());
    rclcpp::shutdown();
    return 0;
}
