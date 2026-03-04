#include "cpp_control/tasks/locomanip/g1.hpp"

namespace cpp_control
{

G1LocoManipNode::G1LocoManipNode() : LocoManipBase("g1_locomanip_controller") {}

}  // namespace cpp_control

// ── Entry point ───────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpp_control::G1LocoManipNode>());
    rclcpp::shutdown();
    return 0;
}
