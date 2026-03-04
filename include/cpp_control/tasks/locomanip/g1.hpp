#pragma once

#include "cpp_control/tasks/locomanip/base.hpp"

namespace cpp_control
{

/**
 * @brief G1-specific loco-manipulation node.
 *
 * For the G1 the LocoManipBase already covers everything.
 * Override here only when you need robot-specific observation terms,
 * different hand-tracking topics, etc.
 */
class G1LocoManipNode : public LocoManipBase
{
public:
    G1LocoManipNode();
    ~G1LocoManipNode() override = default;
};

}  // namespace cpp_control
