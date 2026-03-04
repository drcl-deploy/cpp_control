#pragma once

#include "cpp_control/tasks/locomotion/base.hpp"

namespace cpp_control
{

/**
 * @brief G1-specific locomotion node.
 *
 * For the G1 the LocomotionBase already covers everything.
 * Override here only when you need robot-specific observation terms,
 * different velocity scaling, etc.
 */
class G1LocomotionNode : public LocomotionBase
{
public:
    G1LocomotionNode();
    ~G1LocomotionNode() override = default;
};

}  // namespace cpp_control
