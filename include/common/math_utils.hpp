#pragma once

#include <array>
#include <cmath>

namespace cpp_control
{
namespace math
{

inline std::array<float, 3> quat_rotate_inverse(const std::array<float, 4>& q,
                                                  const std::array<float, 3>& v)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    // R^T * v
    return {
        (1.f - 2.f * (y * y + z * z)) * v[0] + (2.f * (x * y - w * z)) * v[1] +
            (2.f * (x * z + w * y)) * v[2],
        (2.f * (x * y + w * z)) * v[0] + (1.f - 2.f * (x * x + z * z)) * v[1] +
            (2.f * (y * z - w * x)) * v[2],
        (2.f * (x * z - w * y)) * v[0] + (2.f * (y * z + w * x)) * v[1] +
            (1.f - 2.f * (x * x + y * y)) * v[2]};
}

inline std::array<float, 3> get_projected_gravity(const std::array<float, 4>& quat)
{
    // Must match the exact formula used during policy training (master branch).
    // This is NOT the standard R^T * [0,0,-1]; it uses a specific sign convention
    // from the IsaacLab training environment.
    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    return {
         2.0f * (-z * x + w * y),
        -2.0f * ( z * y + w * x),
         1.0f - 2.0f * (w * w + z * z)
    };
}

}  // namespace math
}  // namespace cpp_control
