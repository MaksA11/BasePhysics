#pragma once

#include <cmath>
#include <algorithm>

#include "Math.hpp"

namespace bp::utils
{
    constexpr float epsilon = 0.00005f;

    inline bool NearlyEqual(float val1, float val2)
    {
        return std::abs(val1 - val2) < epsilon;
    }
    inline bool NearlyEqual(Vec2 val1, Vec2 val2)
    {
        return math::DistanceSquared(val1, val2) < epsilon * epsilon;
    }
}