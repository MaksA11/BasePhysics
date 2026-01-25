#pragma once

#include <cmath>
#include <algorithm>

#include "Math.hpp"

namespace bp::utils
{
    constexpr float epsilon = 0.00005f;

    inline float Clamp(float val, float min, float max)
    {           
        return std::clamp(val, min, max);
    }
    
    inline float Abs(float val)
    {
        return std::abs(val);
    }
    inline float Min(float val1, float val2)
    {
        return std::min(val1, val2);
    }
    inline float Max(float val1, float val2)
    {
        return std::max(val1, val2);
    }

    inline bool NearlyEqual(float val1, float val2)
    {
        return Abs(val1 - val2) < epsilon;
    }
    inline bool NearlyEqual(Vec2 val1, Vec2 val2)
    {
        return math::DistanceSquared(val1, val2) < epsilon * epsilon;
    }
}