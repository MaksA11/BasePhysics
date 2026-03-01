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
    inline bool NearlyEqual(const Vec2 &vec1, const Vec2 &vec2)
    {
        return math::DistanceSquared(vec1, vec2) < epsilon * epsilon;
    }

    inline Vec2 ClampMagnitude(const Vec2 &vec, float minVal, float maxVal)
    {
        const float magSq = vec.MagnitudeSquared();

        if(magSq == 0.0f)
            return vec;

        const float mag = std::sqrt(magSq);

        if(mag < minVal)
            return vec * (minVal / mag);

        if(mag > maxVal)
            return vec * (maxVal / mag);

        return vec;
    }
}