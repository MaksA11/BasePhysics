#pragma once

#include <cmath>
#include <algorithm>

#include <BasePhysics/Core/Math.hpp>

namespace bp::utils
{
    constexpr float epsilon = 0.00005f;

    inline bool NearlyEqual(float value1, float value2)
    {
        return std::abs(value1 - value2) < epsilon;
    }
    inline bool NearlyEqual(const Vec2 &vector1, const Vec2 &vector2)
    {
        return math::DistanceSquared(vector1, vector2) < epsilon * epsilon;
    }

    inline Vec2 ClampMagnitude(const Vec2 &vector, float minValue, float maxValue)
    {
        const float magSq = vector.MagnitudeSquared();

        if(magSq == 0.0f)
            return vector;

        const float mag = std::sqrt(magSq);

        if(mag < minValue)
            return vector * (minValue / mag);

        if(mag > maxValue)
            return vector * (maxValue / mag);

        return vector;
    }
}