#pragma once

#include "Vec2.h"

namespace bp::math
{
    inline float Dot(const Vec2 &vec1, const Vec2 &vec2)
    {
        return (vec1.x * vec2.x) + (vec1.y * vec2.y);
    }
    inline float Cross(const Vec2 &vec1, const Vec2 &vec2)
    {
        return (vec1.x * vec2.y) - (vec1.y * vec2.x);
    }
    inline float DistanceSquared(const Vec2 &vec1, const Vec2 &vec2)
    {
        return (vec1.x - vec2.x) * (vec1.x - vec2.x) + (vec1.y - vec2.y) * (vec1.y - vec2.y);
    }
    inline float Distance(const Vec2 &vec1, const Vec2 &vec2)
    {
        return std::sqrt(DistanceSquared(vec1, vec2));
    }
}