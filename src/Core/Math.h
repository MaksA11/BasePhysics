#pragma once

#include "Vec2.h"

namespace bp::math
{
    constexpr float pi = 3.141592653589793f;

    inline float ToDegrees(float angle)
    {
        return angle * 180.0f / pi;
    }
    inline float ToRadians(float angle)
    {
        return angle * pi / 180.0f;
    }

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

    inline Vec2 Transform(Vec2 vertexPosition, Vec2 position, float angle)
    {
        float cosA = cos(angle);
        float sinA = sin(angle);

        float x = cosA * vertexPosition.x + sinA * vertexPosition.y + position.x;
        float y = -sinA * vertexPosition.x + cosA * vertexPosition.y + position.y;

        return Vec2(x, y);
    }
}