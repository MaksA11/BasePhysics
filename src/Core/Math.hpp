#pragma once

#include "Vec2.hpp"

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
    inline Vec2 Perpendicular(Vec2 vec)
    {
        return Vec2(-vec.y, vec.x);
    }

    inline Vec2 Transform(const Vec2 &vertexPosition, Vec2 position, float angle)
    {
        float cosA = std::cos(angle);
        float sinA = std::sin(angle);

        float x = cosA * vertexPosition.x - sinA * vertexPosition.y + position.x;
        float y = sinA * vertexPosition.x + cosA * vertexPosition.y + position.y;

        return Vec2(x, y);
    }
    inline Vec2 Rotate(const Vec2 &vertexPosition, float angle)
    {
        float cosA = std::cos(angle);
        float sinA = std::sin(angle);

        float x = vertexPosition.x * cosA - vertexPosition.y * sinA;
        float y = vertexPosition.x * sinA + vertexPosition.y * cosA;

        return Vec2(x, y);
    }
}