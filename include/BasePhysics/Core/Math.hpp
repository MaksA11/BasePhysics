#pragma once

#include <BasePhysics/Core/Vec2.hpp>

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

    inline float NormalizeAngle(float angle)
    {
        while(angle > pi)
            angle -= 2.0f * pi;
        while(angle < -pi)
            angle += 2.0f * pi;

        return angle;
    }

    inline float Dot(const Vec2 &vector1, const Vec2 &vector2)
    {
        return (vector1.x * vector2.x) + (vector1.y * vector2.y);
    }
    inline float Cross(const Vec2 &vector1, const Vec2 &vector2)
    {
        return (vector1.x * vector2.y) - (vector1.y * vector2.x);
    }
    inline float DistanceSquared(const Vec2 &vector1, const Vec2 &vector2)
    {
        return (vector1.x - vector2.x) * (vector1.x - vector2.x) + (vector1.y - vector2.y) * (vector1.y - vector2.y);
    }
    inline float Distance(const Vec2 &vector1, const Vec2 &vector2)
    {
        return std::sqrt(DistanceSquared(vector1, vector2));
    }
    inline Vec2 Perpendicular(Vec2 vector)
    {
        return Vec2(-vector.y, vector.x);
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