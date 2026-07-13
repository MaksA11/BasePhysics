#pragma once

#include <vector>
#include <limits>

#include <BasePhysics/Core/Vec2.hpp>

namespace bp
{
    struct ContactManifold
    {
        size_t rbIndex1, rbIndex2;
        Vec2 normal;
        float depth;
        
        size_t contactCount;
        Vec2 contactPoint1, contactPoint2;

        ContactManifold() : rbIndex1(std::numeric_limits<size_t>::max()), rbIndex2(std::numeric_limits<size_t>::max()),
            normal(Vec2::Zero()), depth(0.0f), contactCount(0), contactPoint1(Vec2::Zero()), contactPoint2(Vec2::Zero()) {}
        
        ContactManifold(size_t rbIndex1, size_t rbIndex2, Vec2 normal, float depth, Vec2 contactPoint1)
            : rbIndex1(rbIndex1), rbIndex2(rbIndex2), normal(normal), depth(depth), contactCount(1), contactPoint1(contactPoint1), contactPoint2(Vec2::Zero()) {}
        ContactManifold(size_t rbIndex1, size_t rbIndex2, Vec2 normal, float depth, Vec2 contactPoint1, Vec2 contactPoint2)
            : rbIndex1(rbIndex1), rbIndex2(rbIndex2), normal(normal), depth(depth), contactCount(2), contactPoint1(contactPoint1), contactPoint2(contactPoint2) {}
    };
}