#pragma once

#include <vector>

#include "../../Core/Vec2.hpp"

namespace bp
{
    struct ContactManifold
    {
        int rbIndex1, rbIndex2;
        Vec2 normal;
        float depth;
        std::vector<Vec2> contactPoints;

        ContactManifold() : rbIndex1(-1), rbIndex2(-1), normal(Vec2::Zero()), depth(0.0f) {}
        
        ContactManifold(int rbIndex1, int rbIndex2, Vec2 normal, float depth, Vec2 contactPoint1)
            : rbIndex1(rbIndex1), rbIndex2(rbIndex2), normal(normal), depth(depth)
        {
            contactPoints.push_back(contactPoint1);
        }
        ContactManifold(int rbIndex1, int rbIndex2, Vec2 normal, float depth, Vec2 contactPoint1, Vec2 contactPoint2)
            : rbIndex1(rbIndex1), rbIndex2(rbIndex2), normal(normal), depth(depth)
        {
            contactPoints.push_back(contactPoint1);
            contactPoints.push_back(contactPoint2);
        }
    };
}