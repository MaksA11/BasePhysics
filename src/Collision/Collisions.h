#pragma once

#include "../Core/Vec2.h"
#include "../Core/Math.h"
#include "Collider.h"

namespace bp::Collisions
{
    inline bool IntersectCircles(const CircleShape &a, const CircleShape &b, Vec2 posA, Vec2 posB, Vec2 &outNormal, float &outDepth)
    {
        outNormal = Vec2::Zero();
        outDepth = 0.0f;

        float distSq = math::DistanceSquared(posA, posB);
        float radiiSq = (a.radius + b.radius) * (a.radius + b.radius);

        if(distSq >= radiiSq)
            return false;

        outNormal = (posB - posA).Normalized();
        outDepth = radiiSq - distSq;

        return true;
    }
}