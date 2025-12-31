#pragma once

#include "../Core/Vec2.h"
#include "../Core/Math.h"
#include "Collider.h"

namespace bp::collisions
{
    inline Vec2 *GetBoxVertices(const BoxShape &box, Vec2 position, float rotation)
    {
        float left = -box.size.x * 0.5f;
        float right = left + box.size.x;
        float bottom = -box.size.y * 0.5f;
        float top = bottom + box.size.y;

        Vec2 vertices[4];
        vertices[0] = Vec2(left, top);
        vertices[1] = Vec2(right, top);
        vertices[2] = Vec2(right, bottom);
        vertices[3] = Vec2(left, bottom);

        Vec2 *transformedVertices = new Vec2[4];

        for(int i = 0; i < 4; i++)
		{
			Vec2 vertex = vertices[i];
			transformedVertices[i] = math::Transform(vertex, position, rotation);
		}

        return transformedVertices;
    }

    inline bool IntersectCircles(const CircleShape &a, const CircleShape &b, Vec2 posA, Vec2 posB, Vec2 &outNormal, float &outDepth)
    {
        outNormal = Vec2::Zero();
        outDepth = 0.0f;

        float distSq = math::DistanceSquared(posA, posB);
        float dist = std::sqrt(distSq);

        float radii = a.radius + b.radius;
        float radiiSq = radii * radii;

        if(distSq >= radiiSq)
            return false;

        outNormal = (posB - posA).Normalized();
        outDepth = radii - dist;

        return true;
    }

    inline bool IntersectBoxes()
    {

    }

    inline bool IntersectAABBs(const AABB &a, const AABB &b)
    {
        if(a.max.x <= b.min.x || b.max.x <= a.min.x || a.max.y <= b.min.y || b.max.y <= a.min.y)
            return false;

        return true;
    }
}