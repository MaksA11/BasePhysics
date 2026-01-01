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

    inline void ProjectVertices(Vec2 vertices[], Vec2 axis, float &outMin, float &outMax)
    {
        outMin = FLT_MAX;
        outMax = -FLT_MAX;

        for(int i = 0; i < 4; i++)
        {
            Vec2 vert = vertices[i];
            float proj = math::Dot(vert, axis);

            if(proj < outMin)
                outMin = proj;
            if(proj > outMax)
                outMax = proj;
        }
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

    inline bool IntersectBoxes(const BoxShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outNormal, float &outDepth)
    {
        outNormal = Vec2::Zero();
        outDepth = FLT_MAX;

        Vec2 *verts[2];
        verts[0] = GetBoxVertices(a, posA, rotA);
        verts[1] = GetBoxVertices(b, posB, rotB);
        
        for(int i = 0; i < 2; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                Vec2 vert1 = verts[i][j];
                Vec2 vert2 = verts[i][(j + 1) % 4];

                Vec2 edge = vert2 - vert1;
                Vec2 axis = Vec2(-edge.y, edge.x).Normalized();

                float min1, max1;
                float min2, max2;

                ProjectVertices(verts[0], axis, min1, max1);
                ProjectVertices(verts[1], axis, min2, max2);

                if(min1 >= max2 || min2 >= max1)
                {
                    delete[] verts[0];
                    delete[] verts[1];
                    
                    return false;
                }

                float axisDepth = fmin(max2 - min1, max1 - min2);
                if(axisDepth < outDepth)
                {
                    outDepth = axisDepth;
                    outNormal = axis;
                }
            }
        }

        Vec2 direction = posB - posA;
        if(math::Dot(direction, outNormal) < 0)
            outNormal = -outNormal;
        
        delete[] verts[0];
        delete[] verts[1];

        return true;
    }

    inline bool IntersectAABBs(const AABB &a, const AABB &b)
    {
        if(a.max.x <= b.min.x || b.max.x <= a.min.x || a.max.y <= b.min.y || b.max.y <= a.min.y)
            return false;

        return true;
    }
}