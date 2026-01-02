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
    inline void ProjectCircle(Vec2 center, float radius, Vec2 axis, float &outMin, float &outMax)
    {
        Vec2 direction = axis.Normalized();

        Vec2 p1 = center + (direction * radius);
        Vec2 p2 = center - (direction * radius);

        outMin = math::Dot(p1, axis);
        outMax = math::Dot(p2, axis);

        if(outMin > outMax)
            math::Swap(outMin, outMax);
    }

    inline int FindClosestPointIndex(Vec2 center, Vec2 vertices[])
    {
        int result = -1;
        float minDistSq = FLT_MAX;

        for(int i = 0; i < 4; i++)
        {
            Vec2 vert = vertices[i];
            float distSq = math::DistanceSquared(vert, center);

            if(distSq < minDistSq)
            {
                minDistSq = distSq;
                result = i;
            }
        }

        return result;
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
                Vec2 axis = Vec2(-edge.y, edge.x);
                axis.Normalize();

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
    inline bool IntersectCircleBox(const CircleShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotB, Vec2 &outNormal, float &outDepth)
    {
        outNormal = Vec2::Zero();
        outDepth = FLT_MAX;

        Vec2 *verts = GetBoxVertices(b, posB, rotB);
        
        Vec2 axis = Vec2::Zero();
        float axisDepth = 0;
        float min1, max1;
        float min2, max2;

        for(int i = 0; i < 4; i++)
        {
            Vec2 vert1 = verts[i];
            Vec2 vert2 = verts[(i + 1) % 4];

            Vec2 edge = vert2 - vert1;
            axis = Vec2(-edge.y, edge.x);
            axis.Normalize();

            ProjectVertices(verts, axis, min1, max1);
            ProjectCircle(posA, a.radius, axis, min2, max2);

            if(min1 >= max2 || min2 >= max1)
            {
                delete[] verts;

                return false;
            }

            float axisDepth = fmin(max2 - min1, max1 - min2);
            if(axisDepth < outDepth)
            {
                outDepth = axisDepth;
                outNormal = axis;
            }
        }

        int cpIndex = FindClosestPointIndex(posA, verts);
        Vec2 cp = verts[cpIndex];

        axis = cp - posA;
        axis.Normalize();

        ProjectVertices(verts, axis, min1, max1);
        ProjectCircle(posA, a.radius, axis, min2, max2);

        if(min1 >= max2 || min2 >= max1)
        {
            delete[] verts;

            return false;
        }

        axisDepth = fmin(max2 - min1, max1 - min2);
        if(axisDepth < outDepth)
        {
            outDepth = axisDepth;
            outNormal = axis;
        }

        Vec2 direction = posB - posA;

        if(math::Dot(direction, outNormal) < 0)
            outNormal = -outNormal;

        delete[] verts;

        return true;
    }

    inline bool IntersectAABBs(const AABB &a, const AABB &b)
    {
        if(a.max.x <= b.min.x || b.max.x <= a.min.x || a.max.y <= b.min.y || b.max.y <= a.min.y)
            return false;

        return true;
    }
}