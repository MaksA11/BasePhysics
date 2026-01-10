#pragma once

#include "Vec2.hpp"
#include "Math.hpp"
#include "../Collision/ColliderShapes.hpp"

namespace bp::geometry
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

    inline float PointSegmentDistance(Vec2 point, Vec2 segmentA, Vec2 segmentB, Vec2 &outClosestPoint)
    {
        Vec2 segment = segmentB - segmentA;
        Vec2 startToPoint = point - segmentA;

        float proj = math::Dot(startToPoint, segment);
        float lengthSquared = segment.MagnitudeSquared();
        float d = proj / lengthSquared;

        if(d <= 0)
            outClosestPoint = segmentA;
        else if(d >= 1)
            outClosestPoint = segmentB;
        else
            outClosestPoint = segmentA + segment * d;

        return math::DistanceSquared(point, outClosestPoint);
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
}