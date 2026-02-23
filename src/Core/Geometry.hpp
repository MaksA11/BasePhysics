#pragma once

#include <vector>
#include <utility>
#include <limits>

#include "Vec2.hpp"
#include "Math.hpp"

namespace bp::geometry
{
    inline float PointSegmentDistance(Vec2 point, Vec2 segmentA, Vec2 segmentB, Vec2 &outClosestPoint)
    {
        Vec2 segment = segmentB - segmentA;
        Vec2 startToPoint = point - segmentA;

        float proj = math::Dot(startToPoint, segment);
        float lengthSquared = segment.MagnitudeSquared();
        
        if(lengthSquared < 0.00001f)
        {
            outClosestPoint = segmentA;
            return math::DistanceSquared(point, segmentA);
        }

        float d = proj / lengthSquared;

        if(d <= 0)
            outClosestPoint = segmentA;
        else if(d >= 1)
            outClosestPoint = segmentB;
        else
            outClosestPoint = segmentA + segment * d;

        return math::DistanceSquared(point, outClosestPoint);
    }

    inline int FindClosestPointIndex(Vec2 center, const std::vector<Vec2> &vertices)
    {
        int result = -1;
        float minDistSq = std::numeric_limits<float>::max();

        for(int i = 0; i < vertices.size(); i++)
        {
            float distSq = math::DistanceSquared(vertices[i], center);
            if(distSq < minDistSq)
            {
                minDistSq = distSq;
                result = i;
            }
        }
        return result;
    }

    inline void ProjectVertices(const std::vector<Vec2> &vertices, Vec2 axis, float &outMin, float &outMax)
    {
        outMin = std::numeric_limits<float>::max();
        outMax = -std::numeric_limits<float>::max();

        for(const Vec2 &vert : vertices)
        {
            float proj = math::Dot(vert, axis);
            if(proj < outMin)
                outMin = proj;
            if(proj > outMax)
                outMax = proj;
        }
    }

    inline void ProjectCircle(Vec2 center, float radius, Vec2 axis, float &outMin, float &outMax)
    {
        float proj = math::Dot(center, axis);

        outMin = proj - radius;
        outMax = proj + radius;
    }
}