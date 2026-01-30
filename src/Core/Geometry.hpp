#pragma once

#include <array>
#include <utility>

#include "Vec2.hpp"
#include "Math.hpp"
#include "../Collision/ColliderShapes.hpp"

namespace bp::geometry
{
    inline std::vector<Vec2> TransformPolygonVertices(std::vector<Vec2> vertices, Vec2 position, float rotation)
    {
        std::vector<Vec2> transformedVertices = std::vector<Vec2>(vertices.size());

        for(int i = 0; i < vertices.size(); i++)
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
    inline int FindClosestPointIndex(Vec2 center, std::vector<Vec2> vertices)
    {
        int result = -1;
        float minDistSq = FLT_MAX;

        for(int i = 0; i < vertices.size(); i++)
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

    inline void ProjectVertices(std::vector<Vec2> vertices, Vec2 axis, float &outMin, float &outMax)
    {
        outMin = FLT_MAX;
        outMax = -FLT_MAX;

        for(int i = 0; i < vertices.size(); i++)
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
            std::swap(outMin, outMax);
    }
}