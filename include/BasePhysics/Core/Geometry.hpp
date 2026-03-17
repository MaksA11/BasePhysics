#pragma once

#include <vector>
#include <utility>
#include <limits>

#include <BasePhysics/Core/Vec2.hpp>
#include <BasePhysics/Core/Math.hpp>

namespace bp::geometry
{
    float PointSegmentDistance(Vec2 point, Vec2 segmentA, Vec2 segmentB, Vec2 &outClosestPoint);
    int FindClosestPointIndex(Vec2 center, const std::vector<Vec2> &vertices);
    void ProjectVertices(const std::vector<Vec2> &vertices, Vec2 axis, float &outMin, float &outMax);
    void ProjectCircle(Vec2 center, float radius, Vec2 axis, float &outMin, float &outMax);
}