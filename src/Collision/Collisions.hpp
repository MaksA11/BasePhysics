#pragma once

#include "../Core/Vec2.hpp"
#include "../Core/Math.hpp"
#include "../Core/Geometry.hpp"
#include "../Physics/Rigidbody.hpp"
#include "Collider.hpp"

namespace bp::collisions
{   
    bool Collide(Rigidbody *bodyA, Rigidbody *bodyB, Vec2 &outNormal, float &outDepth, std::vector<Vec2> &outContacts);

    bool IntersectCircles(const CircleShape &a, const CircleShape &b, Vec2 posA, Vec2 posB, Vec2 &outNormal, float &outDepth);
    bool IntersectPolygons(const PolygonShape &a, const PolygonShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outNormal, float &outDepth);
    bool IntersectCirclePolygon(const CircleShape &a, const PolygonShape &b, Vec2 posA, Vec2 posB, float rotB, Vec2 &outNormal, float &outDepth);
    bool IntersectAABBs(const AABB &a, const AABB &b);

    Vec2 FindCirclesContactPoint(const CircleShape &a, Vec2 posA, Vec2 posB);
    Vec2 FindCirclePolygonContactPoint(const CircleShape &a, const PolygonShape &b, Vec2 posA, Vec2 posB, float rotB);
    int FindPolygonsContactPoints(const PolygonShape &a, const PolygonShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outContact1, Vec2 &outContact2);
}