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
    bool IntersectBoxes(const BoxShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outNormal, float &outDepth);
    bool IntersectCircleBox(const CircleShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotB, Vec2 &outNormal, float &outDepth);
    bool IntersectAABBs(const AABB &a, const AABB &b);

    Vec2 FindCirclesContactPoint(const CircleShape &a, Vec2 posA, Vec2 posB);
    Vec2 FindCircleBoxContactPoint(const CircleShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotB);
    int FindBoxesContactPoints(const BoxShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outContact1, Vec2 &outContact2);
}