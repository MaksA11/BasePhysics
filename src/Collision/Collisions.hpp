#pragma once

#include "../Core/Vec2.hpp"
#include "../Core/Math.hpp"
#include "../Physics/Rigidbody.hpp"
#include "Collider.hpp"

namespace bp::collisions
{
    Vec2 *GetBoxVertices(const BoxShape &box, Vec2 position, float rotation);
    
    bool Collide(Rigidbody *bodyA, Rigidbody *bodyB, Vec2 &outNormal, float &outDepth, std::vector<Vec2> &outContacts);

    void ProjectVertices(Vec2 vertices[], Vec2 axis, float &outMin, float &outMax);
    void ProjectCircle(Vec2 center, float radius, Vec2 axis, float &outMin, float &outMax);

    int FindClosestPointIndex(Vec2 center, Vec2 vertices[]);

    bool IntersectCircles(const CircleShape &a, const CircleShape &b, Vec2 posA, Vec2 posB, Vec2 &outNormal, float &outDepth);
    bool IntersectBoxes(const BoxShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outNormal, float &outDepth);
    bool IntersectCircleBox(const CircleShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotB, Vec2 &outNormal, float &outDepth);
    bool IntersectAABBs(const AABB &a, const AABB &b);

    Vec2 FindCirclesContactPoint(const CircleShape &a, Vec2 posA, Vec2 posB);
    Vec2 FindCircleBoxContactPoint(const CircleShape &a, Vec2 posA, Vec2 posB);
    std::vector<Vec2> FindBoxesContactPoints(const CircleShape &a, Vec2 posA, Vec2 posB);
}