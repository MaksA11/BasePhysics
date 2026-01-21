#include "Collisions.hpp"

namespace bp::collisions
{
    bool Collide(Rigidbody *bodyA, Rigidbody *bodyB, Vec2 &outNormal, float &outDepth, std::vector<Vec2> &outContacts)
    {
        outNormal = Vec2::Zero();
        outDepth = 0.0f;

        if(bodyA->GetCollider().IsCircle() && bodyB->GetCollider().IsCircle())
        {
            if(collisions::IntersectCircles(*bodyA->GetCollider().GetCircle(), *bodyB->GetCollider().GetCircle(), bodyA->GetPosition(), bodyB->GetPosition(), outNormal, outDepth))
            {
                outContacts.push_back(FindCirclesContactPoint(*bodyA->GetCollider().GetCircle(), bodyA->GetPosition(), bodyB->GetPosition()));
                return true;
            }
        }
        else if(bodyA->GetCollider().IsBox() && bodyB->GetCollider().IsBox())
        {
            if(collisions::IntersectBoxes(*bodyA->GetCollider().GetBox(), *bodyB->GetCollider().GetBox(), bodyA->GetPosition(), bodyB->GetPosition(),
                bodyA->GetRotation(), bodyB->GetRotation(), outNormal, outDepth))
            {
                Vec2 contact1, contact2;
                int contactCount = FindBoxesContactPoints(*bodyA->GetCollider().GetBox(), *bodyB->GetCollider().GetBox(), bodyA->GetPosition(), bodyB->GetPosition(),
                    bodyA->GetRotation(), bodyB->GetRotation(), contact1, contact2);
                outContacts.push_back(contact1);
                if(contactCount == 2)
                    outContacts.push_back(contact2);
                return true;
            }
        }
        else if(bodyA->GetCollider().IsCircle() && bodyB->GetCollider().IsBox())
        {
            if(collisions::IntersectCircleBox(*bodyA->GetCollider().GetCircle(), *bodyB->GetCollider().GetBox(), bodyA->GetPosition(), bodyB->GetPosition(),
                bodyB->GetRotation(), outNormal, outDepth))
            {
                outContacts.push_back(FindCircleBoxContactPoint(*bodyA->GetCollider().GetCircle(), *bodyB->GetCollider().GetBox(),
                    bodyA->GetPosition(), bodyB->GetPosition(), bodyB->GetRotation()));
                return true;
            }
        }
        else if(bodyA->GetCollider().IsBox() && bodyB->GetCollider().IsCircle())
        {
            if(collisions::IntersectCircleBox(*bodyB->GetCollider().GetCircle(), *bodyA->GetCollider().GetBox(), bodyB->GetPosition(), bodyA->GetPosition(),
                bodyA->GetRotation(), outNormal, outDepth))
            {
                outNormal = -outNormal;
                outContacts.push_back(FindCircleBoxContactPoint(*bodyB->GetCollider().GetCircle(), *bodyA->GetCollider().GetBox(),
                    bodyB->GetPosition(), bodyA->GetPosition(), bodyA->GetRotation()));
                return true;
            }
        }

        return false;
    }

    bool IntersectCircles(const CircleShape &a, const CircleShape &b, Vec2 posA, Vec2 posB, Vec2 &outNormal, float &outDepth)
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
    bool IntersectBoxes(const BoxShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outNormal, float &outDepth)
    {
        outNormal = Vec2::Zero();
        outDepth = FLT_MAX;

        std::array<Vec2, 4> verts[2];
        verts[0] = geometry::GetBoxVertices(a, posA, rotA);
        verts[1] = geometry::GetBoxVertices(b, posB, rotB);
        
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

                geometry::ProjectVertices(verts[0], axis, min1, max1);
                geometry::ProjectVertices(verts[1], axis, min2, max2);

                if(min1 >= max2 || min2 >= max1)
                    return false;

                float axisDepth = std::min(max2 - min1, max1 - min2);
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

        return true;
    }
    bool IntersectCircleBox(const CircleShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotB, Vec2 &outNormal, float &outDepth)
    {
        outNormal = Vec2::Zero();
        outDepth = FLT_MAX;

        std::array<Vec2, 4> verts = geometry::GetBoxVertices(b, posB, rotB);
        
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

            geometry::ProjectVertices(verts, axis, min1, max1);
            geometry::ProjectCircle(posA, a.radius, axis, min2, max2);

            if(min1 >= max2 || min2 >= max1)
                return false;

            float axisDepth = fmin(max2 - min1, max1 - min2);
            if(axisDepth < outDepth)
            {
                outDepth = axisDepth;
                outNormal = axis;
            }
        }

        int cpIndex = geometry::FindClosestPointIndex(posA, verts);
        Vec2 cp = verts[cpIndex];

        axis = cp - posA;
        axis.Normalize();

        geometry::ProjectVertices(verts, axis, min1, max1);
        geometry::ProjectCircle(posA, a.radius, axis, min2, max2);

        if(min1 >= max2 || min2 >= max1)
            return false;

        axisDepth = fmin(max2 - min1, max1 - min2);
        if(axisDepth < outDepth)
        {
            outDepth = axisDepth;
            outNormal = axis;
        }

        Vec2 direction = posB - posA;

        if(math::Dot(direction, outNormal) < 0)
            outNormal = -outNormal;

        return true;
    }
    bool IntersectAABBs(const AABB &a, const AABB &b)
    {
        if(a.max.x <= b.min.x || b.max.x <= a.min.x || a.max.y <= b.min.y || b.max.y <= a.min.y)
            return false;

        return true;
    }

    Vec2 FindCirclesContactPoint(const CircleShape &a, Vec2 posA, Vec2 posB)
    {
        Vec2 direction = posB - posA;
        direction.Normalize();
        return posA + direction * a.radius;
    }
    Vec2 FindCircleBoxContactPoint(const CircleShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotB)
    {
        float minDistSq = FLT_MAX;
        Vec2 contact = Vec2::Zero();

        std::array<Vec2, 4> verts = geometry::GetBoxVertices(b, posB, rotB);

        for(int i = 0; i < 4; i++)
        {
            Vec2 vert1 = verts[i];
            Vec2 vert2 = verts[(i + 1) % 4];

            Vec2 cp;
            float distanceSq = geometry::PointSegmentDistance(posA, vert1, vert2, cp);

            if(distanceSq < minDistSq)
            {
                minDistSq = distanceSq;
                contact = cp;
            }
        }

        return contact;
    }
    int FindBoxesContactPoints(const BoxShape &a, const BoxShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outContact1, Vec2 &outContact2)
    {
        int contactCount = 0;
        outContact1 = Vec2::Zero();
        outContact2 = Vec2::Zero();
        
        float minDistSq = FLT_MAX;

        std::array<Vec2, 4> vertsA = geometry::GetBoxVertices(a, posA, rotA);
        std::array<Vec2, 4> vertsB = geometry::GetBoxVertices(b, posB, rotB);

        for(int i = 0; i < 4; i++)
        {
            Vec2 vert = vertsA[i];

            for(int j = 0; j < 4; j++)
            {
                Vec2 vert1 = vertsB[j];
                Vec2 vert2 = vertsB[(j + 1) % 4];

                Vec2 cp;
                float distanceSq = geometry::PointSegmentDistance(vert, vert1, vert2, cp);

                if(math::NearlyEqual(distanceSq, minDistSq))
                {
                    if(!math::NearlyEqual(cp, outContact1))
                    {
                        contactCount = 2;
                        outContact2 = cp;
                    }
                }
                else if(distanceSq < minDistSq)
                {
                    minDistSq = distanceSq;
                    contactCount = 1;
                    outContact1 = cp;
                }
            }
        }
        for(int i = 0; i < 4; i++)
        {
            Vec2 vert = vertsB[i];

            for(int j = 0; j < 4; j++)
            {
                Vec2 vert1 = vertsA[j];
                Vec2 vert2 = vertsA[(j + 1) % 4];

                Vec2 cp;
                float distanceSq = geometry::PointSegmentDistance(vert, vert1, vert2, cp);

                if(math::NearlyEqual(distanceSq, minDistSq))
                {
                    if(!math::NearlyEqual(cp, outContact1))
                    {
                        contactCount = 2;
                        outContact2 = cp;
                    }
                }
                else if(distanceSq < minDistSq)
                {
                    minDistSq = distanceSq;
                    contactCount = 1;
                    outContact1 = cp;
                }
            }
        }

        return contactCount;
    }
}