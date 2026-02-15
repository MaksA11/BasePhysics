#include "Collisions.hpp"

namespace bp::collisions
{
    bool Collide(Rigidbody *bodyA, Rigidbody *bodyB, Vec2 &outNormal, float &outDepth, std::vector<Vec2> &outContacts)
    {
        outNormal = Vec2::Zero();
        outDepth = 0.0f;
        
        if(bodyA->GetCollider().IsCircle())
        {
            if(bodyB->GetCollider().IsCircle())
            {
                if(collisions::IntersectCircles(*bodyA->GetCollider().GetCircle(), *bodyB->GetCollider().GetCircle(), bodyA->GetPosition(), bodyB->GetPosition(), outNormal, outDepth))
                {
                    outContacts.push_back(FindCirclesContactPoint(*bodyA->GetCollider().GetCircle(), bodyA->GetPosition(), bodyB->GetPosition()));
                    return true;
                }
            }
            else if(bodyB->GetCollider().IsBox())
            {
                if(collisions::IntersectCirclePolygon(*bodyA->GetCollider().GetCircle(), bodyB->GetCollider().GetBox()->ToPolygon(), bodyA->GetPosition(), bodyB->GetPosition(),
                    bodyB->GetRotation(), outNormal, outDepth))
                {
                    outContacts.push_back(FindCirclePolygonContactPoint(*bodyA->GetCollider().GetCircle(), bodyB->GetCollider().GetBox()->ToPolygon(),
                        bodyA->GetPosition(), bodyB->GetPosition(), bodyB->GetRotation()));
                    return true;
                }
            }
            else if(bodyB->GetCollider().IsPolygon())
            {
                if(collisions::IntersectCirclePolygon(*bodyA->GetCollider().GetCircle(), *bodyB->GetCollider().GetPolygon(), bodyA->GetPosition(), bodyB->GetPosition(),
                    bodyB->GetRotation(), outNormal, outDepth))
                {
                    outContacts.push_back(FindCirclePolygonContactPoint(*bodyA->GetCollider().GetCircle(), *bodyB->GetCollider().GetPolygon(),
                        bodyA->GetPosition(), bodyB->GetPosition(), bodyB->GetRotation()));
                    return true;
                }
            }
        }
        else if(bodyA->GetCollider().IsBox())
        {
            if(bodyB->GetCollider().IsBox())
            {
                if(collisions::IntersectPolygons(bodyA->GetCollider().GetBox()->ToPolygon(), bodyB->GetCollider().GetBox()->ToPolygon(), bodyA->GetPosition(), bodyB->GetPosition(),
                    bodyA->GetRotation(), bodyB->GetRotation(), outNormal, outDepth))
                {
                    Vec2 contact1, contact2;
                    int contactCount = FindPolygonsContactPoints(bodyA->GetCollider().GetBox()->ToPolygon(), bodyB->GetCollider().GetBox()->ToPolygon(), bodyA->GetPosition(),
                        bodyB->GetPosition(), bodyA->GetRotation(), bodyB->GetRotation(), contact1, contact2);
                    outContacts.push_back(contact1);
                    if(contactCount == 2)
                        outContacts.push_back(contact2);
                    return true;
                }
            }
            else if(bodyB->GetCollider().IsCircle())
            {
                if(collisions::IntersectCirclePolygon(*bodyB->GetCollider().GetCircle(), bodyA->GetCollider().GetBox()->ToPolygon(), bodyB->GetPosition(), bodyA->GetPosition(),
                    bodyA->GetRotation(), outNormal, outDepth))
                {
                    outNormal = -outNormal;
                    outContacts.push_back(FindCirclePolygonContactPoint(*bodyB->GetCollider().GetCircle(), bodyA->GetCollider().GetBox()->ToPolygon(),
                        bodyB->GetPosition(), bodyA->GetPosition(), bodyA->GetRotation()));
                    return true;
                }
            }
            else if(bodyB->GetCollider().IsPolygon())
            {
                if(collisions::IntersectPolygons(bodyA->GetCollider().GetBox()->ToPolygon(), *bodyB->GetCollider().GetPolygon(), bodyA->GetPosition(), bodyB->GetPosition(),
                    bodyA->GetRotation(), bodyB->GetRotation(), outNormal, outDepth))
                {
                    Vec2 contact1, contact2;
                    int contactCount = FindPolygonsContactPoints(bodyA->GetCollider().GetBox()->ToPolygon(), *bodyB->GetCollider().GetPolygon(), bodyA->GetPosition(),
                        bodyB->GetPosition(), bodyA->GetRotation(), bodyB->GetRotation(), contact1, contact2);
                    outContacts.push_back(contact1);
                    if(contactCount == 2)
                        outContacts.push_back(contact2);
                    return true;
                }
            }
        }
        else if(bodyA->GetCollider().IsPolygon())
        {
            if(bodyB->GetCollider().IsPolygon())
            {
                if(collisions::IntersectPolygons(*bodyA->GetCollider().GetPolygon(), *bodyB->GetCollider().GetPolygon(), bodyA->GetPosition(), bodyB->GetPosition(),
                    bodyA->GetRotation(), bodyB->GetRotation(), outNormal, outDepth))
                {
                    Vec2 contact1, contact2;
                    int contactCount = FindPolygonsContactPoints(*bodyA->GetCollider().GetPolygon(), *bodyB->GetCollider().GetPolygon(), bodyA->GetPosition(),
                        bodyB->GetPosition(), bodyA->GetRotation(), bodyB->GetRotation(), contact1, contact2);
                    outContacts.push_back(contact1);
                    if(contactCount == 2)
                        outContacts.push_back(contact2);
                    return true;
                }
            }
            else if(bodyB->GetCollider().IsCircle())
            {
                if(collisions::IntersectCirclePolygon(*bodyB->GetCollider().GetCircle(), *bodyA->GetCollider().GetPolygon(), bodyB->GetPosition(), bodyA->GetPosition(),
                    bodyA->GetRotation(), outNormal, outDepth))
                {
                    outNormal = -outNormal;
                    outContacts.push_back(FindCirclePolygonContactPoint(*bodyB->GetCollider().GetCircle(), *bodyA->GetCollider().GetPolygon(),
                        bodyB->GetPosition(), bodyA->GetPosition(), bodyA->GetRotation()));
                    return true;
                }
            }
            else if(bodyB->GetCollider().IsBox())
            {
                if(collisions::IntersectPolygons(*bodyA->GetCollider().GetPolygon(), bodyB->GetCollider().GetBox()->ToPolygon(), bodyA->GetPosition(), bodyB->GetPosition(),
                    bodyA->GetRotation(), bodyB->GetRotation(), outNormal, outDepth))
                {
                    Vec2 contact1, contact2;
                    int contactCount = FindPolygonsContactPoints(*bodyA->GetCollider().GetPolygon(), bodyB->GetCollider().GetBox()->ToPolygon(), bodyA->GetPosition(),
                        bodyB->GetPosition(), bodyA->GetRotation(), bodyB->GetRotation(), contact1, contact2);
                    outContacts.push_back(contact1);
                    if(contactCount == 2)
                        outContacts.push_back(contact2);
                    return true;
                }
            }
        }

        return false;
    }
    bool Collide(Rigidbody *bodyA, Rigidbody *bodyB)
    {
        Vec2 normal;
        float depth;
        std::vector<Vec2> contacts;

        return Collide(bodyA, bodyB, normal, depth, contacts);
    }

    bool IntersectCircles(const CircleShape &a, const CircleShape &b, Vec2 posA, Vec2 posB, Vec2 &outNormal, float &outDepth)
    {
        outNormal = Vec2::Zero();
        outDepth = 0.0f;

        float dist = math::Distance(posA, posB);
        float radii = a.radius + b.radius;

        if(dist >= radii)
            return false;

        outNormal = (posB - posA).Normalized();
        outDepth = radii - dist;

        return true;
    }
    bool IntersectPolygons(const PolygonShape &a, const PolygonShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outNormal, float &outDepth)
    {
        outNormal = Vec2::Zero();
        outDepth = std::numeric_limits<float>::max();

        const std::vector<Vec2> *verts[2] = { &a.worldVertices, &b.worldVertices };
        const std::vector<Vec2> *axes[2] = { &a.worldNormals, &b.worldNormals };
        
        for(int i = 0; i < 2; i++)
        {
            for(int j = 0; j < axes[i]->size(); j++)
            {
                Vec2 axis = (*axes[i])[j];

                float min1, max1;
                float min2, max2;

                geometry::ProjectVertices(*verts[0], axis, min1, max1);
                geometry::ProjectVertices(*verts[1], axis, min2, max2);

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
    bool IntersectCirclePolygon(const CircleShape &a, const PolygonShape &b, Vec2 posA, Vec2 posB, float rotB, Vec2 &outNormal, float &outDepth)
    {
        outNormal = Vec2::Zero();
        outDepth = std::numeric_limits<float>::max();

        const std::vector<Vec2> &verts = b.worldVertices;
        const std::vector<Vec2> &axes = b.worldNormals;

        for(int i = 0; i < axes.size(); i++)
        {
            Vec2 axis = axes[i];
            float min1, max1;
            float min2, max2;

            geometry::ProjectVertices(verts, axis, min1, max1);
            geometry::ProjectCircle(posA, a.radius, axis, min2, max2);

            if(min1 >= max2 || min2 >= max1)
                return false;

            float axisDepth = std::min(max2 - min1, max1 - min2);
            if(axisDepth < outDepth)
            {
                outDepth = axisDepth;
                outNormal = axis;
            }
        }

        int cpIndex = geometry::FindClosestPointIndex(posA, verts);
        Vec2 cp = verts[cpIndex];

        Vec2 axis = cp - posA;
        axis.Normalize();
        float min1, max1;
        float min2, max2;

        geometry::ProjectVertices(verts, axis, min1, max1);
        geometry::ProjectCircle(posA, a.radius, axis, min2, max2);

        if(min1 >= max2 || min2 >= max1)
            return false;

        float axisDepth = std::min(max2 - min1, max1 - min2);
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
    Vec2 FindCirclePolygonContactPoint(const CircleShape &a, const PolygonShape &b, Vec2 posA, Vec2 posB, float rotB)
    {
        float minDistSq = std::numeric_limits<float>::max();
        Vec2 contact = Vec2::Zero();

        std::vector<Vec2> verts = b.worldVertices;

        for(int i = 0; i < verts.size(); i++)
        {
            Vec2 vert1 = verts[i];
            Vec2 vert2 = verts[(i + 1) % verts.size()];

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
    int FindPolygonsContactPoints(const PolygonShape &a, const PolygonShape &b, Vec2 posA, Vec2 posB, float rotA, float rotB, Vec2 &outContact1, Vec2 &outContact2)
    {
        int contactCount = 0;
        outContact1 = Vec2::Zero();
        outContact2 = Vec2::Zero();
        
        float minDistSq = std::numeric_limits<float>::max();

        std::vector<Vec2> vertsA = a.worldVertices;
        std::vector<Vec2> vertsB = b.worldVertices;

        for(int i = 0; i < vertsA.size(); i++)
        {
            Vec2 vert = vertsA[i];

            for(int j = 0; j < vertsB.size(); j++)
            {
                Vec2 vert1 = vertsB[j];
                Vec2 vert2 = vertsB[(j + 1) % vertsB.size()];

                Vec2 cp;
                float distanceSq = geometry::PointSegmentDistance(vert, vert1, vert2, cp);

                if(utils::NearlyEqual(distanceSq, minDistSq))
                {
                    if(!utils::NearlyEqual(cp, outContact1))
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
        for(int i = 0; i < vertsB.size(); i++)
        {
            Vec2 vert = vertsB[i];

            for(int j = 0; j < vertsA.size(); j++)
            {
                Vec2 vert1 = vertsA[j];
                Vec2 vert2 = vertsA[(j + 1) % vertsA.size()];

                Vec2 cp;
                float distanceSq = geometry::PointSegmentDistance(vert, vert1, vert2, cp);

                if(utils::NearlyEqual(distanceSq, minDistSq))
                {
                    if(!utils::NearlyEqual(cp, outContact1))
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