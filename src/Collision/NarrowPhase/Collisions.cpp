#include <BasePhysics/Collision/NarrowPhase/Collisions.hpp>

namespace bp::collisions
{
    bool Collide(Rigidbody *rb1, Rigidbody *rb2, Vec2 &outNormal, float &outDepth, std::vector<Vec2> &outContacts)
    {
        outNormal = Vec2::Zero();
        outDepth = 0.0f;

        if(rb1 == rb2)
            return false;
        
        if(rb1->GetCollider().IsCircle())
        {
            if(rb2->GetCollider().IsCircle())
            {
                if(collisions::IntersectCircles(*rb1->GetCollider().GetCircle(), *rb2->GetCollider().GetCircle(), rb1->GetPosition(), rb2->GetPosition(), outNormal, outDepth))
                {
                    outContacts.push_back(FindCirclesContactPoint(*rb1->GetCollider().GetCircle(), rb1->GetPosition(), rb2->GetPosition()));
                    return true;
                }
            }
            else if(rb2->GetCollider().IsBox())
            {
                if(collisions::IntersectCirclePolygon(*rb1->GetCollider().GetCircle(), rb2->GetCollider().GetBox()->ToPolygon(), rb1->GetPosition(), rb2->GetPosition(),
                    rb2->GetRotation(), outNormal, outDepth))
                {
                    outContacts.push_back(FindCirclePolygonContactPoint(*rb1->GetCollider().GetCircle(), rb2->GetCollider().GetBox()->ToPolygon(),
                        rb1->GetPosition(), rb2->GetPosition(), rb2->GetRotation()));
                    return true;
                }
            }
            else if(rb2->GetCollider().IsPolygon())
            {
                if(collisions::IntersectCirclePolygon(*rb1->GetCollider().GetCircle(), *rb2->GetCollider().GetPolygon(), rb1->GetPosition(), rb2->GetPosition(),
                    rb2->GetRotation(), outNormal, outDepth))
                {
                    outContacts.push_back(FindCirclePolygonContactPoint(*rb1->GetCollider().GetCircle(), *rb2->GetCollider().GetPolygon(),
                        rb1->GetPosition(), rb2->GetPosition(), rb2->GetRotation()));
                    return true;
                }
            }
        }
        else if(rb1->GetCollider().IsBox())
        {
            if(rb2->GetCollider().IsBox())
            {
                if(collisions::IntersectPolygons(rb1->GetCollider().GetBox()->ToPolygon(), rb2->GetCollider().GetBox()->ToPolygon(), rb1->GetPosition(), rb2->GetPosition(),
                    rb1->GetRotation(), rb2->GetRotation(), outNormal, outDepth))
                {
                    Vec2 contact1, contact2;
                    int contactCount = FindPolygonsContactPoints(rb1->GetCollider().GetBox()->ToPolygon(), rb2->GetCollider().GetBox()->ToPolygon(), rb1->GetPosition(),
                        rb2->GetPosition(), rb1->GetRotation(), rb2->GetRotation(), contact1, contact2);
                    outContacts.push_back(contact1);
                    if(contactCount == 2)
                        outContacts.push_back(contact2);
                    return true;
                }
            }
            else if(rb2->GetCollider().IsCircle())
            {
                if(collisions::IntersectCirclePolygon(*rb2->GetCollider().GetCircle(), rb1->GetCollider().GetBox()->ToPolygon(), rb2->GetPosition(), rb1->GetPosition(),
                    rb1->GetRotation(), outNormal, outDepth))
                {
                    outNormal = -outNormal;
                    outContacts.push_back(FindCirclePolygonContactPoint(*rb2->GetCollider().GetCircle(), rb1->GetCollider().GetBox()->ToPolygon(),
                        rb2->GetPosition(), rb1->GetPosition(), rb1->GetRotation()));
                    return true;
                }
            }
            else if(rb2->GetCollider().IsPolygon())
            {
                if(collisions::IntersectPolygons(rb1->GetCollider().GetBox()->ToPolygon(), *rb2->GetCollider().GetPolygon(), rb1->GetPosition(), rb2->GetPosition(),
                    rb1->GetRotation(), rb2->GetRotation(), outNormal, outDepth))
                {
                    Vec2 contact1, contact2;
                    int contactCount = FindPolygonsContactPoints(rb1->GetCollider().GetBox()->ToPolygon(), *rb2->GetCollider().GetPolygon(), rb1->GetPosition(),
                        rb2->GetPosition(), rb1->GetRotation(), rb2->GetRotation(), contact1, contact2);
                    outContacts.push_back(contact1);
                    if(contactCount == 2)
                        outContacts.push_back(contact2);
                    return true;
                }
            }
        }
        else if(rb1->GetCollider().IsPolygon())
        {
            if(rb2->GetCollider().IsPolygon())
            {
                if(collisions::IntersectPolygons(*rb1->GetCollider().GetPolygon(), *rb2->GetCollider().GetPolygon(), rb1->GetPosition(), rb2->GetPosition(),
                    rb1->GetRotation(), rb2->GetRotation(), outNormal, outDepth))
                {
                    Vec2 contact1, contact2;
                    int contactCount = FindPolygonsContactPoints(*rb1->GetCollider().GetPolygon(), *rb2->GetCollider().GetPolygon(), rb1->GetPosition(),
                        rb2->GetPosition(), rb1->GetRotation(), rb2->GetRotation(), contact1, contact2);
                    outContacts.push_back(contact1);
                    if(contactCount == 2)
                        outContacts.push_back(contact2);
                    return true;
                }
            }
            else if(rb2->GetCollider().IsCircle())
            {
                if(collisions::IntersectCirclePolygon(*rb2->GetCollider().GetCircle(), *rb1->GetCollider().GetPolygon(), rb2->GetPosition(), rb1->GetPosition(),
                    rb1->GetRotation(), outNormal, outDepth))
                {
                    outNormal = -outNormal;
                    outContacts.push_back(FindCirclePolygonContactPoint(*rb2->GetCollider().GetCircle(), *rb1->GetCollider().GetPolygon(),
                        rb2->GetPosition(), rb1->GetPosition(), rb1->GetRotation()));
                    return true;
                }
            }
            else if(rb2->GetCollider().IsBox())
            {
                if(collisions::IntersectPolygons(*rb1->GetCollider().GetPolygon(), rb2->GetCollider().GetBox()->ToPolygon(), rb1->GetPosition(), rb2->GetPosition(),
                    rb1->GetRotation(), rb2->GetRotation(), outNormal, outDepth))
                {
                    Vec2 contact1, contact2;
                    int contactCount = FindPolygonsContactPoints(*rb1->GetCollider().GetPolygon(), rb2->GetCollider().GetBox()->ToPolygon(), rb1->GetPosition(),
                        rb2->GetPosition(), rb1->GetRotation(), rb2->GetRotation(), contact1, contact2);
                    outContacts.push_back(contact1);
                    if(contactCount == 2)
                        outContacts.push_back(contact2);
                    return true;
                }
            }
        }

        return false;
    }
    bool Collide(Rigidbody *rb1, Rigidbody *rb2)
    {
        Vec2 normal;
        float depth;
        std::vector<Vec2> contacts;

        return Collide(rb1, rb2, normal, depth, contacts);
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
        
        for(size_t i = 0; i < 2; i++)
        {
            for(size_t j = 0; j < axes[i]->size(); j++)
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

        for(size_t i = 0; i < axes.size(); i++)
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

        size_t cpIndex = geometry::FindClosestPointIndex(posA, verts);
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

        for(size_t i = 0; i < verts.size(); i++)
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
        size_t contactCount = 0;
        outContact1 = Vec2::Zero();
        outContact2 = Vec2::Zero();
        
        float minDistSq = std::numeric_limits<float>::max();

        std::vector<Vec2> vertsA = a.worldVertices;
        std::vector<Vec2> vertsB = b.worldVertices;

        for(size_t i = 0; i < vertsA.size(); i++)
        {
            Vec2 vert = vertsA[i];

            for(size_t j = 0; j < vertsB.size(); j++)
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
        for(size_t i = 0; i < vertsB.size(); i++)
        {
            Vec2 vert = vertsB[i];

            for(size_t j = 0; j < vertsA.size(); j++)
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