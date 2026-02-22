#include "PhysicsScene.hpp"

namespace bp
{
    PhysicsScene::PhysicsScene()
    {
        gravity = Vec2(0.0f, -9.81f);
        contacts.reserve(512);
    }
    PhysicsScene::PhysicsScene(Vec2 gravity)
    {
        this->gravity = gravity;
        contacts.reserve(512);
    }
    PhysicsScene::~PhysicsScene()
    {
        for(Rigidbody *rb : bodies)
        {
            delete rb;
        }
        bodies.clear();
    }

    Rigidbody *PhysicsScene::AddRigidbody(Rigidbody *rb)
    {
        bodies.push_back(rb);
        return rb;
    }
    Rigidbody *PhysicsScene::AddRigidbody(BodyPreset preset)
    {
        Rigidbody *rb = Rigidbody::CreateRigidbody(preset);
        bodies.push_back(rb);
        return rb;
    }
    void PhysicsScene::RemoveRigidbody(Rigidbody *rb)
    {
        Rigidbody::DeleteRigidbody(rb, bodies);
    }
    void PhysicsScene::RemoveRigidbody(int index)
    {
        Rigidbody::DeleteRigidbody(bodies[index], bodies);
    }

    void PhysicsScene::Step(float deltaTime, unsigned int substeps, unsigned int iterations)
    {
        deltaTime /= (float)substeps;

        for(int substep = 0; substep < substeps; substep++)
        {
            for(Rigidbody *rb : bodies)
            {
                rb->IntegrateVelocity(deltaTime, gravity);
                rb->ApplyDamping(deltaTime);
                rb->IntegratePosition(deltaTime);
                rb->GetCollider().UpdateWorldGeometry(rb->GetPosition(), rb->GetRotation());
            }
            
            DetectCollisions();
            for(int iter = 0; iter < iterations; iter++)
            {
                for(const ContactManifold &contact : contacts)
                {
                    ResolveCollision(contact);
                }
            }
            
            for(const ContactManifold &contact : contacts)
            {
                SeparateBodies(contact);

                Rigidbody *rb1 = bodies[contact.rbIndex1];
                Rigidbody *rb2 = bodies[contact.rbIndex2];
                rb1->GetCollider().UpdateWorldGeometry(rb1->GetPosition(), rb1->GetRotation());
                rb2->GetCollider().UpdateWorldGeometry(rb2->GetPosition(), rb2->GetRotation());
            }
        }
    }

    void PhysicsScene::DetectCollisions()
    {
        contacts.clear();

        for(int i = 0; i < (int)bodies.size() - 1; i++)
        {
            for(int j = i + 1; j < (int)bodies.size(); j++)
            {
                if((bodies[i]->IsStatic() && bodies[j]->IsStatic()) || (bodies[i]->GetCollider().IsSensor() || bodies[j]->GetCollider().IsSensor()))
                    continue;

                if(!collisions::IntersectAABBs(bodies[i]->GetCollider().GetAABB(bodies[i]->GetPosition(), bodies[i]->GetRotation()),
                    bodies[j]->GetCollider().GetAABB(bodies[j]->GetPosition(), bodies[j]->GetRotation())))
				    continue;

                Vec2 normal;
                float depth;
                std::vector<Vec2> contactPoints;

                if(collisions::Collide(bodies[i], bodies[j], normal, depth, contactPoints))
                {
                    if(contactPoints.size() == 1)
                        contacts.push_back(ContactManifold(i, j, normal, depth, contactPoints[0]));
                    if(contactPoints.size() == 2)
                        contacts.push_back(ContactManifold(i, j, normal, depth, contactPoints[0], contactPoints[1]));
                }
            }
        }
    }
    void PhysicsScene::SeparateBodies(const ContactManifold &contact)
    {
        Rigidbody *rb1 = bodies[contact.rbIndex1];
        Rigidbody *rb2 = bodies[contact.rbIndex2];
        
        const float percent = 0.8f;
        const float slop = 0.01f;

        float invMassSum = rb1->GetInverseMass() + rb2->GetInverseMass();
        if(invMassSum == 0.0f)
            return;

        float correctionMag = std::max(contact.depth - slop, 0.0f) / invMassSum * percent;
        Vec2 correction = contact.normal * correctionMag;

        if(!rb1->IsStatic())
            rb1->Move(-correction * rb1->GetInverseMass());
        if(!rb2->IsStatic())
            rb2->Move(correction * rb2->GetInverseMass());
    }
    void PhysicsScene::ResolveCollision(const ContactManifold &contact)
    {
        Rigidbody *rb1 = bodies[contact.rbIndex1];
        Rigidbody *rb2 = bodies[contact.rbIndex2];

        const Vec2 &normal = contact.normal;
        const std::vector<Vec2> &contacts = contact.contactPoints;

        Vec2 impulses[2];
        Vec2 r1s[2];
        Vec2 r2s[2];
        float js[2];

        float e = std::min(rb1->GetCollider().GetRestitution(), rb2->GetCollider().GetRestitution());
        float sf = std::sqrt(rb1->GetCollider().GetFriction() * rb2->GetCollider().GetFriction());
        float df = 0.8f * sf;

        for(int i = 0; i < contacts.size(); i++)
        {
            Vec2 r1 = contacts[i] - rb1->GetPosition();
            Vec2 r2 = contacts[i] - rb2->GetPosition();

            r1s[i] = r1;
            r2s[i] = r2;

            Vec2 r1Perp = math::Perpendicular(r1);
            Vec2 r2Perp = math::Perpendicular(r2);

            Vec2 vel1 = r1Perp * rb1->GetAngularVelocity();
            Vec2 vel2 = r2Perp * rb2->GetAngularVelocity();

            Vec2 relativeVel = (rb2->GetLinearVelocity() + vel2) - (rb1->GetLinearVelocity() + vel1);

            float contactVelMag = math::Dot(relativeVel, normal);
            if(contactVelMag > 0.0f)
            {
                impulses[i] = Vec2::Zero();
                js[i] = 0.0f;
                continue;
            }

            float r1PerpDotN = math::Dot(r1Perp, normal);
            float r2PerpDotN = math::Dot(r2Perp, normal);

            float j = (-(1.0f + e) * contactVelMag) /
                (rb1->GetInverseMass() + rb2->GetInverseMass() +
                (r1PerpDotN * r1PerpDotN) * rb1->GetInverseInertia() +
                (r2PerpDotN * r2PerpDotN) * rb2->GetInverseInertia());

            j /= (float)contacts.size();
            
            js[i] = j;
            impulses[i] = j * normal;
        }
        for(int i = 0; i < contacts.size(); i++)
        {
            rb1->ApplyImpulse(-impulses[i]);
            rb1->ApplyAngularImpulse(math::Cross(r1s[i], -impulses[i]));
            rb2->ApplyImpulse(impulses[i]);
            rb2->ApplyAngularImpulse(math::Cross(r2s[i], impulses[i]));
        }

        for(int i = 0; i < contacts.size(); i++)
        {
            Vec2 r1 = contacts[i] - rb1->GetPosition();
            Vec2 r2 = contacts[i] - rb2->GetPosition();

            r1s[i] = r1;
            r2s[i] = r2;

            Vec2 r1Perp = math::Perpendicular(r1);
            Vec2 r2Perp = math::Perpendicular(r2);

            Vec2 vel1 = r1Perp * rb1->GetAngularVelocity();
            Vec2 vel2 = r2Perp * rb2->GetAngularVelocity();

            Vec2 relativeVel = (rb2->GetLinearVelocity() + vel2) - (rb1->GetLinearVelocity() + vel1);

            Vec2 tangent = relativeVel - math::Dot(relativeVel, normal) * normal;
            if(utils::NearlyEqual(tangent, Vec2::Zero()))
            {
                impulses[i] = Vec2::Zero();
                js[i] = 0.0f;
                continue;
            }
            
            tangent.Normalize();

            float r1PerpDotT = math::Dot(r1Perp, tangent);
            float r2PerpDotT = math::Dot(r2Perp, tangent);

            float jt = (-math::Dot(relativeVel, tangent)) /
                (rb1->GetInverseMass() + rb2->GetInverseMass() +
                (r1PerpDotT * r1PerpDotT) * rb1->GetInverseInertia() +
                (r2PerpDotT * r2PerpDotT) * rb2->GetInverseInertia());

            jt /= (float)contacts.size();

            if(std::abs(jt) <= js[i] * sf)
                impulses[i] = jt * tangent;
            else
                impulses[i] = -js[i] * tangent * df;
        }
        for(int i = 0; i < contacts.size(); i++)
        {
            rb1->ApplyImpulse(-impulses[i]);
            rb1->ApplyAngularImpulse(math::Cross(r1s[i], -impulses[i]));
            rb2->ApplyImpulse(impulses[i]);
            rb2->ApplyAngularImpulse(math::Cross(r2s[i], impulses[i]));
        }
    }

    void PhysicsScene::SetGravity(Vec2 val)
    {
        gravity = val;
    }

    const std::vector<Rigidbody *> &PhysicsScene::GetBodies() const
    {
        return bodies;
    }
    const std::vector<ContactManifold> &PhysicsScene::GetContacts() const
    {
        return contacts;
    }
}