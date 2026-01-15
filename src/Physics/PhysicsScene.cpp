#include "PhysicsScene.hpp"

namespace bp
{
    PhysicsScene::PhysicsScene()
    {
        gravity = Vec2(0.0f, -9.81f);
    }
    PhysicsScene::~PhysicsScene()
    {
        for(auto &rb : bodies)
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

    void PhysicsScene::Step(float deltaTime, unsigned int substeps)
    {
        for(int currentSubstep = 0; currentSubstep < substeps; currentSubstep++)
        {
            DetectCollisions();
            for(const ContactManifold &contact : contacts)
            {
                ResolveCollision(contact);
                SeparateBodies(contact);
            }
            for(auto &rb : bodies)
            {
                rb->PhysicsStep(deltaTime, substeps, gravity);
            }
        }
    }

    void PhysicsScene::DetectCollisions()
    {
        contacts.clear();
        contacts.shrink_to_fit();

        for(int i = 0; i < bodies.size() - 1; i++)
        {
            for(int j = i + 1; j < bodies.size(); j++)
            {
                if(bodies[i]->IsStatic() && bodies[j]->IsStatic())
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
        
        Vec2 translationVector = contact.normal * contact.depth;
        
        if(rb1->IsStatic())
            rb2->Move(translationVector);
        else if(rb2->IsStatic())
            rb1->Move(-translationVector);
        else
        {
            rb1->Move(-translationVector * 0.5f);
            rb2->Move(translationVector * 0.5f);
        }
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

        float e = std::min(rb1->GetCollider().GetRestitution(), rb2->GetCollider().GetRestitution());

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

            float contactVelMag = math::Dot(relativeVel, contact.normal);
            if(contactVelMag > 0.0f)
            {
                impulses[i] = Vec2::Zero();
                continue;
            }

            float r1PerpDotNormal = math::Dot(r1Perp, normal);
            float r2PerpDotNormal = math::Dot(r2Perp, normal);

            float j = (-(1.0f + e) * contactVelMag) /
                (rb1->GetInverseMass() + rb2->GetInverseMass() +
                (r1PerpDotNormal * r1PerpDotNormal) * rb1->GetInverseInertia() +
                (r2PerpDotNormal * r2PerpDotNormal) * rb2->GetInverseInertia());
            
            j /= (float)contacts.size();

            impulses[i] = normal * j;
        }
        for(int i = 0; i < contacts.size(); i++)
        {
            rb1->ApplyImpulse(-impulses[i]);
            rb1->ApplyAngularImpulse(math::Cross(r1s[i], -impulses[i]));
            rb2->ApplyImpulse(impulses[i]);
            rb2->ApplyAngularImpulse(math::Cross(r2s[i], impulses[i]));
        }
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