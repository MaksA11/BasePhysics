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
            StepBodies(deltaTime, substeps);
            DetectCollisions();
            SeparateBodies();
            ResolveCollisions();
        }
    }
    void PhysicsScene::StepBodies(float deltaTime, unsigned int substeps)
    {
        for(auto &rb : bodies)
        {
            rb->PhysicsStep(deltaTime, substeps, gravity);
        }
    }

    void PhysicsScene::DetectCollisions()
    {
        contacts.clear();

        for(int i = 0; i < bodies.size() - 1; i++)
        {
            for(int j = i + 1; j < bodies.size(); j++)
            {
                Vec2 normal;
                float depth;
                std::vector<Vec2> contactPoints;

                if(collisions::Collide(bodies[i], bodies[j], normal, depth, contactPoints))
                {
                    contacts.push_back(ContactManifold(i, j, normal, depth, contactPoints[0]));
                    if(contactPoints.size() == 2)
                        contacts.push_back(ContactManifold(i, j, normal, depth, contactPoints[0], contactPoints[1]));
                }
            }
        }
    }
    void PhysicsScene::SeparateBodies()
    {
        for(auto contact : contacts)
        {
            Vec2 translationVector = contact.normal * contact.depth;
            Rigidbody *rb1 = bodies[contact.rbIndex1];
            Rigidbody *rb2 = bodies[contact.rbIndex2];

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
    }
    void PhysicsScene::ResolveCollisions()
    {
        for(auto contact : contacts)
        {
            
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