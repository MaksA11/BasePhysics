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
            SeparateBodies();
            ResolveCollisions();
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

                if(collisions::Collide(bodies[i], bodies[j], normal, depth))
                    contacts.push_back(ContactManifold(i, j, normal, depth, Vec2::Zero()));
            }
        }
    }
    void PhysicsScene::SeparateBodies()
    {
        for(auto contact : contacts)
        {
            Vec2 translationVector = contact.normal * contact.depth;
            bodies[contact.rbIndex1]->Move(-translationVector * 0.5f);
            bodies[contact.rbIndex2]->Move(translationVector * 0.5f);
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
}