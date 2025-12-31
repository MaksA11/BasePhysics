#include "PhysicsScene.h"

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

    void PhysicsScene::Step(float deltaTime, unsigned int substeps)
    {
        for(int currentSubstep = 0; currentSubstep < substeps; currentSubstep++)
        {
            for(int i = 0; i < bodies.size() - 1; i++)
            {
                for(int j = i + 1; j < bodies.size(); j++)
                {
                    Vec2 normal;
                    float depth;

                    Rigidbody *bodyA = bodies[i];
                    Rigidbody *bodyB = bodies[j];

                    if(bodyA->GetCollider().IsCircle() && bodyB->GetCollider().IsCircle())
                    {
                        if(collisions::IntersectCircles(*bodyA->GetCollider().GetCircle(), *bodyB->GetCollider().GetCircle(), bodyA->GetPosition(), bodyB->GetPosition(), normal, depth))
                        {
                            Vec2 translationVector = normal * depth;

                            bodies[i]->Move(-translationVector * 0.5f);
                            bodies[j]->Move(translationVector * 0.5f);
                        }
                    }
                    // if(bodyA->GetCollider().IsBox() && bodyB->GetCollider().IsBox())
                    // {
                    //     if(collisions::IntersectBoxes())
                    //     {
                    //         Vec2 translationVector = normal * depth;

                    //         bodies[i]->Move(-translationVector * 0.5f);
                    //         bodies[j]->Move(translationVector * 0.5f);
                    //     }
                    // }
                }
            }
        }
    }

    const std::vector<Rigidbody *> &PhysicsScene::GetBodies() const
    {
        return bodies;
    }
}