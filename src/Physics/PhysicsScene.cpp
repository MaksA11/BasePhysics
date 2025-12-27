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

    void PhysicsScene::AddRigidbody(Rigidbody *rb)
    {
        bodies.push_back(rb);
    }
    void PhysicsScene::RemoveRigidbody(Rigidbody *rb)
    {
        int x;
    }

    void PhysicsScene::Step(float deltaTime, unsigned int substeps)
    {
        int x;
    }

    const std::vector<Rigidbody *> &PhysicsScene::GetBodies() const
    {
        return bodies;
    }
}