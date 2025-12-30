#pragma once

#include <vector>

#include "Rigidbody.h"
#include "../Collision/Collider.h"
#include "../Collision/Collisions.h"

namespace bp
{
    class PhysicsScene
    {
        private:
            Vec2 gravity;
            std::vector<Rigidbody *> bodies;

        public:
            PhysicsScene();
            ~PhysicsScene();

            Rigidbody *AddRigidbody(Rigidbody *rb);
            Rigidbody *AddRigidbody(BodyPreset preset);
            void RemoveRigidbody(Rigidbody *rb);

            void Step(float deltaTime, unsigned int substeps);

            const std::vector<Rigidbody *> &GetBodies() const;
    };
}