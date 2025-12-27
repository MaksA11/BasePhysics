#pragma once

#include <vector>

#include "Rigidbody.h"

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

            void AddRigidbody(Rigidbody *rb);
            void RemoveRigidbody(Rigidbody *rb);

            void Step(float deltaTime, unsigned int substeps);

            const std::vector<Rigidbody *> &GetBodies() const;
    };
}