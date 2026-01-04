#pragma once

#include <vector>

#include "Rigidbody.hpp"
#include "../Collision/Collider.hpp"
#include "../Collision/Collisions.hpp"

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
            void RemoveRigidbody(int index);

            void Step(float deltaTime, unsigned int substeps);

            const std::vector<Rigidbody *> &GetBodies() const;
    };
}