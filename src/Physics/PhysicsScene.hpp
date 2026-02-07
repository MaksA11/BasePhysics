#pragma once

#include <vector>

#include "../Collision/Collider.hpp"
#include "../Collision/Collisions.hpp"
#include "../Collision/ContactManifold.hpp"
#include "Rigidbody.hpp"

namespace bp
{
    class PhysicsScene
    {
        private:
            Vec2 gravity;
            std::vector<Rigidbody *> bodies;
            std::vector<ContactManifold> contacts;

            void DetectCollisions();
            void SeparateBodies(const ContactManifold &contact);
            void ResolveCollision(const ContactManifold &contact);

        public:
            PhysicsScene();
            ~PhysicsScene();

            Rigidbody *AddRigidbody(Rigidbody *rb);
            Rigidbody *AddRigidbody(BodyPreset preset);
            void RemoveRigidbody(Rigidbody *rb);
            void RemoveRigidbody(int index);

            void Step(float deltaTime, unsigned int substeps, unsigned int iterations);

            void SetGravity(Vec2 val);

            const std::vector<Rigidbody *> &GetBodies() const;
            const std::vector<ContactManifold> &GetContacts() const;
    };
}