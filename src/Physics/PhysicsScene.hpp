#pragma once

#include <vector>
#include <utility>
#include <set>

#include "../Collision/Collider.hpp"
#include "../Collision/Collisions.hpp"
#include "../Collision/ContactManifold.hpp"
#include "../Collision/HashGrid.hpp"
#include "Rigidbody.hpp"

namespace bp
{
    class PhysicsScene
    {
        private:
            Vec2 gravity;
            std::vector<Rigidbody *> bodies;
            std::vector<ContactManifold> contacts;
            HashGrid hashGrid;

            void DetectCollisions();
            void SeparateBodies(const ContactManifold &contact);
            void ResolveCollision(const ContactManifold &contact);

        public:
            PhysicsScene();
            PhysicsScene(Vec2 gravity);
            ~PhysicsScene();

            Rigidbody *AddRigidbody(Rigidbody *rb);
            Rigidbody *AddRigidbody(BodyPreset preset);
            void RemoveRigidbody(Rigidbody *rb);
            void RemoveRigidbody(int index);

            void Step(float deltaTime, unsigned int substeps, unsigned int iterations);

            void SetGravity(Vec2 val);

            const std::vector<Rigidbody *> &GetBodies() const;
            const std::vector<ContactManifold> &GetContacts() const;
            const HashGrid &GetHashGrid() const;
    };
}