#pragma once

#include <vector>

#include <BasePhysics/Collision/Collider.hpp>
#include <BasePhysics/Collision/NarrowPhase/Collisions.hpp>
#include <BasePhysics/Collision/NarrowPhase/ContactManifold.hpp>
#include <BasePhysics/Collision/BroadPhase/HashGrid.hpp>
#include <BasePhysics/Physics/Rigidbody.hpp>
#include <BasePhysics/Physics/Joint.hpp>

namespace bp
{
    class PhysicsScene
    {
        private:
            Vec2 gravity;
            std::vector<Rigidbody *> bodies;
            std::vector<Joint *> joints;
            std::vector<ContactManifold> contacts;
            
            HashGrid hashGrid;
            std::vector<int> pairCheckMatrix;
            int checkCounter;

            int aabbCollisionCheckCount;
            int satCollisionCheckCount;

            void DetectCollisions();
            void SeparateBodies(const ContactManifold &contact);
            void ResolveCollision(const ContactManifold &contact);

        public:
            PhysicsScene();
            PhysicsScene(Vec2 gravity, float cellSize = 2.5f);
            ~PhysicsScene();

            Rigidbody *AddRigidbody(Rigidbody *rb);
            Rigidbody *AddRigidbody(BodyPreset preset);
            void RemoveRigidbody(Rigidbody *rb);
            void RemoveRigidbody(int index);

            Joint *CreateJoint(Joint *joint);
            Joint *CreateJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, JointType jointType);
            void RemoveJoint(Joint *joint);
            void RemoveJoint(int index);

            void Step(float deltaTime, unsigned int substeps, unsigned int iterations);

            void SetGravity(Vec2 val);

            const std::vector<Rigidbody *> &GetBodies() const;
            const std::vector<Joint *> &GetJoints() const;
            const std::vector<ContactManifold> &GetContacts() const;
            const HashGrid &GetHashGrid() const;

            int GetAABBCollisionCheckCount() const;
            int GetSATCollisionCheckCount() const;
    };
}