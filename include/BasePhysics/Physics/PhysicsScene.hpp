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
            std::vector<size_t> pairCheckMatrix;
            size_t checkCounter;

            size_t aabbCollisionCheckCount;
            size_t satCollisionCheckCount;

            void DetectCollisions();
            void SeparateBodies(const ContactManifold &contact);
            void ResolveCollision(const ContactManifold &contact);

        public:
            PhysicsScene();
            ~PhysicsScene();

            void Init(Vec2 gravity, float cellSize);

            Rigidbody *AddRigidbody(Rigidbody *rb);
            Rigidbody *AddRigidbody(BodyPreset preset);
            Rigidbody *AddRigidbody(Vec2 position, float rotation, ColliderShape shape, float mass, float linearDamping, float angularDamping,
                float restitution, float friction, bool isStatic, bool lockRotation, bool usesGravity, bool isSensor);
                
            void RemoveRigidbody(Rigidbody *rb);
            void RemoveRigidbody(size_t index);

            Joint *CreateJoint(Joint *joint);
            Joint *CreateJoint(JointPreset preset);
            Joint *CreateJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, bool disableCollision, JointType jointType);

            void RemoveJoint(Joint *joint);
            void RemoveJoint(size_t index);

            void Step(float deltaTime, size_t substeps, size_t iterations);

            void SetGravity(Vec2 val);
            void Clear();

            const std::vector<Rigidbody *> &GetBodies() const;
            const std::vector<Joint *> &GetJoints() const;
            const std::vector<ContactManifold> &GetContacts() const;
            const HashGrid &GetHashGrid() const;

            size_t GetAABBCollisionCheckCount() const;
            size_t GetSATCollisionCheckCount() const;

            PhysicsScene(const PhysicsScene &) = delete;
            PhysicsScene & operator = (const PhysicsScene &) = delete;
    };
}