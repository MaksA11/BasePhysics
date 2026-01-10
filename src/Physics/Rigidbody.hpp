#pragma once

#include <iostream>
#include <vector>

#include "../Core/Vec2.hpp"
#include "../Core/Math.hpp"
#include "../Collision/Collider.hpp"

namespace bp
{
    struct BodyPreset
    {
        Vec2 position;
        float rotation;

        ColliderShape shape;

        float mass, linearDamping, angularDamping, restitution, friction;
        bool isStatic, usesGravity;

        BodyPreset() : position(Vec2::Zero()), rotation(0.0f), shape(CircleShape(0.5f)), mass(1.0f), linearDamping(0.1f), angularDamping(0.1f),
            restitution(0.1f), friction(0.1f), isStatic(false), usesGravity(true) {}
        BodyPreset(Vec2 position, float rotation, ColliderShape shape, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity)
            : position(position), rotation(rotation), shape(shape), mass(mass), linearDamping(linearDamping), angularDamping(angularDamping),
            restitution(restitution), friction(friction), isStatic(isStatic), usesGravity(usesGravity) {}
    };

    class Rigidbody
    {
        private:
            Rigidbody(Vec2 position, float rotation, Collider collider, float mass, float linearDamping, float angularDamping, bool isStatic, bool usesGravity)
                : position(position), rotation(rotation), collider(collider), mass(mass), inertia(collider.CalculateInertia(mass)),
                linearDamping(linearDamping), angularDamping(angularDamping), isStatic(isStatic), usesGravity(usesGravity),
                linearVelocity(Vec2::Zero()), force(Vec2::Zero()), angularVelocity(0.0f), torque(0.0f) {}
            Rigidbody(BodyPreset preset, Collider collider)
                : position(preset.position), rotation(preset.rotation), collider(collider), mass(preset.mass), inertia(collider.CalculateInertia(preset.mass)),
                linearDamping(preset.linearDamping), angularDamping(preset.angularDamping), isStatic(preset.isStatic), usesGravity(preset.usesGravity),
                linearVelocity(Vec2::Zero()), force(Vec2::Zero()), angularVelocity(0.0f), torque(0.0f) {}

            Vec2 position;
            float rotation;

            Collider collider;
            float mass, inertia, linearDamping, angularDamping;
            bool isStatic, usesGravity;

            Vec2 linearVelocity, force;
            float angularVelocity, torque;

        public:
            static Rigidbody *CreateRigidbody(BodyPreset preset);
            static Rigidbody *CreateCircleBody(Vec2 position, float rotation, float radius, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity);
            static Rigidbody *CreateBoxBody(Vec2 position, float rotation, Vec2 size, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity);
            // static Rigidbody *CreatePolygonBody(Vec2 position, float rotation, std::vector<Vec2> vertices, float restitution, float friction);

            static void DeleteRigidbody(Rigidbody *rb, std::vector<Rigidbody *> &bodies);

            Vec2 GetPosition();
            float GetRotation();

            Collider GetCollider();

            void Move(Vec2 delta);
            void MoveTo(Vec2 position);
            void Rotate(float angle);

            void PhysicsStep(float deltaTime, unsigned int substeps, Vec2 gravity);

            bool IsStatic();
            bool UsesGravity();

            void SetProperties(const BodyPreset &preset);
            BodyPreset GetProperties() const;
    };
}