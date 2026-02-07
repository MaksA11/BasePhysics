#pragma once

#include <iostream>
#include <vector>

#include "../Core/Vec2.hpp"
#include "../Core/Math.hpp"
#include "../Core/Utils.hpp"
#include "../Collision/Collider.hpp"
#include "BodyPreset.hpp"

namespace bp
{
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
            static Rigidbody *CreateCircleBody(Vec2 position, float rotation, float radius, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity, bool isSensor);
            static Rigidbody *CreateBoxBody(Vec2 position, float rotation, Vec2 size, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity, bool isSensor);
            static Rigidbody *CreatePolygonBody(Vec2 position, float rotation, std::vector<Vec2> vertices, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity, bool isSensor);

            static void DeleteRigidbody(Rigidbody *rb, std::vector<Rigidbody *> &bodies);

            Vec2 GetPosition();
            float GetRotation();

            const Collider& GetCollider() const;

            void Move(Vec2 delta);
            void MoveTo(Vec2 position);
            void Rotate(float angle);

            Vec2 GetLinearVelocity();
            void SetLinearVelocity(Vec2 velocity);
            float GetAngularVelocity();
            void SetAngularVelocity(float velocity);

            void ApplyImpulse(Vec2 impulse);
            void ApplyAngularImpulse(float impulse);

            void ApplyForce(Vec2 force);
            void ApplyTorque(float torque);

            float GetMass();
            float GetInverseMass();
            float GetInertia();
            float GetInverseInertia();

            void IntegrateVelocity(float deltaTime, Vec2 gravity);
            void ApplyDamping(float deltaTime);
            void IntegratePosition(float deltaTime);

            bool IsStatic();
            bool UsesGravity();

            void SetProperties(const BodyPreset &preset);
            BodyPreset GetProperties() const;
    };
}