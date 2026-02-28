#pragma once

#include "../Core/Vec2.hpp"
#include "../Collision/ColliderShapes.hpp"

namespace bp
{
    struct BodyPreset
    {
        Vec2 position;
        float rotation;

        ColliderShape shape;

        float mass, linearDamping, angularDamping, restitution, friction;
        bool isStatic, usesGravity, lockRotation, isSensor;

        BodyPreset() : position(Vec2::Zero()), rotation(0.0f), shape(CircleShape(0.5f)), mass(1.0f), linearDamping(0.1f), angularDamping(0.1f),
            restitution(0.1f), friction(0.1f), isStatic(false), usesGravity(true), lockRotation(false), isSensor(false) {}
        BodyPreset(Vec2 position, float rotation, ColliderShape shape, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity, bool isSensor, bool lockRotation)
            : position(position), rotation(rotation), shape(shape), mass(mass), linearDamping(linearDamping), angularDamping(angularDamping),
            restitution(restitution), friction(friction), isStatic(isStatic), usesGravity(usesGravity), lockRotation(lockRotation), isSensor(isSensor) {}
    };
}