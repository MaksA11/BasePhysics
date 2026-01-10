#include "Rigidbody.hpp"

namespace bp
{
    Rigidbody *Rigidbody::CreateRigidbody(BodyPreset preset)
    {
        Collider collider = Collider(preset.shape, preset.restitution, preset.friction);
        return new Rigidbody(preset, collider);
    }
    Rigidbody *Rigidbody::CreateCircleBody(Vec2 position, float rotation, float radius, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity)
    {
        Collider collider = Collider(CircleShape(radius), restitution, friction);
        return new Rigidbody(position, rotation, collider, mass, linearDamping, angularDamping, isStatic, usesGravity);
    }
    Rigidbody *Rigidbody::CreateBoxBody(Vec2 position, float rotation, Vec2 size, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity)
    {
        Collider collider = Collider(BoxShape(size), restitution, friction);
        return new Rigidbody(position, rotation, collider, mass, linearDamping, angularDamping, isStatic, usesGravity);
    }

    void Rigidbody::DeleteRigidbody(Rigidbody *rb, std::vector<Rigidbody *> &bodies)
    {
        auto it = std::find(bodies.begin(), bodies.end(), rb);

        if(it != bodies.end())
        {
            delete rb;
            bodies.erase(it);
        }
    }

    Vec2 Rigidbody::GetPosition()
    {
        return position;
    }
    float Rigidbody::GetRotation()
    {
        return rotation;
    }

    Collider Rigidbody::GetCollider()
    {
        return collider;
    }

    void Rigidbody::Move(Vec2 delta)
    {
        position += delta;
    }
    void Rigidbody::MoveTo(Vec2 position)
    {
        this->position = position;
    }
    void Rigidbody::Rotate(float angle)
    {
        rotation += angle;
        rotation = std::fmod(rotation, math::ToRadians(360.0f));
    }

    void Rigidbody::PhysicsStep(float deltaTime, unsigned int substeps, Vec2 gravity)
    {
        if(isStatic)
		    return;

        deltaTime /= (float)substeps;

        Vec2 acceleration = force / mass;
        linearVelocity = linearVelocity + acceleration * deltaTime;
        if(usesGravity)
            linearVelocity = linearVelocity + gravity * deltaTime;

        Move(linearVelocity * deltaTime);
        Rotate(-angularVelocity * deltaTime);

        force = Vec2::Zero();
    }

    bool Rigidbody::IsStatic()
    {
        return isStatic;
    }
    bool Rigidbody::UsesGravity()
    {
        return usesGravity;
    }

    void Rigidbody::SetProperties(const BodyPreset &preset)
    {
        position = preset.position;
        rotation = preset.rotation;
        mass = preset.mass;
        linearDamping = preset.linearDamping;
        angularDamping = preset.angularDamping;
        isStatic = preset.isStatic;
        usesGravity = preset.usesGravity;
        collider = Collider(preset.shape, preset.restitution, preset.friction);
        inertia = collider.CalculateInertia(mass);
    }
    BodyPreset Rigidbody::GetProperties() const
    {
        BodyPreset preset;
        preset.position = position;
        preset.rotation = rotation;
        if(collider.IsCircle())
            preset.shape = *collider.GetCircle();
        else if(collider.IsBox())
            preset.shape = *collider.GetBox();
        else if(collider.IsPolygon())
            preset.shape = *collider.GetPolygon();
        preset.mass = mass;
        preset.linearDamping = linearDamping;
        preset.angularDamping = angularDamping;
        preset.restitution = collider.GetRestitution();
        preset.friction = collider.GetFriction();
        preset.isStatic = isStatic;
        preset.usesGravity = usesGravity;
        return preset;
    }
}