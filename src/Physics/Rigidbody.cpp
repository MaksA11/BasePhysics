#include "Rigidbody.h"

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
}