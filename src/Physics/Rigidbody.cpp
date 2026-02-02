#include "Rigidbody.hpp"

namespace bp
{
    Rigidbody *Rigidbody::CreateRigidbody(BodyPreset preset)
    {
        Collider collider = Collider(preset.shape, preset.restitution, preset.friction, preset.isSensor);
        return new Rigidbody(preset, collider);
    }
    Rigidbody *Rigidbody::CreateCircleBody(Vec2 position, float rotation, float radius, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity, bool isSensor)
    {
        Collider collider = Collider(CircleShape(radius), restitution, friction);
        return new Rigidbody(position, rotation, collider, mass, linearDamping, angularDamping, isStatic, usesGravity);
    }
    Rigidbody *Rigidbody::CreateBoxBody(Vec2 position, float rotation, Vec2 size, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity, bool isSensor)
    {
        Collider collider = Collider(BoxShape(size), restitution, friction, isSensor);
        return new Rigidbody(position, rotation, collider, mass, linearDamping, angularDamping, isStatic, usesGravity);
    }
    Rigidbody *Rigidbody::CreatePolygonBody(Vec2 position, float rotation, std::vector<Vec2> vertices, float mass, float linearDamping, float angularDamping, float restitution, float friction, bool isStatic, bool usesGravity, bool isSensor)
    {
        Collider collider = Collider(PolygonShape(vertices), restitution, isSensor);
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

    const Collider& Rigidbody::GetCollider() const
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

    Vec2 Rigidbody::GetLinearVelocity()
    {
        return linearVelocity;
    }
    void Rigidbody::SetLinearVelocity(Vec2 velocity)
    {
        linearVelocity = velocity;
    }
    float Rigidbody::GetAngularVelocity()
    {
        return angularVelocity;
    }
    void Rigidbody::SetAngularVelocity(float velocity)
    {
        angularVelocity = velocity;
    }

    void Rigidbody::ApplyImpulse(Vec2 impulse)
    {
        if(isStatic)
            return;
        linearVelocity += impulse * GetInverseMass();
    }
    void Rigidbody::ApplyAngularImpulse(float impulse)
    {
        if(isStatic)
            return;
        angularVelocity += impulse * GetInverseInertia();
    }

    void Rigidbody::ApplyForce(Vec2 force)
    {
        if(isStatic)
            return;
        this->force += force / mass;
    }
    void Rigidbody::ApplyTorque(float torque)
    {
        if(isStatic)
            return;
        this->torque += torque / inertia;
    }
    
    float Rigidbody::GetMass()
    {
        return mass;
    }
    float Rigidbody::GetInverseMass()
    {
        if(isStatic)
            return 0.0f;
        return 1.0f / mass;
    }
    float Rigidbody::GetInertia()
    {
        return inertia;
    }
    float Rigidbody::GetInverseInertia()
    {
        if(isStatic)
            return 0.0f;
        return 1.0f / inertia;
    }

    void Rigidbody::PhysicsStep(float deltaTime, unsigned int substeps, Vec2 gravity)
    {
        if(isStatic)
		    return;

        deltaTime /= (float)substeps;

        Vec2 acceleration = force / mass;
        linearVelocity += acceleration * deltaTime;

        float angularAcceleration = torque / inertia;
        angularVelocity += angularAcceleration * deltaTime;

        linearVelocity *= 1.0f / (1.0f + deltaTime * linearDamping);
        angularVelocity *= 1.0f / (1.0f + deltaTime * angularDamping);

        if(usesGravity)
            linearVelocity += gravity * deltaTime;

        Move(linearVelocity * deltaTime);
        Rotate(angularVelocity * deltaTime);

        force = Vec2::Zero();
        torque = 0.0f;
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
        if(isStatic != preset.isStatic)
        {
            linearVelocity = Vec2::Zero();
            angularVelocity = 0.0f;
        }
        isStatic = preset.isStatic;
        usesGravity = preset.usesGravity;
        collider = Collider(preset.shape, preset.restitution, preset.friction, preset.isSensor);
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
        preset.isSensor = collider.IsSensor();
        return preset;
    }
}