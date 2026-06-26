#include <BasePhysics/Physics/Rigidbody.hpp>

namespace bp
{
    Rigidbody *Rigidbody::CreateRigidbody(BodyPreset preset)
    {
        Collider collider = Collider(preset.shape, preset.restitution, preset.friction, preset.isSensor);
        return new Rigidbody(preset, collider);
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

    Vec2 Rigidbody::GetPosition() const
    {
        return position;
    }
    float Rigidbody::GetRotation() const
    {
        return rotation;
    }

    const Collider &Rigidbody::GetCollider() const
    {
        return collider;
    }
    Collider &Rigidbody::GetCollider()
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
    void Rigidbody::SetRotation(float rotation)
    {
        this->rotation = rotation;
    }

    Vec2 Rigidbody::GetLinearVelocity() const
    {
        return linearVelocity;
    }
    void Rigidbody::SetLinearVelocity(Vec2 velocity)
    {
        linearVelocity = velocity;
    }
    float Rigidbody::GetAngularVelocity() const
    {
        return angularVelocity;
    }
    void Rigidbody::SetAngularVelocity(float velocity)
    {
        angularVelocity = velocity;
    }

    Vec2 Rigidbody::GetVelocityAtLocalPoint(const Vec2 &localPoint) const
    {
        return linearVelocity + math::Perpendicular(math::Rotate(localPoint, rotation)) * angularVelocity;
    }
    Vec2 Rigidbody::GetVelocityAtWorldPoint(const Vec2 &worldPoint) const
    {
        return linearVelocity + math::Perpendicular(worldPoint - position) * angularVelocity;
    }

    void Rigidbody::ApplyLinearImpulse(Vec2 impulse)
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

    void Rigidbody::ApplyImpulseAtLocalPoint(Vec2 impulse, Vec2 localPoint)
    {
        if(isStatic)
            return;

        linearVelocity += impulse * GetInverseMass();
        angularVelocity += math::Cross(localPoint, impulse) * GetInverseInertia();
    }
    void Rigidbody::ApplyImpulseAtWorldPoint(Vec2 impulse, Vec2 worldPoint)
    {
        if(isStatic)
            return;
        
        linearVelocity += impulse * GetInverseMass();
        angularVelocity += math::Cross(worldPoint - position, impulse) * GetInverseInertia();
    }

    void Rigidbody::ApplyForce(Vec2 force)
    {
        if(isStatic)
            return;
        this->force += force;
    }
    void Rigidbody::ApplyTorque(float torque)
    {
        if(isStatic)
            return;
        this->torque += torque;
    }
    
    float Rigidbody::GetMass() const
    {
        return mass;
    }
    float Rigidbody::GetInverseMass() const
    {
        if(isStatic)
            return 0.0f;
        return 1.0f / mass;
    }
    float Rigidbody::GetInertia() const
    {
        return inertia;
    }
    float Rigidbody::GetInverseInertia() const
    {
        if(isStatic || lockRotation)
            return 0.0f;
        return 1.0f / inertia;
    }

    void Rigidbody::IntegrateVelocity(float deltaTime, Vec2 gravity)
    {
        if(isStatic)
		    return;

        Vec2 acceleration = force / mass;
        linearVelocity += acceleration * deltaTime;

        float angularAcceleration = torque / inertia;
        angularVelocity += angularAcceleration * deltaTime;

        if(usesGravity)
            linearVelocity += gravity * deltaTime;

        force = Vec2::Zero();
        torque = 0.0f;
    }
    void Rigidbody::ApplyDamping(float deltaTime)
    {
        if(isStatic)
		    return;

        linearVelocity *= 1.0f / (1.0f + deltaTime * linearDamping);
        angularVelocity *= 1.0f / (1.0f + deltaTime * angularDamping);

        float linearThreshold = 0.0001f;
        float angularThreshold = 0.0001f;

        if(linearVelocity.MagnitudeSquared() < linearThreshold * linearThreshold)
            linearVelocity = Vec2::Zero();

        if(std::abs(angularVelocity) < angularThreshold)
            angularVelocity = 0.0f;
    }
    void Rigidbody::IntegratePosition(float deltaTime)
    {
        if(isStatic)
		    return;

        Move(linearVelocity * deltaTime);
        if(!lockRotation)
            Rotate(angularVelocity * deltaTime);
    }

    bool Rigidbody::IsStatic() const
    {
        return isStatic;
    }
    bool Rigidbody::UsesGravity() const
    {
        return usesGravity;
    }
    bool Rigidbody::IsRotationLocked() const
    {
        return lockRotation;
    }

    void Rigidbody::SetStatic(bool value)
    {
        linearVelocity = Vec2::Zero();
        angularVelocity = 0.0f;

        isStatic = value;
    }
    void Rigidbody::UseGravity(bool value)
    {
        usesGravity = value;
    }
    void Rigidbody::LockRotation(bool value)
    {
        lockRotation = value;
    }

    void Rigidbody::SetProperties(const BodyPreset &preset)
    {
        position = preset.position;
        rotation = preset.rotation;
        mass = preset.mass;
        linearDamping = preset.linearDamping;
        angularDamping = preset.angularDamping;
        if(isStatic != preset.isStatic || collider.IsSensor() != preset.isSensor)
        {
            linearVelocity = Vec2::Zero();
            angularVelocity = 0.0f;
        }
        isStatic = preset.isStatic;
        usesGravity = preset.usesGravity;
        lockRotation = preset.lockRotation;
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
        preset.lockRotation = lockRotation;
        preset.isSensor = collider.IsSensor();
        return preset;
    }
}