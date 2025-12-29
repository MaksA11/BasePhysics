#include "Rigidbody.h"

namespace bp
{
    Rigidbody *Rigidbody::CreateCircleBody(Vec2 position, float rotation, float radius, float restitution, float friciton)
    {
        CircleShape shape = CircleShape(radius);
        Collider collider = Collider(shape, restitution, friciton);
        return new Rigidbody(position, rotation, collider);
    }
    Rigidbody *Rigidbody::CreateBoxBody(Vec2 position, float rotation, Vec2 size, float restitution, float friciton)
    {
        BoxShape shape = BoxShape(size);
        Collider collider = Collider(shape, restitution, friciton);
        return new Rigidbody(position, rotation, collider);
    }
    Rigidbody *Rigidbody::CreatePolygonBody(Vec2 position, float rotation, std::vector<Vec2> vertices, float restitution, float friciton)
    {
        PolygonShape shape = PolygonShape(vertices);
        Collider collider = Collider(shape, restitution, friciton);
        return new Rigidbody(position, rotation, collider);
    }

    void Rigidbody::DeleteRigidbody(Rigidbody *rb, std::vector<Rigidbody *> &bodies)
    {
        
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
}