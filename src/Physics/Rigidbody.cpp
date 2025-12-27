#include "Rigidbody.h"

namespace bp
{
    Rigidbody::Rigidbody(Vec2 position, float rotation, Vec2 size)
    {
        this->position = position;
        this->rotation = rotation;
        this->size = size;
    }

    Rigidbody *Rigidbody::CreateRigidbody(Vec2 position, float rotation, Vec2 size)
    {
        return new Rigidbody(position, rotation, size);
    }

    void Rigidbody::DeleteRigidbody(Rigidbody *rb, std::vector<Rigidbody *> &bodies)
    {
        int x;
    }

    Vec2 Rigidbody::GetPosition()
    {
        return position;
    }
    Vec2 Rigidbody::GetSize()
    {
        return size;
    }
    float Rigidbody::GetRotation()
    {
        return rotation;
    }
}