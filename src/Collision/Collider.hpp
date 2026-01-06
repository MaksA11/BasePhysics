#pragma once

#include <vector>
#include <variant>

#include "../Core/Vec2.hpp"
#include "AABB.hpp"

namespace bp
{
    struct CircleShape
    {
        float radius;

        CircleShape(float radius)
        {
            this->radius = radius;
        }
    };
    struct BoxShape
    {
        Vec2 size;
        Vec2 vertices[4];

        BoxShape(Vec2 size)
        {
            this->size = size;
        }
    };
    struct PolygonShape
    {
        std::vector<Vec2> vertices;

        PolygonShape(std::vector<Vec2> vertices)
        {
            this->vertices = vertices;
        }
    };

    using ColliderShape = std::variant<CircleShape, BoxShape, PolygonShape>;

    class Collider
    {
        private:
            ColliderShape shape;
            float restitution;
            float friction;

        public:
            Collider(ColliderShape shape, float restitution, float friction) : shape(shape), restitution(restitution), friction(friction) {}

            bool IsCircle() const;
            bool IsBox() const;
            bool IsPolygon() const;

            const CircleShape *GetCircle() const;
            const BoxShape *GetBox() const;
            const PolygonShape *GetPolygon() const;

            float GetRestitution() const;
            float GetFriction() const;

            const AABB GetAABB(Vec2 pos) const;

            float CalculateInertia(float mass);
    };
}