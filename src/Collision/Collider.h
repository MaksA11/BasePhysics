#pragma once

#include <vector>
#include <variant>

#include "../Core/Vec2.h"
#include "AABB.h"

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
            Collider(CircleShape shape, float restitution, float friction) : shape(shape), restitution(restitution), friction(friction) {}
            Collider(BoxShape shape, float restitution, float friction) : shape(shape), restitution(restitution), friction(friction) {}
            Collider(PolygonShape shape, float restitution, float friction) : shape(shape), restitution(restitution), friction(friction) {}

            bool IsCircle();
            bool IsBox();
            bool IsPolygon();

            const CircleShape *GetCircle() const;
            const BoxShape *GetBox() const;
            const PolygonShape *GetPolygon() const;

            AABB GetAABB(Vec2 pos) const;
    };
}