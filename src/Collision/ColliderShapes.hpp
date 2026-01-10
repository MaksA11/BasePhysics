#pragma once

#include <vector>
#include <variant>

#include "../Core/Vec2.hpp"

namespace bp
{
    struct CircleShape
    {
        float radius;

        CircleShape(float radius) : radius(radius) {}
    };
    struct BoxShape
    {
        Vec2 size;
        Vec2 vertices[4];

        BoxShape(Vec2 size) : size(size) {}
    };
    struct PolygonShape
    {
        std::vector<Vec2> vertices;

        PolygonShape(std::vector<Vec2> vertices) : vertices(vertices) {}
    };

    using ColliderShape = std::variant<CircleShape, BoxShape, PolygonShape>;
}