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
    struct PolygonShape
    {
        std::vector<Vec2> vertices;
        std::vector<Vec2> normals;

        std::vector<Vec2> worldVertices;
        std::vector<Vec2> worldNormals;
        
        PolygonShape() {}
        PolygonShape(std::vector<Vec2> vertices) : vertices(vertices)
        {
            int n = vertices.size();

            worldVertices.resize(n);
            worldNormals.resize(n);
            normals.resize(n);

            for(int i = 0; i < n; i++)
            {
                Vec2 v1 = vertices[i];
                Vec2 v2 = vertices[(i + 1) % n];
                Vec2 edge = v2 - v1;
                normals[i] = math::Perpendicular(edge).Normalized();
            }
        }
    };
    struct BoxShape
    {
        Vec2 size;
        PolygonShape polygon;

        BoxShape(Vec2 size) : size(size)
        {
            float left = -size.x * 0.5f;
            float right = left + size.x;
            float bottom = -size.y * 0.5f;
            float top = bottom + size.y;

            std::vector<Vec2> vertices = {
                Vec2(left, top),
                Vec2(right, top),
                Vec2(right, bottom),
                Vec2(left, bottom)
            };

            polygon = PolygonShape(vertices);
        }

        const PolygonShape &ToPolygon() const
        {
            return polygon;
        }
        PolygonShape &ToPolygon()
        {
            return polygon;
        }
    };

    using ColliderShape = std::variant<CircleShape, BoxShape, PolygonShape>;
}