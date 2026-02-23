#include "Collider.hpp"

namespace bp
{
    bool Collider::IsCircle() const
    {
        return std::holds_alternative<CircleShape>(this->shape);
    }
    bool Collider::IsBox() const
    {
        return std::holds_alternative<BoxShape>(this->shape);
    }
    bool Collider::IsPolygon() const
    {
        return std::holds_alternative<PolygonShape>(this->shape);
    }

    const CircleShape *Collider::GetCircle() const
    {
        return std::get_if<CircleShape>(&this->shape);
    }
    const BoxShape *Collider::GetBox() const
    {
        return std::get_if<BoxShape>(&this->shape);
    }
    const PolygonShape *Collider::GetPolygon() const
    {
        return std::get_if<PolygonShape>(&this->shape);
    }

    float Collider::GetRestitution() const
    {
        return restitution;
    }
    float Collider::GetFriction() const
    {
        return friction;
    }

    const AABB &Collider::GetAABB(Vec2 pos, float rot) const
    {
        return aabb;
    }

    float Collider::CalculateInertia(float mass)
    {
        if(IsCircle())
            return (1.0f / 2.0f) * mass * GetCircle()->radius * GetCircle()->radius;
        if(IsBox())
        {
            Vec2 size = GetBox()->size;
            return (1.0f / 12.0f) * mass * (size.x * size.x + size.y * size.y);
        }
        if(IsPolygon())
        {
            const std::vector<Vec2> &verts = GetPolygon()->vertices;
            
            if(verts.size() < 3)
                return 1.0f;

            float numerator = 0.0f;
            float denominator = 0.0f;

            for(int i = 0; i < verts.size(); i++)
            {
                const Vec2 &v1 = verts[i];
                const Vec2 &v2 = verts[(i + 1) % verts.size()];

                float cross = std::abs(math::Cross(v1, v2)); 
                float term = v1.MagnitudeSquared() + math::Dot(v1, v2) + v2.MagnitudeSquared();

                numerator += cross * term;
                denominator += cross;
            }

            return (mass / 6.0f) * (numerator / denominator);
        }

        return 1.0f;
    }
    void Collider::UpdateWorldGeometry(Vec2 pos, float rot)
    {
        std::visit([this, pos, rot](auto &&shape)
        {
            using T = std::decay_t<decltype(shape)>;
            if constexpr(std::is_same_v<T, CircleShape>) 
                this->aabb = AABB(pos - Vec2(shape.radius, shape.radius), pos + Vec2(shape.radius, shape.radius));
            else if constexpr(std::is_same_v<T, PolygonShape> || std::is_same_v<T, BoxShape>) 
            {
                PolygonShape &poly = [&]() -> PolygonShape &
                {
                    if constexpr(std::is_same_v<T, BoxShape>)
                        return shape.polygon;
                    else
                        return shape;
                }();

                float cosA = std::cos(rot);
                float sinA = std::sin(rot);

                Vec2 min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
                Vec2 max(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

                for(size_t i = 0; i < poly.vertices.size(); i++)
                {
                    float vx = poly.vertices[i].x;
                    float vy = poly.vertices[i].y;

                    Vec2 &worldV = poly.worldVertices[i];
                    worldV.x = vx * cosA - vy * sinA + pos.x;
                    worldV.y = vx * sinA + vy * cosA + pos.y;

                    if(worldV.x < min.x)
                        min.x = worldV.x;
                    if(worldV.y < min.y)
                        min.y = worldV.y;
                    if(worldV.x > max.x)
                        max.x = worldV.x;
                    if(worldV.y > max.y)
                        max.y = worldV.y;

                    float nx = poly.normals[i].x;
                    float ny = poly.normals[i].y;
                    poly.worldNormals[i].x = nx * cosA - ny * sinA;
                    poly.worldNormals[i].y = nx * sinA + ny * cosA;
                }

                this->aabb = AABB(min, max);
            }
        }, this->shape);
    }

    void Collider::SetSensor(bool val)
    {
        isSensor = val;
    }
    const bool Collider::IsSensor() const
    {
        return isSensor;
    }
}