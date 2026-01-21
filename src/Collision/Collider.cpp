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

    const AABB Collider::GetAABB(Vec2 pos, float rot) const
    {
        if(const CircleShape *circle = GetCircle())
        {
            float r = circle->radius;
            return AABB(pos - Vec2(r, r), pos + Vec2(r, r));
        }
        if(const BoxShape *box = GetBox())
        {
            Vec2 min = Vec2(FLT_MAX, FLT_MAX);
		    Vec2 max = Vec2(-FLT_MAX, -FLT_MAX);

            std::array<Vec2, 4> verts = geometry::GetBoxVertices(*box, pos, rot);

            for(int i = 0; i < 4; i++)
			{
				if(verts[i].x < min.x)
					min.x = verts[i].x;
				if(verts[i].x> max.x)
					max.x = verts[i].x;
				if(verts[i].y < min.y)
					min.y = verts[i].y;
				if(verts[i].y > max.y)
					max.y = verts[i].y;
			}

            return AABB(min, max);
        }
        if(const PolygonShape *poly = GetPolygon())
            return AABB(Vec2::Zero(), Vec2::Zero()); // TODO: implement
        
        return AABB(Vec2::Zero(), Vec2::Zero());
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
            return 0.0f; // TODO: implement

        return 1.0f;
    }
}