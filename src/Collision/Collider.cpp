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
            Vec2 min = Vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		    Vec2 max = Vec2(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

            std::vector<Vec2> verts = geometry::TransformPolygonVertices(box->ToPolygon().vertices, pos, rot);

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
        {
            Vec2 min = Vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		    Vec2 max = Vec2(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
            
            std::vector<Vec2> verts = geometry::TransformPolygonVertices(poly->vertices, pos, rot);

            for(int i = 0; i < verts.size(); i++)
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
}