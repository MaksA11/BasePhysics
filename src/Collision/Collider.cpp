#include "Collider.h"

namespace bp
{
    bool Collider::IsCircle()
    {
        return std::holds_alternative<CircleShape>(this->shape);
    }
    bool Collider::IsBox()
    {
        return std::holds_alternative<BoxShape>(this->shape);
    }
    bool Collider::IsPolygon()
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

    AABB Collider::GetAABB(Vec2 pos) const
    {
        if(const CircleShape* circle = GetCircle())
        {
            float r = circle->radius;
            return AABB(pos - Vec2(r, r), pos + Vec2(r, r));
        }
        if(const BoxShape* box = GetBox())
            return AABB(pos - box->size / 2, pos + box->size / 2);
        if(const PolygonShape* poly = GetPolygon())
            return AABB(Vec2::Zero(), Vec2::Zero()); // TODO: implement
        
        return AABB(Vec2::Zero(), Vec2::Zero());
    }
}