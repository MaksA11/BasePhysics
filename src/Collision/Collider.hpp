#pragma once

#include <vector>

#include "../Core/Vec2.hpp"
#include "../Core/Utils.hpp"
#include "../Core/Geometry.hpp"
#include "ColliderShapes.hpp"
#include "AABB.hpp"

namespace bp
{
    class Collider
    {
        private:
            ColliderShape shape;
            float restitution;
            float friction;

            bool isSensor;

        public:
            Collider(ColliderShape shape, float restitution, float friction, bool isSensor = false) : shape(shape), restitution(restitution), friction(friction), isSensor(isSensor) {}

            bool IsCircle() const;
            bool IsBox() const;
            bool IsPolygon() const;

            const CircleShape *GetCircle() const;
            const BoxShape *GetBox() const;
            const PolygonShape *GetPolygon() const;

            float GetRestitution() const;
            float GetFriction() const;

            const AABB GetAABB(Vec2 pos, float rot) const;

            float CalculateInertia(float mass);

            void SetSensor(bool val);
            const bool IsSensor() const;
    };
}