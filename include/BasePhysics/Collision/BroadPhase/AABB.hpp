#pragma once

#include <BasePhysics/Core/Vec2.hpp>

namespace bp
{
    struct AABB
    {
        Vec2 min, max;

        AABB(Vec2 min, Vec2 max) : min(min), max(max) {}
    };
}