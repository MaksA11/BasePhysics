#pragma once

#include <chrono>

#include <SFML/Graphics.hpp>

#include "../Core/Vec2.hpp"

namespace demo::random
{
    inline void Init()
    {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
    }

    inline int RandomInt(int min, int max)
    {
        return std::rand() % (max - min + 1) + min;
    }

    inline float RandomFloat(float min, float max)
    {
        return min + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * (max - min);
    }

    inline bp::Vec2 RandomVec2(float min, float max)
    {
        return bp::Vec2(RandomFloat(min, max), RandomFloat(min, max));
    }

    inline bool RandomBool()
    {
        return std::rand() % 2 == 1;
    }

    inline sf::Color RandomColor()
    {
        return sf::Color(RandomInt(0, 255), RandomInt(0, 255), RandomInt(0, 255), 255);
    }
}