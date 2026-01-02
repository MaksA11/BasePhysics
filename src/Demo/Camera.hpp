#pragma once

#include <iostream>

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>

#include "../Core/Vec2.hpp"

namespace demo
{
    class Camera
    {
        private:
            sf::Vector2u windowSize;
            float zoom;

            sf::View view;

        public:
            Camera();
            Camera(bp::Vec2 position, sf::Vector2u windowSize, float zoom);

            void Move(sf::Vector2f delta);
            void MoveTo(sf::Vector2f position);
            void Zoom(float factor);
            void SetScreenSize(sf::Vector2u size);

            void ApplyTo(sf::RenderWindow &window);
            
            sf::Vector2i WorldToScreen(bp::Vec2 position, sf::RenderWindow &window);
            bp::Vec2 ScreenToWorld(sf::Vector2i position, sf::RenderWindow &window);

            std::string DebugInfo();
    };
}