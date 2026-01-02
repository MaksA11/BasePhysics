#include "Camera.hpp"

namespace demo
{
    Camera::Camera() {}
    Camera::Camera(bp::Vec2 position, sf::Vector2u windowSize, float zoom)
    {
        this->zoom = zoom;
        this->windowSize = windowSize;

        view.setCenter(position.x, position.y);
        view.setSize(windowSize.x / zoom, -(windowSize.y / zoom));
    }

    void Camera::Move(sf::Vector2f delta)
    {
        view.setCenter(view.getCenter() + delta);
    }
    void Camera::MoveTo(sf::Vector2f position)
    {
        view.setCenter(position);
    }
    void Camera::Zoom(float factor)
    {
        zoom *= factor;
        view.setSize(windowSize.x / zoom, -(windowSize.y / zoom));
    }
    void Camera::SetScreenSize(sf::Vector2u size)
    {
        windowSize = size;
        view.setSize(windowSize.x / zoom, -(windowSize.y / zoom));
    }
    
    void Camera::ApplyTo(sf::RenderWindow& window)
    {
        window.setView(view);
    }

    sf::Vector2i Camera::WorldToScreen(bp::Vec2 position, sf::RenderWindow& window)
    {
        sf::Vector2f world = {position.x, position.y};
        return window.mapCoordsToPixel(world, view);
    }
    bp::Vec2 Camera::ScreenToWorld(sf::Vector2i position, sf::RenderWindow& window)
    {
        sf::Vector2f world = window.mapPixelToCoords(position, view);
        return bp::Vec2(world.x, world.y);
    }

    std::string Camera::DebugInfo()
    {
        return "x: " + std::to_string(view.getCenter().x) + ", y: " + std::to_string(view.getCenter().y) + "\nzoom: " + std::to_string(zoom);
    }
}