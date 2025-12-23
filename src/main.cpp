#include <iostream>

#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>
#include <imgui.h>
#include <imgui-SFML.h>

#include "Core/Vec2.h"

bp::Vec2 wektor;

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML window");
    window.setFramerateLimit(60);

    ImGui::SFML::Init(window);

    sf::Clock deltaClock;

    while(window.isOpen())
    {
        sf::Event event;
        while(window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(window, event);

            if(event.type == sf::Event::Closed)
                window.close();
        }

        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::Begin("Hello");

        wektor.x += 0.01 * deltaClock.getElapsedTime().asMicroseconds();
        wektor.y -= 0.01 * deltaClock.getElapsedTime().asMicroseconds();
        
        std::string message = std::to_string(wektor.x) + ", " + std::to_string(wektor.y) + " - " + std::to_string(wektor.Magnitude());
        ImGui::Text(message.c_str());
        ImGui::Text("Hello, World!");
        ImGui::End();

        window.clear();
        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();

    return EXIT_SUCCESS;
}