#include "App.h"

namespace demo
{
    App::App()
    {
        deltaTime = 0.0f;
        appRunning = true;
    }

    void App::Init(unsigned int width, unsigned int height, const char *name)
    {
        window = new sf::RenderWindow(sf::VideoMode(width, height), name);
        ImGui::SFML::Init(*window);
        window->setFramerateLimit(60);
        clock = sf::Clock();
        camera = Camera(bp::Vec2::Zero(), {width, height}, 100);
    }

    void App::Start()
    {
        scene.AddRigidbody(bp::Rigidbody::CreateRigidbody(bp::Vec2::Zero(), 0, bp::Vec2::One()));
    }

    void App::Update()
    {
        
    }

    void App::Render()
    {
        window->clear();

        sf::RectangleShape rectangle;
        for(auto &rb : scene.GetBodies())
        {
            rectangle.setPosition({rb->GetPosition().x, rb->GetPosition().y});
            rectangle.setSize({rb->GetSize().x, rb->GetSize().y});
            rectangle.setFillColor(sf::Color::Green); 
            rectangle.setOrigin({rectangle.getSize().x / 2, rectangle.getSize().y / 2});
            window->draw(rectangle);
        }

        ImGui::SFML::Render(*window);
        camera.ApplyTo(*window);
        window->display();
    }
    void App::RenderUI()
    {
        ImGui::SFML::Update(*window, clock.getElapsedTime());

        // ui here
        ImGui::Begin("Hello");
        ImGui::Text("Hello, World!");
        ImGui::End();
    }

    void App::Input()
    {
        sf::Event event;
        while(window->pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);
            if(event.type == sf::Event::Closed)
                appRunning = false;
        }
    }

    void App::CalculateDeltaTime()
    {
        deltaTime = clock.restart().asSeconds();
    }
    void App::Reset()
    {
        
    }
    void App::CleanUp()
    {
        ImGui::SFML::Shutdown();
        window->close();
        delete window;
        window = nullptr;
    }
    bool App::AppRunning()
    {
        return appRunning;
    }
}