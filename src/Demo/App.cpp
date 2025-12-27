#include "App.h"

namespace demo
{
    App::App()
    {
        deltaTime = 0.0f;
        appRunning = true;
        fps = 0.0f;
    }

    void App::Init(unsigned int width, unsigned int height, const char *name)
    {
        window = new sf::RenderWindow(sf::VideoMode(width, height), name);
        ImGui::SFML::Init(*window);
        // window->setFramerateLimit(60);
        clock = sf::Clock();
        camera = Camera(bp::Vec2::Zero(), window->getSize(), 50);
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
        window->clear(sf::Color(80, 90, 100));

        sf::RectangleShape rectangle;
        for(auto &rb : scene.GetBodies())
        {
            rectangle.setPosition(sf::Vector2f(rb->GetPosition().x, rb->GetPosition().y));
            rectangle.setSize(sf::Vector2f(rb->GetSize().x, rb->GetSize().y));
            rectangle.setFillColor(sf::Color::White);
            rectangle.setOutlineColor(sf::Color::Black);
            rectangle.setOutlineThickness(0.067f); 
            rectangle.setOrigin(sf::Vector2f(rectangle.getSize().x / 2, rectangle.getSize().y / 2));
            window->draw(rectangle);
        }

        ImGui::SFML::Render(*window);
        camera.ApplyTo(*window);
        window->display();
    }
    void App::RenderUI()
    {
        ImGui::SFML::Update(*window, clock.getElapsedTime());

        ImGui::Begin("Debug");
        ImGui::Text("Camera");
        ImGui::Text(camera.DebugInfo().c_str());
        ImGui::Separator();
        ImGui::Text("Body count");
        ImGui::Text(std::to_string(scene.GetBodies().size()).c_str());
        ImGui::Separator();
        ImGui::Text("FPS");
        ImGui::Text(std::to_string(fps).c_str());
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

            if(event.type == sf::Event::MouseButtonPressed)
            {
                if(event.mouseButton.button == sf::Mouse::Right)
                {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(*window);
                    bp::Vec2 worldPos = camera.ScreenToWorld(mousePos, *window);
                    scene.AddRigidbody(bp::Rigidbody::CreateRigidbody(worldPos, 0, bp::Vec2::One()));
                }
            }
            
            if(event.type == sf::Event::MouseWheelScrolled)
                camera.Zoom(event.mouseWheelScroll.delta > 0 ? 1.1f : 0.9f);
        }

        const float speed = 5.0f * deltaTime;
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            camera.Move(sf::Vector2f(0.0f, -speed));
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            camera.Move(sf::Vector2f(0.0f, speed));
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            camera.Move(sf::Vector2f(-speed, 0.0f));
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            camera.Move(sf::Vector2f(speed, 0.0f));
    }

    void App::CalculateDeltaTime()
    {
        deltaTime = clock.restart().asSeconds();
        fps = deltaTime > 0.0f ? 1.0f / deltaTime : 0.0f;
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