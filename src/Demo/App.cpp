#include "App.h"

namespace demo
{
    App::App()
    {
        deltaTime = 0.0f;
        appRunning = true;
        fps = 0.0f;
    }

    void App::Init(unsigned int width, unsigned int height, const char *name, bool fullscreen)
    {
        window = fullscreen ? new sf::RenderWindow(sf::VideoMode(width, height), name, sf::Style::Fullscreen) : new sf::RenderWindow(sf::VideoMode(width, height), name);
        ImGui::SFML::Init(*window);
        // window->setFramerateLimit(60);
        clock = sf::Clock();
        camera = Camera(bp::Vec2::Zero(), window->getSize(), 50);
    }

    void App::Start()
    {
        bp::BodyPreset preset;
        preset.position = bp::Vec2::Zero();
        preset.rotation = 0.0f;
        preset.mass = 1.0f;
        preset.shape = bp::BoxShape(bp::Vec2::One());
        // preset.shape = bp::CircleShape(0.5f);
        preset.linearDamping = 0.1f;
        preset.angularDamping = 0.1f;
        preset.restitution = 0.1f;
        preset.friction = 0.1f;
        preset.isStatic = false;
        preset.usesGravity = true;

        scene.AddRigidbody(preset);
    }

    void App::Update()
    {
        scene.Step(deltaTime, 1);
        scene.GetBodies()[0]->Rotate(bp::math::pi * 0.5f * deltaTime);
    }

    void App::Render()
    {
        window->clear(sf::Color(80, 90, 100));

        sf::RectangleShape rectangle;
        sf::CircleShape circle;

        for(auto &rb : scene.GetBodies())
        {
            if(rb->GetCollider().IsCircle())
            {
                circle.setPosition(sf::Vector2f(rb->GetPosition().x, rb->GetPosition().y));
                circle.setRadius(rb->GetCollider().GetCircle()->radius);
                circle.setRotation(-bp::math::ToDegrees(rb->GetRotation()));
                circle.setFillColor(sf::Color::White);
                circle.setOutlineColor(sf::Color::Black);
                circle.setOutlineThickness(-0.067f);
                circle.setOrigin(sf::Vector2f(circle.getRadius(), circle.getRadius()));
                window->draw(circle);
            }
            else if(rb->GetCollider().IsBox())
            {
                rectangle.setPosition(sf::Vector2f(rb->GetPosition().x, rb->GetPosition().y));
                rectangle.setSize(sf::Vector2f(rb->GetCollider().GetBox()->size.x, rb->GetCollider().GetBox()->size.y));
                rectangle.setRotation(-bp::math::ToDegrees(rb->GetRotation()));
                rectangle.setFillColor(sf::Color::White);
                rectangle.setOutlineColor(sf::Color::Black);
                rectangle.setOutlineThickness(-0.067f);
                rectangle.setOrigin(sf::Vector2f(rectangle.getSize().x / 2, rectangle.getSize().y / 2));
                window->draw(rectangle);

                sf::CircleShape vertex;
                bp::Vec2 *vertices = bp::collisions::GetBoxVertices(*rb->GetCollider().GetBox(), rb->GetPosition(), rb->GetRotation());
                for(int i = 0; i < 4; i++)
                {
                    float radius = 0.05f;
                    vertex.setPosition(sf::Vector2f(vertices[i].x, vertices[i].y));
                    vertex.setRadius(radius);
                    vertex.setFillColor(sf::Color::Green);
                    vertex.setOrigin(sf::Vector2f(radius, radius));
                    window->draw(vertex);
                }
                delete[] vertices;
            }
            else if(rb->GetCollider().IsPolygon())
            {
                // TODO: implement
            }
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
                if(event.mouseButton.button == sf::Mouse::Left)
                {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(*window);
                    bp::Vec2 worldPos = camera.ScreenToWorld(mousePos, *window);

                    bp::BodyPreset preset;
                    preset.position = worldPos;
                    preset.rotation = 0.0f;
                    preset.mass = 1.0f;
                    preset.shape = bp::CircleShape(0.5f);
                    preset.linearDamping = 0.1f;
                    preset.angularDamping = 0.1f;
                    preset.restitution = 0.1f;
                    preset.friction = 0.1f;
                    preset.isStatic = false;
                    preset.usesGravity = true;
                    scene.AddRigidbody(preset);
                }
                if(event.mouseButton.button == sf::Mouse::Right)
                {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(*window);
                    bp::Vec2 worldPos = camera.ScreenToWorld(mousePos, *window);
                    
                    bp::BodyPreset preset;
                    preset.position = worldPos;
                    preset.rotation = 0.0f;
                    preset.mass = 1.0f;
                    preset.shape = bp::BoxShape(bp::Vec2::One());
                    preset.linearDamping = 0.1f;
                    preset.angularDamping = 0.1f;
                    preset.restitution = 0.1f;
                    preset.friction = 0.1f;
                    preset.isStatic = false;
                    preset.usesGravity = true;
                    scene.AddRigidbody(preset);
                }
            }
            
            if(event.type == sf::Event::MouseWheelScrolled)
                camera.Zoom(event.mouseWheelScroll.delta > 0 ? 1.1f : 0.9f);
        }

        const float camSpeed = 5.0f * deltaTime;
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            camera.Move(sf::Vector2f(0.0f, camSpeed));
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            camera.Move(sf::Vector2f(0.0f, -camSpeed));
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            camera.Move(sf::Vector2f(-camSpeed, 0.0f));
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            camera.Move(sf::Vector2f(camSpeed, 0.0f));

        const float rbSpeed = 5.0f * deltaTime;
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
            scene.GetBodies()[0]->Move(bp::Vec2::Up() * rbSpeed);
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
            scene.GetBodies()[0]->Move(-bp::Vec2::Up() * rbSpeed);
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            scene.GetBodies()[0]->Move(bp::Vec2::Right() * rbSpeed);
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            scene.GetBodies()[0]->Move(-bp::Vec2::Right() * rbSpeed);
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