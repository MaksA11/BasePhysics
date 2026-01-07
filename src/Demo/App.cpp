#include "App.hpp"

namespace demo
{
    App::App()
    {
        deltaTime = 0.0f;
        appRunning = true;
        fps = 0.0f;
        isDragging = false;
    }

    void App::Init(unsigned int width, unsigned int height, const char *name, WindowType windowType)
    {
        if(windowType == WINDOW)
            window = new sf::RenderWindow(sf::VideoMode(width, height), name);
        else if(windowType == FULLSCREEN)
            window = new sf::RenderWindow(sf::VideoMode::getDesktopMode(), name, sf::Style::Fullscreen);
        else if(windowType == BORDERLESS)
            window = new sf::RenderWindow(sf::VideoMode::getDesktopMode(), name, sf::Style::None);
        else
        {
            std::cerr << "Unknown window type\n";
            exit(EXIT_FAILURE);
        }

        ImGui::SFML::Init(*window);
        // window->setFramerateLimit(60);
        clock = sf::Clock();
        camera = Camera(bp::Vec2::Zero(), window->getSize(), 50);

        random::Init();
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
        colors.push_back(random::RandomColor());

        for(int i = 0; i < 30; i++)
        {
            sf::Vector2f center = camera.GetPosition();
            sf::Vector2f size = camera.GetViewSize();
            float halfX = size.x * 0.5f;
            float halfY = std::abs(size.y) * 0.5f;
            float padding = 0.5f;

            preset.position = bp::Vec2(random::RandomFloat(-(center.x + halfX - padding), center.x + halfX - padding),
                random::RandomFloat(-(center.y + halfY - padding), center.y + halfY - padding));

            if(random::RandomBool())
                preset.shape = bp::BoxShape(random::RandomVec2(1.0f, 1.5f));
            else
                preset.shape = bp::CircleShape(random::RandomFloat(0.5f, 0.75f));

            scene.AddRigidbody(preset);
            colors.push_back(random::RandomColor());
        }
    }

    void App::Update()
    {
        scene.Step(deltaTime, 1);

        sf::Vector2f center = camera.GetPosition();
        sf::Vector2f size = camera.GetViewSize();

        float halfX = size.x * 0.5f;
        float halfY = std::abs(size.y) * 0.5f;

        for(auto &rb : scene.GetBodies())
        {
            bp::Vec2 pos = rb->GetPosition();

            if(pos.x > center.x + halfX)
                pos.x = center.x - halfX;
            else if(pos.x < center.x - halfX)
                pos.x = center.x + halfX;

            if(pos.y > center.y + halfY)
                pos.y = center.y - halfY;
            else if(pos.y < center.y - halfY)
                pos.y = center.y + halfY;

            rb->MoveTo(pos);
        }
    }

    void App::Render()
    {
        window->clear(sf::Color(80, 90, 100));

        sf::RectangleShape rectangle;
        sf::CircleShape circle;

        int i = 0;
        for(auto &rb : scene.GetBodies())
        {
            if(rb->GetCollider().IsCircle())
            {
                circle.setPosition(sf::Vector2f(rb->GetPosition().x, rb->GetPosition().y));
                circle.setRadius(rb->GetCollider().GetCircle()->radius);
                circle.setRotation(-bp::math::ToDegrees(rb->GetRotation()));
                circle.setFillColor(colors[i]);
                circle.setOutlineColor(sf::Color::Black);
                circle.setOutlineThickness(-0.067f);
                circle.setOrigin(sf::Vector2f(circle.getRadius(), circle.getRadius()));
                window->draw(circle);

                bp::Vec2 va = bp::math::Transform(bp::Vec2::Zero(), rb->GetPosition(), rb->GetRotation());
                bp::Vec2 vb = bp::math::Transform(bp::Vec2(circle.getRadius(), 0.0f), rb->GetPosition(), rb->GetRotation());
                sf::Vertex line[] = {
                    sf::Vertex(sf::Vector2f(va.x, va.y), sf::Color::Black),
                    sf::Vertex(sf::Vector2f(vb.x, vb.y), sf::Color::Black)
                };

                window->draw(line, 2, sf::Lines);
            }
            else if(rb->GetCollider().IsBox())
            {
                rectangle.setPosition(sf::Vector2f(rb->GetPosition().x, rb->GetPosition().y));
                rectangle.setSize(sf::Vector2f(rb->GetCollider().GetBox()->size.x, rb->GetCollider().GetBox()->size.y));
                rectangle.setRotation(-bp::math::ToDegrees(rb->GetRotation()));
                rectangle.setFillColor(colors[i]);
                rectangle.setOutlineColor(sf::Color::Black);
                rectangle.setOutlineThickness(-0.067f);
                rectangle.setOrigin(sf::Vector2f(rectangle.getSize().x * 0.5f, rectangle.getSize().y * 0.5f));
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

            i++;
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
        ImGui::Text("Body[0]");
        ImGui::Text(("x: " + std::to_string(scene.GetBodies()[0]->GetPosition().x) + ", y: " + std::to_string(scene.GetBodies()[0]->GetPosition().y) +
            "\nrot: " + std::to_string(scene.GetBodies()[0]->GetRotation())).c_str());
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
                    preset.shape = bp::CircleShape(random::RandomFloat(0.5f, 0.75f));
                    preset.linearDamping = 0.1f;
                    preset.angularDamping = 0.1f;
                    preset.restitution = 0.1f;
                    preset.friction = 0.1f;
                    preset.isStatic = false;
                    preset.usesGravity = true;
                    scene.AddRigidbody(preset);

                    colors.push_back(random::RandomColor());
                }
                if(event.mouseButton.button == sf::Mouse::Right)
                {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(*window);
                    bp::Vec2 worldPos = camera.ScreenToWorld(mousePos, *window);
                    
                    bp::BodyPreset preset;
                    preset.position = worldPos;
                    preset.rotation = 0.0f;
                    preset.mass = 1.0f;
                    preset.shape = bp::BoxShape(random::RandomVec2(1.0f, 1.5f));
                    preset.linearDamping = 0.1f;
                    preset.angularDamping = 0.1f;
                    preset.restitution = 0.1f;
                    preset.friction = 0.1f;
                    preset.isStatic = false;
                    preset.usesGravity = true;
                    scene.AddRigidbody(preset);

                    colors.push_back(random::RandomColor());
                }
                if(event.mouseButton.button == sf::Mouse::Middle)
                {
                    isDragging = true;
                    lastMousePos = sf::Mouse::getPosition(*window);
                }                
            }
            if(event.type == sf::Event::MouseButtonReleased)
            {
                if(event.mouseButton.button == sf::Mouse::Middle)
                    isDragging = false;
            }
            
            if(event.type == sf::Event::MouseWheelScrolled)
                camera.Zoom(event.mouseWheelScroll.delta > 0 ? 1.1f : 0.9f);
        }
        
        if(isDragging)
        {
            sf::Vector2i currentMousePos = sf::Mouse::getPosition(*window);
            sf::Vector2i delta = lastMousePos - currentMousePos;
            
            float worldDeltaX = delta.x / camera.GetZoom();
            float worldDeltaY = -delta.y / camera.GetZoom();
            
            camera.Move(sf::Vector2f(worldDeltaX, worldDeltaY));
            
            lastMousePos = currentMousePos;
        }

        const float rbSpeed = 5.0f * deltaTime;
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up) || sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            scene.GetBodies()[0]->Move(bp::Vec2::Up() * rbSpeed);
        else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down) || sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            scene.GetBodies()[0]->Move(-bp::Vec2::Up() * rbSpeed);
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right) || sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            scene.GetBodies()[0]->Move(bp::Vec2::Right() * rbSpeed);
        else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left) || sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            scene.GetBodies()[0]->Move(-bp::Vec2::Right() * rbSpeed);

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::E))
            scene.GetBodies()[0]->Rotate(bp::math::pi * 0.5f * deltaTime);
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Q))
            scene.GetBodies()[0]->Rotate(bp::math::pi * -0.5f * deltaTime);

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
            appRunning = false;
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