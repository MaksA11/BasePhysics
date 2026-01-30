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
        preset.position = bp::Vec2::Zero();
        preset.rotation = 0.0f;
        preset.mass = 1.0f;
        preset.shape = bp::BoxShape(bp::Vec2::One());
        // preset.shape = bp::CircleShape(0.5f);
        preset.linearDamping = 0.1f;
        preset.angularDamping = 0.1f;
        preset.restitution = 0.4f;
        preset.friction = 0.6f;
        preset.isStatic = false;
        preset.usesGravity = true;

        scene.AddRigidbody(preset);
        colors.push_back(random::RandomColor());

        bp::CircleShape circle = bp::CircleShape(0.5f);
        bp::BoxShape box = bp::BoxShape(bp::Vec2::One());
        bp::PolygonShape triangle = bp::PolygonShape({{0.0f, 1.0f}, {0.866f, -0.5f}, {-0.866f, -0.5f}});
        bp::PolygonShape pentagon = bp::PolygonShape({{0.0f, 1.0f}, {0.951f, 0.309f}, {0.588f, -0.809f}, {-0.588f, -0.809f}, {-0.951f, 0.309f}});
        bp::PolygonShape hexagon = bp::PolygonShape({{0.0f, 1.0f}, {0.866f, 0.5f}, {0.866f, -0.5f}, {0.0f, -1.0f}, {-0.866f, -0.5f}, {-0.866f, 0.5f}});
        bp::PolygonShape septagon = bp::PolygonShape({{0.0f, 1.0f}, {0.782f, 0.623f}, {0.975f, -0.223f}, {0.434f, -0.901f}, {-0.434f, -0.901f}, {-0.975f, -0.223f}, {-0.782f, 0.623f}});
        bp::PolygonShape octagon = bp::PolygonShape({{0.383f, 0.924f}, {0.924f, 0.383f}, {0.924f, -0.383f}, {0.383f, -0.924f}, {-0.383f, -0.924f}, {-0.924f, -0.383f}, {-0.924f, 0.383f}, {-0.383f, 0.924f}});

        for(int i = 0; i < 100; i++)
        {
            sf::Vector2f center = camera.GetPosition();
            sf::Vector2f size = camera.GetViewSize();
            float halfX = size.x * 0.5f;
            float halfY = std::abs(size.y) * 0.5f;
            float padding = 0.5f;

            preset.position = bp::Vec2(random::RandomFloat(-(center.x + halfX - padding), center.x + halfX - padding),
                random::RandomFloat(-(center.y + halfY - padding), center.y + halfY - padding));
                
            switch(random::RandomInt(2, 8))
            {
                case 2: preset.shape = circle; break;
                case 3: preset.shape = triangle; break;
                case 4: preset.shape = box; break;
                case 5: preset.shape = pentagon; break;
                case 6: preset.shape = hexagon; break;
                case 7: preset.shape = septagon; break;
                case 8: preset.shape = octagon; break;
            }

            scene.AddRigidbody(preset);
            colors.push_back(random::RandomColor());
        }

        preset.position = bp::Vec2(0.0f, -14.0f);
        preset.shape = bp::BoxShape(bp::Vec2(52.0f, 4.0f));
        preset.isStatic = true;
        
        scene.AddRigidbody(preset);
        colors.push_back(random::RandomColor());
        preset.isStatic = false;
    }

    void App::Update()
    {
        scene.Step(deltaTime, 8);
    }

    void App::Render()
    {
        window->clear(sf::Color(80, 90, 100));

        sf::RectangleShape rectangle;
        sf::CircleShape circle;
        sf::ConvexShape polygon;

        int i = 0;
        for(bp::Rigidbody *rb : scene.GetBodies())
        {
            if(rb->GetCollider().IsCircle())
            {
                circle.setPosition(sf::Vector2f(rb->GetPosition().x, rb->GetPosition().y));
                circle.setRadius(rb->GetCollider().GetCircle()->radius);
                circle.setRotation(bp::math::ToDegrees(rb->GetRotation()));
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
                rectangle.setRotation(bp::math::ToDegrees(rb->GetRotation()));
                rectangle.setFillColor(colors[i]);
                rectangle.setOutlineColor(sf::Color::Black);
                rectangle.setOutlineThickness(-0.067f);
                rectangle.setOrigin(sf::Vector2f(rectangle.getSize().x * 0.5f, rectangle.getSize().y * 0.5f));
                window->draw(rectangle);
            }
            else if(rb->GetCollider().IsPolygon())
            {
                polygon.setPointCount(rb->GetCollider().GetPolygon()->vertices.size());
        
                for(int i = 0; i < rb->GetCollider().GetPolygon()->vertices.size(); i++)
                {
                    polygon.setPoint(i, sf::Vector2f(rb->GetCollider().GetPolygon()->vertices[i].x,
                        rb->GetCollider().GetPolygon()->vertices[i].y));
                }
                
                polygon.setPosition(rb->GetPosition().x, rb->GetPosition().y);
                polygon.setRotation(bp::math::ToDegrees(rb->GetRotation()));
                polygon.setFillColor(colors[i]);
                polygon.setOutlineColor(sf::Color::Black);
                polygon.setOutlineThickness(-0.067f);
                polygon.setOrigin(0.0f, 0.0f);
                
                window->draw(polygon);
            }
            
            // auto aabb = rb->GetCollider().GetAABB(rb->GetPosition(), rb->GetRotation());
            // sf::Vector2f size(aabb.max.x - aabb.min.x, aabb.max.y - aabb.min.y);
            // sf::Vector2f position(aabb.min.x, aabb.min.y);
            // rectangle.setPosition(position);
            // rectangle.setSize(size);
            // rectangle.setFillColor(sf::Color::Transparent);
            // rectangle.setOutlineColor(sf::Color::Red);
            // rectangle.setOutlineThickness(0.04f);
            // rectangle.setOrigin(0.0f, 0.0f);
            // rectangle.setRotation(0.0f);
            // window->draw(rectangle);

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
        ImGui::Text("Contact count");
        ImGui::Text(std::to_string(scene.GetContacts().size()).c_str());
        ImGui::Separator();
        ImGui::Text("Body[0]");
        ImGui::Text(("x: " + std::to_string(scene.GetBodies()[0]->GetPosition().x) + ", y: " + std::to_string(scene.GetBodies()[0]->GetPosition().y) +
            "\nrot: " + std::to_string(scene.GetBodies()[0]->GetRotation()) +
            "\nvel: " + std::to_string(scene.GetBodies()[0]->GetLinearVelocity().x) + ", y: " + std::to_string(scene.GetBodies()[0]->GetLinearVelocity().y) +
            "\nangvel: " + std::to_string(scene.GetBodies()[0]->GetAngularVelocity())).c_str());
        ImGui::Separator();
        ImGui::Text("FPS");
        ImGui::Text(std::to_string(fps).c_str());
        ImGui::Separator();
        if(ImGui::Button("Reset"))
            Reset();
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

                    preset.shape = bp::CircleShape(random::RandomFloat(0.5f, 0.75f));
                    preset.position = worldPos;

                    scene.AddRigidbody(preset);
                    colors.push_back(random::RandomColor());
                }
                if(event.mouseButton.button == sf::Mouse::Right)
                {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(*window);
                    bp::Vec2 worldPos = camera.ScreenToWorld(mousePos, *window);
                    
                    preset.shape = bp::BoxShape(random::RandomVec2(1.0f, 1.5f));
                    preset.position = worldPos;

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

        const float rbForce = 5.0 * deltaTime;
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up) || sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            scene.GetBodies()[0]->ApplyImpulse(bp::Vec2::Up() * rbForce);
        else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down) || sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            scene.GetBodies()[0]->ApplyImpulse(-bp::Vec2::Up() * rbForce);
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right) || sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            scene.GetBodies()[0]->ApplyImpulse(bp::Vec2::Right() * rbForce);
        else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left) || sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            scene.GetBodies()[0]->ApplyImpulse(-bp::Vec2::Right() * rbForce);

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
        for(int i = scene.GetBodies().size() - 1; i >= 0; i--)
        {
            scene.RemoveRigidbody(i);
        }
        colors.clear();
        colors.shrink_to_fit();
        Start();
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