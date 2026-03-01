#include "PoolDemo.hpp"

namespace demo
{
    PoolDemoApp::PoolDemoApp()
    {
        deltaTime = 0.0f;
        appRunning = true;
        isDragging = false;
    }

    void PoolDemoApp::Init(unsigned int width, unsigned int height, const char *name, WindowType windowType)
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
        window->setFramerateLimit(500);
        clock = sf::Clock();
        camera = Camera(bp::Vec2::Zero(), window->getSize(), 50);

        random::Init();
        camera.Zoom(1.5f);
    }

    void PoolDemoApp::Start()
    {
        scene = bp::PhysicsScene(bp::Vec2(bp::Vec2::Zero()));

        bp::BodyPreset bound = bp::BodyPreset(bp::Vec2::Zero(), 0.0f, bp::BoxShape(bp::Vec2(1.0f, 10.0f)), 1.0f, 0.0f, 0.0f, 0.3f, 0.1f, true, false, false, true);
        bp::BodyPreset table = bp::BodyPreset(bp::Vec2::Zero(), 0.0f, bp::BoxShape(bp::Vec2(22.0f, 12.0f)), 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, true, false, true, true);
        bp::BodyPreset hole = bp::BodyPreset(bp::Vec2::Zero(), 0.0f, bp::CircleShape(0.67f), 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, true, false, true, true);
        bp::BodyPreset ball = bp::BodyPreset(bp::Vec2::Zero(), 0.0f, bp::CircleShape(0.35f), 0.5f, 0.75f, 0.75f, 0.9f, 0.0f, false, false, false, false);

        bound.position = bp::Vec2(10.5f, 0.0f);
        scene.AddRigidbody(bound);
        colors.push_back(sf::Color(0, 0, 0, 0));

        bound.position = bp::Vec2(-10.5f, 0.0f);
        scene.AddRigidbody(bound);
        colors.push_back(sf::Color(0, 0, 0, 0));

        bound.position = bp::Vec2(0.0f, 5.5f);
        bound.shape = bp::BoxShape(bp::Vec2(20.0f, 1.0f));
        scene.AddRigidbody(bound);
        colors.push_back(sf::Color(0, 0, 0, 0));

        bound.position = bp::Vec2(0.0f, -5.5f);
        scene.AddRigidbody(bound);
        colors.push_back(sf::Color(0, 0, 0, 0));

        scene.AddRigidbody(table);
        colors.push_back(sf::Color(90, 50, 30));

        table.shape = bp::BoxShape(bp::Vec2(20.0f, 10.0f));
        scene.AddRigidbody(table);
        colors.push_back(sf::Color(5, 105, 0));

        const float x = 10.0f;
        const float y = 5.0f;

        bp::Vec2 holePositions[] = {
            {-x, y}, {0.0f, y}, {x, y},
            {-x, -y}, {0.0f, -y}, {x, -y}
        };

        for(int i = 0; i < 6; i++)
        {
            hole.position = holePositions[i];
            scene.AddRigidbody(hole);
            colors.push_back(sf::Color(0, 0, 0));
        }
        hole.shape = bp::CircleShape(0.3f);
        for(int i = 0; i < 6; i++)
        {
            hole.position = holePositions[i];
            holes.push_back(scene.AddRigidbody(hole));
            colors.push_back(sf::Color(0, 0, 0));
        }

        bp::Vec2 origin = bp::Vec2(-4.5f, 0.0f);
        const float radius = 0.35f;
        const float spacing = 0.0f;
        const int rows = 5;
        const float diameter = radius * 2.0f;
        const float yStep = diameter + spacing;
        const float xStep = std::sqrt(3.0f) * radius + spacing;
        int i = 0;
        for(int col = 0; col < rows; col++)
        {
            for(int row = 0; row <= col; row++)
            {
                float x = origin.x - col * xStep;
                float y = origin.y + row * yStep - (col * yStep * 0.5f);

                ball.position = bp::Vec2(x, y);
                if(i % 2 == 0 && i != 4)
                {
                    solids.push_back(scene.AddRigidbody(ball));
                    colors.push_back(sf::Color(255, 0, 0));
                }
                else if(i != 4)
                {
                    stripes.push_back(scene.AddRigidbody(ball));
                    colors.push_back(sf::Color(0, 255, 0));
                }
                else
                {
                    eightBall = scene.AddRigidbody(ball);
                    colors.push_back(sf::Color(20, 20, 20));
                }

                i++;
            }
        }

        ball.position = bp::Vec2(4.5f, 0.0f);
        whiteBall = scene.AddRigidbody(ball);
        colors.push_back(sf::Color(255, 255, 255));
    }

    void PoolDemoApp::Update()
    {
        scene.Step(deltaTime, 6, 6);

        const std::vector<bp::Rigidbody *> &bodies = scene.GetBodies();

        for(int i = bodies.size() - 1; i >= 0; i--)
        {
            bp::Rigidbody *rb = bodies[i];

            for(bp::Rigidbody *hole : holes)
            {
                if(bp::collisions::Collide(rb, hole))
                {
                    if(rb == eightBall)
                    {
                        Reset();
                        return;
                    }

                    if(rb == whiteBall)
                    {
                        rb->MoveTo(bp::Vec2(4.5f, 0.15f));
                        rb->SetLinearVelocity(bp::Vec2::Zero());
                        rb->SetAngularVelocity(0.0f);
                        break; 
                    }

                    auto itSolid = std::find(solids.begin(), solids.end(), rb);
                    auto itStripe = std::find(stripes.begin(), stripes.end(), rb);

                    if(itSolid != solids.end() || itStripe != stripes.end())
                    {
                        if(itSolid != solids.end())
                            solids.erase(itSolid);
                        if(itStripe != stripes.end())
                            stripes.erase(itStripe);

                        if(i < colors.size())
                            colors.erase(colors.begin() + i);

                        scene.RemoveRigidbody(i);
                        break;
                    }
                }
            }
        }
    }

    void PoolDemoApp::Render()
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

            i++;
        }

        if(isDragging)
        {
            sf::Vector2i mousePos = sf::Mouse::getPosition(*window);
            bp::Vec2 worldPos = camera.ScreenToWorld(mousePos, *window);
            bp::Vec2 va = whiteBall->GetPosition();
            bp::Vec2 vb = worldPos;
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(va.x, va.y), sf::Color::Black),
                sf::Vertex(sf::Vector2f(vb.x, vb.y), sf::Color::Black)
            };

            window->draw(line, 2, sf::Lines);
        }

        ImGui::SFML::Render(*window);
        camera.ApplyTo(*window);
        window->display();
    }
    void PoolDemoApp::RenderUI()
    {
        ImGui::SFML::Update(*window, clock.getElapsedTime());

        ImGui::GetBackgroundDrawList()->AddText(
            ImGui::GetFont(),
            50.0f,
            ImVec2(10.0f, 10.0f),
            IM_COL32(255, 0, 0, 255),
            ("Solids: " + std::to_string(7 - solids.size())).c_str()
        );
        ImGui::GetBackgroundDrawList()->AddText(
            ImGui::GetFont(),
            50.0f,
            ImVec2(10.0f, 70.0f),
            IM_COL32(0, 255, 0, 255),
            ("Stripes: " + std::to_string(7 - stripes.size())).c_str()
        );
    }

    void PoolDemoApp::Input()
    {
        sf::Event event;
        while(window->pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);
            if(event.type == sf::Event::Closed)
                appRunning = false;

            if(event.type == sf::Event::MouseButtonPressed)
            {
                if(event.mouseButton.button == sf::Mouse::Left && whiteBall->GetLinearVelocity().Magnitude() < 0.05f)
                    isDragging = true;
            }
            if(event.type == sf::Event::MouseButtonReleased)
            {
                if(event.mouseButton.button == sf::Mouse::Left && isDragging)
                {
                    isDragging = false;

                    sf::Vector2i mousePos = sf::Mouse::getPosition(*window);
                    bp::Vec2 worldPos = camera.ScreenToWorld(mousePos, *window);
                    bp::Vec2 direction = whiteBall->GetPosition() - worldPos;
                    bp::Vec2 impulse = direction * 2.35f;
                    impulse = bp::utils::ClampMagnitude(impulse, 0.0f, 35.0f);
                    
                    whiteBall->ApplyImpulse(impulse);
                }
            }
        }

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
            appRunning = false;
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::R))
            Reset();
    }

    void PoolDemoApp::CalculateDeltaTime()
    {
        deltaTime = clock.restart().asSeconds();
        fps = deltaTime > 0.0f ? 1.0f / deltaTime : 0.0f;
    }
    void PoolDemoApp::Reset()
    {
        for(int i = scene.GetBodies().size() - 1; i >= 0; i--)
        {
            scene.RemoveRigidbody(i);
        }
        colors.clear();
        colors.shrink_to_fit();
        holes.clear();
        holes.shrink_to_fit();
        solids.clear();
        solids.shrink_to_fit();
        stripes.clear();
        stripes.shrink_to_fit();
        Start();
    }
    void PoolDemoApp::CleanUp()
    {
        ImGui::SFML::Shutdown();
        window->close();
        delete window;
        window = nullptr;
    }
    bool PoolDemoApp::AppRunning()
    {
        return appRunning;
    }
}