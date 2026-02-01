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
        window->setFramerateLimit(500);
        clock = sf::Clock();
        camera = Camera(bp::Vec2::Zero(), window->getSize(), 50);

        random::Init();
    }

    void App::Start()
    {
        selectedRb = nullptr;

        renderBodies = true;
        renderOutlines = true;
        renderCircleLines = true;
        renderVertices = false;
        renderAABBs = false;
        renderContactPoints = false;

        substeps = 8;
        fpsLimit = 500;

        shapeIndex = 0;
        spawnPreset.position = bp::Vec2(0.0f, -14.0f);
        spawnPreset.rotation = 0.0f;
        spawnPreset.shape = bp::BoxShape(bp::Vec2(52.0f, 4.0f));
        spawnPreset.mass = 1.0f;
        spawnPreset.linearDamping = 0.1f;
        spawnPreset.angularDamping = 0.1f;
        spawnPreset.restitution = 0.4f;
        spawnPreset.friction = 0.6f;
        spawnPreset.isStatic = true;
        spawnPreset.usesGravity = true;

        scene.AddRigidbody(spawnPreset);
        colors.push_back(random::RandomColor());
        spawnPreset.isStatic = false;

        for(int i = 0; i < 100; i++)
        {
            sf::Vector2f center = camera.GetPosition();
            sf::Vector2f size = camera.GetViewSize();
            float halfX = size.x * 0.5f;
            float halfY = std::abs(size.y) * 0.5f;
            float padding = 0.5f;

            spawnPreset.position = bp::Vec2(random::RandomFloat(-(center.x + halfX - padding), center.x + halfX - padding),
                random::RandomFloat(-(center.y + halfY - padding), center.y + halfY - padding));
                
            spawnPreset.shape = shapes[random::RandomInt(0, 6)];

            scene.AddRigidbody(spawnPreset);
            colors.push_back(random::RandomColor());
        }

        spawnPreset.isSensor = true;
    }

    void App::Update()
    {
        scene.Step(deltaTime, substeps);

        for(int i = 0; i < scene.GetBodies().size(); i++)
        {
            if(scene.GetBodies()[i]->GetPosition().y < -50.0f)
            {
                colors.erase(colors.begin() + i);
                scene.RemoveRigidbody(i);
            }
        }
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
            if(renderBodies)
            {
                if(rb->GetCollider().IsCircle())
                {
                    circle.setPosition(sf::Vector2f(rb->GetPosition().x, rb->GetPosition().y));
                    circle.setRadius(rb->GetCollider().GetCircle()->radius);
                    circle.setRotation(bp::math::ToDegrees(rb->GetRotation()));
                    circle.setFillColor(colors[i]);

                    if(selectedRb && selectedRb == rb)
                        circle.setOutlineColor(sf::Color::Red);
                    else
                        circle.setOutlineColor(sf::Color::Black);

                    circle.setOutlineThickness(renderOutlines ? -0.067f : 0.0f);
                    circle.setOrigin(sf::Vector2f(circle.getRadius(), circle.getRadius()));
                    window->draw(circle);

                    if(renderCircleLines)
                    {
                        bp::Vec2 va = bp::math::Transform(bp::Vec2::Zero(), rb->GetPosition(), rb->GetRotation());
                        bp::Vec2 vb = bp::math::Transform(bp::Vec2(circle.getRadius(), 0.0f), rb->GetPosition(), rb->GetRotation());
                        sf::Vertex line[] = {
                            sf::Vertex(sf::Vector2f(va.x, va.y), sf::Color::Black),
                            sf::Vertex(sf::Vector2f(vb.x, vb.y), sf::Color::Black)
                        };

                        window->draw(line, 2, sf::Lines);
                    }
                }
                else if(rb->GetCollider().IsBox())
                {
                    rectangle.setPosition(sf::Vector2f(rb->GetPosition().x, rb->GetPosition().y));
                    rectangle.setSize(sf::Vector2f(rb->GetCollider().GetBox()->size.x, rb->GetCollider().GetBox()->size.y));
                    rectangle.setRotation(bp::math::ToDegrees(rb->GetRotation()));
                    rectangle.setFillColor(colors[i]);

                    if(selectedRb && selectedRb == rb)
                        rectangle.setOutlineColor(sf::Color::Red);
                    else
                        rectangle.setOutlineColor(sf::Color::Black);

                    rectangle.setOutlineThickness(renderOutlines ? -0.067f : 0.0f);
                    rectangle.setOrigin(sf::Vector2f(rectangle.getSize().x * 0.5f, rectangle.getSize().y * 0.5f));

                    window->draw(rectangle);

                    if(renderVertices)
                    {
                        sf::CircleShape vertex;
                        std::vector<bp::Vec2> vertices = bp::geometry::TransformPolygonVertices(rb->GetCollider().GetBox()->ToPolygon().vertices, rb->GetPosition(), rb->GetRotation());
                        for(int i = 0; i < vertices.size(); i++)
                        {
                            float radius = 0.05f;
                            vertex.setPosition(sf::Vector2f(vertices[i].x, vertices[i].y));
                            vertex.setRadius(radius);
                            vertex.setFillColor(sf::Color::Green);
                            vertex.setOrigin(sf::Vector2f(radius, radius));
                            window->draw(vertex);
                        }
                    }
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

                    if(selectedRb && selectedRb == rb)
                        polygon.setOutlineColor(sf::Color::Red);
                    else
                        polygon.setOutlineColor(sf::Color::Black);

                    polygon.setOutlineThickness(renderOutlines ? -0.067f : 0.0f);
                    polygon.setOrigin(0.0f, 0.0f);
                    
                    window->draw(polygon);

                    if(renderVertices)
                    {
                        sf::CircleShape vertex;
                        const bp::PolygonShape &polygon = *rb->GetCollider().GetPolygon();
                        std::vector<bp::Vec2> vertices = bp::geometry::TransformPolygonVertices(polygon.vertices, rb->GetPosition(), rb->GetRotation());
                        for(int i = 0; i < vertices.size(); i++)
                        {
                            float radius = 0.05f;
                            vertex.setPosition(sf::Vector2f(vertices[i].x, vertices[i].y));
                            vertex.setRadius(radius);
                            vertex.setFillColor(sf::Color::Green);
                            vertex.setOrigin(sf::Vector2f(radius, radius));
                            window->draw(vertex);
                        }
                    }
                }
            }
            
            if(renderAABBs)
            {
                auto aabb = rb->GetCollider().GetAABB(rb->GetPosition(), rb->GetRotation());
                sf::Vector2f size(aabb.max.x - aabb.min.x, aabb.max.y - aabb.min.y);
                sf::Vector2f position(aabb.min.x, aabb.min.y);
                rectangle.setPosition(position);
                rectangle.setSize(size);
                rectangle.setFillColor(sf::Color::Transparent);
                rectangle.setOutlineColor(sf::Color::Red);
                rectangle.setOutlineThickness(0.04f);
                rectangle.setOrigin(0.0f, 0.0f);
                rectangle.setRotation(0.0f);
                window->draw(rectangle);
            }

            i++;
        }

        if(renderContactPoints)
        {
            for(const bp::ContactManifold &contact : scene.GetContacts())
            {
                sf::CircleShape circle;
                float radius = 0.05f;

                for(int i = 0; i < contact.contactPoints.size(); i++)
                {
                    circle.setPosition(sf::Vector2f(contact.contactPoints[i].x, contact.contactPoints[i].y));
                    circle.setRadius(radius);
                    circle.setFillColor(sf::Color::Magenta);
                    circle.setOrigin(sf::Vector2f(radius, radius));
                    window->draw(circle);
                }
            }
        }
        
        ImGui::SFML::Render(*window);
        camera.ApplyTo(*window);
        window->display();
    }
    void App::RenderUI()
    {
        ImGui::SFML::Update(*window, clock.getElapsedTime());

        ImGui::Begin("Simulation info");
        ImGui::Text("Camera");
        ImGui::Text(camera.DebugInfo().c_str());
        ImGui::Separator();
        ImGui::Text("Body count");
        ImGui::Text(std::to_string(scene.GetBodies().size()).c_str());
        ImGui::Separator();
        ImGui::Text("Contact count");
        ImGui::Text(std::to_string(scene.GetContacts().size()).c_str());
        ImGui::Separator();
        ImGui::Text("FPS");
        ImGui::Text(std::to_string(fps).c_str());
        ImGui::End();

        ImGui::Begin("Spawn");
        ImGui::Text("Shape");
        const char* items[] = { "Circle", "Triangle", "Box", "Pentagon", "Hexagon", "Septagon", "Octagon" };
        ImGui::ListBox("##Shape", &shapeIndex, items, IM_ARRAYSIZE(items), 7);
        spawnPreset.shape = shapes[shapeIndex];
        ImGui::Separator();
        ImGui::Checkbox("Is static", &spawnPreset.isStatic);
        ImGui::Checkbox("Use gravity", &spawnPreset.usesGravity);
        ImGui::Separator();
        ImGui::Text("Mass");
        ImGui::SliderFloat("##Mass", &spawnPreset.mass, 0.1f, 100.0f);
        ImGui::Text("Restitution");
        ImGui::SliderFloat("##Restitution", &spawnPreset.restitution, 0.0f, 10.0f);
        ImGui::Text("Friction");
        ImGui::SliderFloat("##Friction", &spawnPreset.friction, 0.0f, 10.0f);
        ImGui::Text("Linear damping");
        ImGui::SliderFloat("##Linear damping", &spawnPreset.linearDamping, 0.0f, 10.0f);
        ImGui::Text("Angular damping");
        ImGui::SliderFloat("##Angular damping", &spawnPreset.angularDamping, 0.0f, 10.0f);
        ImGui::End();

        ImGui::Begin("Selected rigidbody");
        if(selectedRb)
        {
            selectedPreset = selectedRb->GetProperties();

            std::string pos = "Position: " + selectedPreset.position.ToString();
            std::string rot = "Rotation: " + std::to_string(bp::math::ToDegrees(selectedPreset.rotation));
            std::string vel = "Linear velocity: " + selectedRb->GetLinearVelocity().ToString();
            std::string angVel = "Angular velocity: " + std::to_string(bp::math::ToDegrees(selectedRb->GetAngularVelocity()));

            ImGui::Text(pos.c_str());
            ImGui::Text(rot.c_str());
            ImGui::Text(vel.c_str());
            ImGui::Text(angVel.c_str());
            ImGui::Separator();
            ImGui::Checkbox("Is static", &selectedPreset.isStatic);
            ImGui::Checkbox("Use gravity", &selectedPreset.usesGravity);
            ImGui::Separator();
            ImGui::Text("Mass");
            ImGui::SliderFloat("##Mass", &selectedPreset.mass, 0.1f, 100.0f);
            ImGui::Text("Restitution");
            ImGui::SliderFloat("##Restitution", &selectedPreset.restitution, 0.0f, 10.0f);
            ImGui::Text("Friction");
            ImGui::SliderFloat("##Friction", &selectedPreset.friction, 0.0f, 10.0f);
            ImGui::Text("Linear damping");
            ImGui::SliderFloat("##Linear damping", &selectedPreset.linearDamping, 0.0f, 10.0f);
            ImGui::Text("Angular damping");
            ImGui::SliderFloat("##Angular damping", &selectedPreset.angularDamping, 0.0f, 10.0f);
            
            selectedRb->SetProperties(selectedPreset);

            ImGui::Separator();
            if(ImGui::Button("Delete rigidbody"))
            {
                const std::vector<bp::Rigidbody *> &bodies = scene.GetBodies();
                for(int i = 0; i < bodies.size(); i++)
                {
                    if(bodies[i] == selectedRb)
                    {
                        colors.erase(colors.begin() + i);
                        break;
                    }
                }

                scene.RemoveRigidbody(selectedRb);
                selectedRb = nullptr;
            }
        }
        else
            ImGui::Text("Rigidbody not selected");
        ImGui::End();

        ImGui::Begin("Simulation options");
        ImGui::Text("Substeps");
        ImGui::SliderInt("##Substeps", &substeps, 1, 24);
        ImGui::Text("FPS limit");
        if(ImGui::SliderInt("##FPS limit", &fpsLimit, 1, 500))
            window->setFramerateLimit(fpsLimit);
        ImGui::Separator();
        ImGui::Checkbox("Render bodies", &renderBodies);
        ImGui::Checkbox("Render outlines", &renderOutlines);
        ImGui::Checkbox("Render circle lines", &renderCircleLines);
        ImGui::Checkbox("Render vertices", &renderVertices);
        ImGui::Checkbox("Render AABBs", &renderAABBs);
        ImGui::Checkbox("Render contact points", &renderContactPoints);
        ImGui::Separator();
        if(ImGui::Button("Delete all rigidbodies"))
        {
            for(int i = scene.GetBodies().size() - 1; i > 0; --i)
            {
                colors.erase(colors.begin() + i);
                scene.RemoveRigidbody(i);
            }
        }
        if(ImGui::Button("Reset"))
            Reset();
        ImGui::SameLine();
        if(ImGui::Button("Exit"))
            appRunning = false;
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
                if(ImGui::GetIO().WantCaptureMouse)
                    continue;

                if(event.mouseButton.button == sf::Mouse::Left)
                {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(*window);
                    bp::Vec2 worldPos = camera.ScreenToWorld(mousePos, *window);

                    bp::Rigidbody *mouseRb = bp::Rigidbody::CreateCircleBody(worldPos, 0.0f, 0.00005f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, true, false, true);

                    for(bp::Rigidbody *rb : scene.GetBodies())
                    {
                        if(bp::collisions::Collide(rb, mouseRb))
                        {
                            selectedRb = rb;
                            break;
                        }
                        else
                            selectedRb = nullptr;   
                    }

                    delete mouseRb;
                }
                if(event.mouseButton.button == sf::Mouse::Right)
                {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(*window);
                    bp::Vec2 worldPos = camera.ScreenToWorld(mousePos, *window);
                    
                    spawnPreset.position = worldPos;

                    scene.AddRigidbody(spawnPreset);
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

            if(event.type == sf::Event::MouseWheelScrolled && !ImGui::GetIO().WantCaptureMouse)
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

        if(selectedRb)
        {
            const float rbForce = 5.0 * deltaTime;
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up) || sf::Keyboard::isKeyPressed(sf::Keyboard::W))
                selectedRb->ApplyImpulse(bp::Vec2::Up() * rbForce);
            else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down) || sf::Keyboard::isKeyPressed(sf::Keyboard::S))
                selectedRb->ApplyImpulse(-bp::Vec2::Up() * rbForce);
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right) || sf::Keyboard::isKeyPressed(sf::Keyboard::D))
                selectedRb->ApplyImpulse(bp::Vec2::Right() * rbForce);
            else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left) || sf::Keyboard::isKeyPressed(sf::Keyboard::A))
                selectedRb->ApplyImpulse(-bp::Vec2::Right() * rbForce);

            if(sf::Keyboard::isKeyPressed(sf::Keyboard::E))
                selectedRb->ApplyAngularImpulse(bp::math::pi * -0.05f * rbForce);
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Q))
                selectedRb->ApplyAngularImpulse(bp::math::pi * 0.05f * rbForce);

            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Delete))
            {
                const std::vector<bp::Rigidbody *> &bodies = scene.GetBodies();
                for(int i = 0; i < bodies.size(); i++)
                {
                    if(bodies[i] == selectedRb)
                    {
                        colors.erase(colors.begin() + i);
                        break;
                    }
                }

                scene.RemoveRigidbody(selectedRb);
                selectedRb = nullptr;
            }
        }

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