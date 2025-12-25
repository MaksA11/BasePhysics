#include "Demo.h"

Demo::Demo()
{
    window = nullptr;
	clock = nullptr;
	deltaTime = 0.0f;
	appRunning = true;
}

void Demo::Init(unsigned int width, unsigned int height, const char *name)
{
	window = new sf::RenderWindow(sf::VideoMode(width, height), name);
    ImGui::SFML::Init(*window);
    window->setFramerateLimit(60);
	clock = new sf::Clock();
	windowSize = sf::Vector2u(width, height);
}

void Demo::Start()
{
    
}

void Demo::Update()
{

}

void Demo::Render()
{
    window->clear();

    // render here

    ImGui::SFML::Render(*window);
    window->display();
}
void Demo::RenderUI()
{
    ImGui::SFML::Update(*window, clock->getElapsedTime());

    // ui here
    ImGui::Begin("Hello");
    ImGui::Text("Hello, World!");
    ImGui::End();
}

void Demo::Input()
{
    sf::Event event;
	while(window->pollEvent(event))
	{
		ImGui::SFML::ProcessEvent(event);
		if(event.type == sf::Event::Closed)
			appRunning = false;
	}
}

void Demo::CalculateDeltaTime()
{
    deltaTime = clock->restart().asSeconds();
}
void Demo::Reset()
{
    
}
void Demo::CleanUp()
{
    ImGui::SFML::Shutdown();
	window->close();
	delete window;
	window = nullptr;
}
bool Demo::AppRunning()
{
    return appRunning;
}