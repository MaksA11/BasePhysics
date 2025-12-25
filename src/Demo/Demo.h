#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>
#include <imgui.h>
#include <imgui-SFML.h>

class Demo
{
    private:
        sf::RenderWindow *window;
		sf::Clock *clock;
		sf::Vector2u windowSize;
		float deltaTime;
		bool appRunning;

    public:
		Demo();

		void Init(unsigned int width, unsigned int height, const char *name);

		void Start();
		void Update();
		void Render();
		void RenderUI();
		void Input();
		void CalculateDeltaTime();

		void Reset();
		void CleanUp();
		bool AppRunning();
};