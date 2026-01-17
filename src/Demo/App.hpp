#pragma once

#include <iostream>

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>
#include <imgui.h>
#include <imgui-SFML.h>

#include "../Physics/PhysicsScene.hpp"
#include "../Collision/Collider.hpp"

#include "Camera.hpp"
#include "Random.hpp"

namespace demo
{
	enum WindowType
	{
		WINDOW = 0,
		FULLSCREEN = 1,
		BORDERLESS = 2
	};

	class App
	{
		private:
			sf::RenderWindow *window;
			sf::Clock clock;

			float deltaTime;
			bool appRunning;

			float fps;

			Camera camera;

			bp::PhysicsScene scene;

			sf::Vector2i lastMousePos;
			bool isDragging;
			
			bp::BodyPreset preset;
			std::vector<sf::Color> colors;

		public:
			App();

			void Init(unsigned int width, unsigned int height, const char *name, WindowType windowType);

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
}