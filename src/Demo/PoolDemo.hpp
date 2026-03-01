#pragma once

#include <iostream>
#include <array>
#include <algorithm>

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

	class PoolDemoApp
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

			std::vector<sf::Color> colors;

            std::vector<bp::Rigidbody *> holes;
            std::vector<bp::Rigidbody *> solids;
            std::vector<bp::Rigidbody *> stripes;
            bp::Rigidbody *whiteBall;
            bp::Rigidbody *eightBall;

		public:
			PoolDemoApp();

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