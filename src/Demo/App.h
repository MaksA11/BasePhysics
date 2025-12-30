#pragma once

#include <iostream>

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>
#include <imgui.h>
#include <imgui-SFML.h>

#include "../Physics/PhysicsScene.h"
#include "../Collision/Collider.h"

#include "Camera.h"

namespace demo
{
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

		public:
			App();

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
}