#pragma once

#include <iostream>
#include <array>
#include <algorithm>

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>
#include <imgui.h>
#include <imgui-SFML.h>

#include <BasePhysics/BasePhysics.hpp>

#include "Utils/WindowType.hpp"
#include "Utils/Camera.hpp"
#include "Utils/Random.hpp"

namespace demo
{
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
			
			bool isDragging;
			bool isPlacing;
			bool canBePlaced;

			bp::Vec2 whiteBallImpulse;
			float whiteBallImpulseMaxMagnitude;
			float whiteBallImpulseMultiplier;

			std::vector<sf::Color> colors;

            std::vector<bp::Rigidbody *> bounds;
            std::vector<bp::Rigidbody *> holes;
            std::vector<bp::Rigidbody *> solids;
            std::vector<bp::Rigidbody *> stripes;
            bp::Rigidbody *whiteBall;
            bp::Rigidbody *eightBall;
			bp::Rigidbody *innerTable;

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