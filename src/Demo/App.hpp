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
			
			bp::Rigidbody *selectedRb;

			int shapeIndex;
			const std::array<bp::ColliderShape, 7> shapes = {
				bp::CircleShape(1.0f),
				bp::PolygonShape({{0.0f, 1.0f}, {0.866f, -0.5f}, {-0.866f, -0.5f}}),
				bp::BoxShape(bp::Vec2::One() * 1.8f),
				bp::PolygonShape({{0.0f, 1.0f}, {0.951f, 0.309f}, {0.588f, -0.809f}, {-0.588f, -0.809f}, {-0.951f, 0.309f}}),
				bp::PolygonShape({{0.0f, 1.0f}, {0.866f, 0.5f}, {0.866f, -0.5f}, {0.0f, -1.0f}, {-0.866f, -0.5f}, {-0.866f, 0.5f}}),
				bp::PolygonShape({{0.0f, 1.0f}, {0.782f, 0.623f}, {0.975f, -0.223f}, {0.434f, -0.901f}, {-0.434f, -0.901f}, {-0.975f, -0.223f}, {-0.782f, 0.623f}}),
				bp::PolygonShape({{0.383f, 0.924f}, {0.924f, 0.383f}, {0.924f, -0.383f}, {0.383f, -0.924f}, {-0.383f, -0.924f}, {-0.924f, -0.383f}, {-0.924f, 0.383f}, {-0.383f, 0.924f}})
			};

			bool renderBodies;
			bool renderOutlines;
			bool renderCircleLines;
			bool renderVertices;
			bool renderAABBs;
			bool renderContactPoints;

			int substeps;
			int framerateLimit;

			bp::BodyPreset spawnPreset;
			bp::BodyPreset selectedPreset;
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