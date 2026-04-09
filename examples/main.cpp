#include <iostream>

#include "Demos/PhysicsDemo.hpp"
#include "Demos/PoolDemo.hpp"

#define WIDTH 2560
#define HEIGHT 1440

#ifdef _WIN32
#define WINDOW_TYPE demo::WindowType::BORDERLESS
#elif __linux__
#define WINDOW_TYPE demo::WindowType::WINDOW
#endif

int main(int argc, char *argv[])
{
    if(argc > 1)
	{
		std::string arg = argv[1];
		if(arg == "pool")
		{
            demo::PoolDemoApp app = demo::PoolDemoApp();

            app.Init(WIDTH, HEIGHT, "BasePhysics Demo", WINDOW_TYPE);
            app.Start();

            while(app.AppRunning())
            {
                app.CalculateDeltaTime();
                app.Input();
                app.Update();
                app.RenderUI();
                app.Render();
            }

            app.CleanUp();

            return EXIT_SUCCESS;
        }
	}

    demo::PhysicsDemoApp app = demo::PhysicsDemoApp();
    // demo::PoolDemoApp app = demo::PoolDemoApp();
	
    app.Init(WIDTH, HEIGHT, "BasePhysics Demo", WINDOW_TYPE);
    app.Start();

    while(app.AppRunning())
    {
        app.CalculateDeltaTime();
        app.Input();
        app.Update();
        app.RenderUI();
        app.Render();
    }

    app.CleanUp();

    return EXIT_SUCCESS;
}