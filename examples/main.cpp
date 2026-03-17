#include <iostream>

#include "Demos/PhysicsDemo.hpp"
#include "Demos/PoolDemo.hpp"

int main(int argc, char *argv[])
{
    if(argc > 1)
	{
		std::string arg = argv[1];
		if(arg == "pool")
		{
            demo::PoolDemoApp *app = new demo::PoolDemoApp();

            app->Init(2560, 1440, "BasePhysics Demo", demo::WindowType::BORDERLESS);
            app->Start();

            while(app->AppRunning())
            {
                app->CalculateDeltaTime();
                app->Input();
                app->Update();
                app->RenderUI();
                app->Render();
            }

            app->CleanUp();

            delete app;

            return EXIT_SUCCESS;
        }
	}

    demo::PhysicsDemoApp *app = new demo::PhysicsDemoApp();
    // demo::PoolDemoApp *app = new demo::PoolDemoApp();
	
    app->Init(2560, 1440, "BasePhysics Demo", demo::WindowType::BORDERLESS);
    app->Start();

    while(app->AppRunning())
    {
        app->CalculateDeltaTime();
        app->Input();
        app->Update();
        app->RenderUI();
        app->Render();
    }

    app->CleanUp();

    delete app;

    return EXIT_SUCCESS;
}