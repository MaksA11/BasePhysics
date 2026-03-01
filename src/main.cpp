#include <iostream>

// #include "Demo/App.hpp"
#include "Demo/PoolDemo.hpp"

int main()
{
    // demo::App *app = new demo::App();
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