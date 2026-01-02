#include <iostream>

#include "Demo/app.h"

int main()
{
    demo::App *app = new demo::App();
	
    app->Init(2560, 1440, "BasePhysics app", demo::WindowType::BORDERLESS);
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