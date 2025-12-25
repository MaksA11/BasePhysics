#include <iostream>

#include "Demo/Demo.h"

int main()
{
    Demo *demo = new Demo();
	
    demo->Init(640*2, 480*2, "BasePhysics Demo");
    demo->Start();

    while(demo->AppRunning())
    {
        demo->CalculateDeltaTime();
        demo->Input();
        demo->Update();
        demo->RenderUI();
        demo->Render();
    }

    demo->CleanUp();

    delete demo;

    return EXIT_SUCCESS;
}