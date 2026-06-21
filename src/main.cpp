#include "denavit_hartenberg.h"
#include "environment3d.h"
#include "raylib.h"

int main(void)
{
    constexpr int screenWidth = 1280;
    constexpr int screenHeight = 720;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Visualize3D");
    SetTargetFPS(60);

    Environment3D environment = CreateEnvironment3D();
    DhRobot robot = CreateDhRobot();
    InitEnvironment3D();
    InitDhVisuals();

    while (!WindowShouldClose())
    {
        UpdateEnvironment3D(&environment);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(environment.camera);
        DrawEnvironment3D(environment);
        DrawDhRobot3D(robot);
        EndMode3D();

        DrawDhRobotMatrices(robot);

        EndDrawing();
    }

    ShutdownDhVisuals();
    ShutdownEnvironment3D();
    CloseWindow();
    return 0;
}
