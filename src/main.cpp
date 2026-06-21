#include "denavit_hartenberg.h"
#include "environment3d.h"
#include "inverse_kinematics.h"
#include "raylib.h"
#include "ui.h"

int main(void)
{
    SetConfigFlags(
        FLAG_MSAA_4X_HINT |
        FLAG_WINDOW_RESIZABLE |
        FLAG_WINDOW_HIGHDPI);
    InitWindow(1280, 720, "Visualize3D");
    MaximizeWindow();
    SetTargetFPS(60);

    Environment3D environment = CreateEnvironment3D();
    DhRobot robot = CreateDhRobot();
    InverseKinematicsController inverseKinematics =
        CreateInverseKinematicsController(robot);
    InitEnvironment3D();
    InitUi();

    while (!WindowShouldClose())
    {
        const bool inverseKinematicsCapturesMouse =
            UpdateInverseKinematics(
                &inverseKinematics,
                &robot,
                environment.camera);
        UpdateEnvironment3D(
            &environment,
            inverseKinematicsCapturesMouse);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(environment.camera);
        DrawEnvironment3D(environment);
        DrawDhRobot3D(robot);
        DrawInverseKinematics3D(inverseKinematics, robot);
        EndMode3D();

        DrawDhRobotMatrices(robot);
        DrawInverseKinematicsUi(inverseKinematics);

        EndDrawing();
    }

    ShutdownUi();
    ShutdownEnvironment3D();
    CloseWindow();
    return 0;
}
