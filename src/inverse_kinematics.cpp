#include "inverse_kinematics.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "raymath.h"
#include "ui.h"

namespace
{
constexpr float pickRadius = 0.34f;
constexpr float targetRadius = 0.12f;
constexpr float finiteDifferenceDegrees = 0.25f;
constexpr float damping = 0.06f;
constexpr float maximumAngleStep = 3.0f;
constexpr float targetTolerance = 0.008f;
constexpr int solverIterationsPerFrame = 18;

Rectangle GetLockButtonBounds()
{
    return Rectangle { 18.0f, 18.0f, 190.0f, 48.0f };
}

float LengthSquared(const Vector3 &value)
{
    return Vector3DotProduct(value, value);
}

bool IntersectRayPlane(
    const Ray &ray,
    const Vector3 &planePoint,
    const Vector3 &planeNormal,
    Vector3 *intersection)
{
    constexpr float parallelTolerance = 0.00001f;
    const float denominator =
        Vector3DotProduct(ray.direction, planeNormal);

    if (std::fabs(denominator) < parallelTolerance)
    {
        return false;
    }

    const float distance =
        Vector3DotProduct(
            Vector3Subtract(planePoint, ray.position),
            planeNormal) /
        denominator;

    if (distance < 0.0f)
    {
        return false;
    }

    *intersection = Vector3Add(
        ray.position,
        Vector3Scale(ray.direction, distance));
    return true;
}

bool Invert3x3(const float matrix[3][3], float inverse[3][3])
{
    const float determinant =
        matrix[0][0] *
            (matrix[1][1] * matrix[2][2] -
             matrix[1][2] * matrix[2][1]) -
        matrix[0][1] *
            (matrix[1][0] * matrix[2][2] -
             matrix[1][2] * matrix[2][0]) +
        matrix[0][2] *
            (matrix[1][0] * matrix[2][1] -
             matrix[1][1] * matrix[2][0]);

    if (std::fabs(determinant) < 0.0000001f)
    {
        return false;
    }

    const float inverseDeterminant = 1.0f / determinant;

    inverse[0][0] =
        (matrix[1][1] * matrix[2][2] -
         matrix[1][2] * matrix[2][1]) *
        inverseDeterminant;
    inverse[0][1] =
        (matrix[0][2] * matrix[2][1] -
         matrix[0][1] * matrix[2][2]) *
        inverseDeterminant;
    inverse[0][2] =
        (matrix[0][1] * matrix[1][2] -
         matrix[0][2] * matrix[1][1]) *
        inverseDeterminant;
    inverse[1][0] =
        (matrix[1][2] * matrix[2][0] -
         matrix[1][0] * matrix[2][2]) *
        inverseDeterminant;
    inverse[1][1] =
        (matrix[0][0] * matrix[2][2] -
         matrix[0][2] * matrix[2][0]) *
        inverseDeterminant;
    inverse[1][2] =
        (matrix[0][2] * matrix[1][0] -
         matrix[0][0] * matrix[1][2]) *
        inverseDeterminant;
    inverse[2][0] =
        (matrix[1][0] * matrix[2][1] -
         matrix[1][1] * matrix[2][0]) *
        inverseDeterminant;
    inverse[2][1] =
        (matrix[0][1] * matrix[2][0] -
         matrix[0][0] * matrix[2][1]) *
        inverseDeterminant;
    inverse[2][2] =
        (matrix[0][0] * matrix[1][1] -
         matrix[0][1] * matrix[1][0]) *
        inverseDeterminant;

    return true;
}

void NormalizeJointAngle(float *angleDegrees)
{
    *angleDegrees = std::remainder(*angleDegrees, 360.0f);
}

void SolveInverseKinematics(DhRobot *robot, const Vector3 &target)
{
    for (int iteration = 0;
         iteration < solverIterationsPerFrame;
         ++iteration)
    {
        UpdateDhRobot(robot);

        const Vector3 currentPosition =
            GetDhEndEffectorWorldPosition(*robot);
        const Vector3 error =
            Vector3Subtract(target, currentPosition);

        if (LengthSquared(error) <
            targetTolerance * targetTolerance)
        {
            break;
        }

        float jacobian[3][dhJointCount] = {};

        for (std::size_t joint = 0;
             joint < dhJointCount;
             ++joint)
        {
            robot->joints[joint].thetaDegrees +=
                finiteDifferenceDegrees;
            UpdateDhRobot(robot);

            const Vector3 displacedPosition =
                GetDhEndEffectorWorldPosition(*robot);
            const Vector3 derivative = Vector3Scale(
                Vector3Subtract(
                    displacedPosition,
                    currentPosition),
                1.0f / finiteDifferenceDegrees);

            jacobian[0][joint] = derivative.x;
            jacobian[1][joint] = derivative.y;
            jacobian[2][joint] = derivative.z;

            robot->joints[joint].thetaDegrees -=
                finiteDifferenceDegrees;
        }

        UpdateDhRobot(robot);

        float normalMatrix[3][3] = {};

        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                for (std::size_t joint = 0;
                     joint < dhJointCount;
                     ++joint)
                {
                    normalMatrix[row][column] +=
                        jacobian[row][joint] *
                        jacobian[column][joint];
                }
            }

            normalMatrix[row][row] += damping * damping;
        }

        float inverseNormalMatrix[3][3] = {};

        if (!Invert3x3(normalMatrix, inverseNormalMatrix))
        {
            break;
        }

        const float errorValues[3] = {
            error.x,
            error.y,
            error.z
        };
        float weightedError[3] = {};

        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                weightedError[row] +=
                    inverseNormalMatrix[row][column] *
                    errorValues[column];
            }
        }

        for (std::size_t joint = 0;
             joint < dhJointCount;
             ++joint)
        {
            float angleStep = 0.0f;

            for (int axis = 0; axis < 3; ++axis)
            {
                angleStep +=
                    jacobian[axis][joint] *
                    weightedError[axis];
            }

            angleStep = std::clamp(
                angleStep,
                -maximumAngleStep,
                maximumAngleStep);
            robot->joints[joint].thetaDegrees += angleStep;
            NormalizeJointAngle(
                &robot->joints[joint].thetaDegrees);
        }
    }

    UpdateDhRobot(robot);
}
}

InverseKinematicsController CreateInverseKinematicsController(
    const DhRobot &robot)
{
    InverseKinematicsController controller = {};
    controller.locked = true;
    controller.targetPosition =
        GetDhEndEffectorWorldPosition(robot);
    return controller;
}

bool UpdateInverseKinematics(
    InverseKinematicsController *controller,
    DhRobot *robot,
    const Camera3D &camera)
{
    const Vector2 mousePosition = GetMousePosition();
    const Rectangle buttonBounds = GetLockButtonBounds();
    const bool mouseOverButton =
        CheckCollisionPointRec(mousePosition, buttonBounds);

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
        mouseOverButton)
    {
        controller->locked = !controller->locked;
        controller->dragging = false;
        controller->uiMouseCaptured = true;
        controller->targetPosition =
            GetDhEndEffectorWorldPosition(*robot);
    }

    if (controller->uiMouseCaptured &&
        IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
    {
        controller->uiMouseCaptured = false;
    }

    if (controller->locked)
    {
        controller->dragging = false;
        controller->hoveringEndEffector = false;
        return controller->uiMouseCaptured || mouseOverButton;
    }

    const Ray mouseRay =
        GetMouseRay(mousePosition, camera);
    const Vector3 endEffectorPosition =
        GetDhEndEffectorWorldPosition(*robot);
    const RayCollision endEffectorHit =
        GetRayCollisionSphere(
            mouseRay,
            endEffectorPosition,
            pickRadius);

    controller->hoveringEndEffector =
        endEffectorHit.hit || controller->dragging;

    if (!controller->uiMouseCaptured &&
        IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
        endEffectorHit.hit)
    {
        controller->dragging = true;
        controller->dragPlanePoint = endEffectorPosition;
        controller->dragPlaneNormal = Vector3Normalize(
            Vector3Subtract(camera.target, camera.position));

        Vector3 planeIntersection = {};

        if (IntersectRayPlane(
                mouseRay,
                controller->dragPlanePoint,
                controller->dragPlaneNormal,
                &planeIntersection))
        {
            controller->dragOffset = Vector3Subtract(
                endEffectorPosition,
                planeIntersection);
        }
        else
        {
            controller->dragOffset = {};
        }
    }

    if (controller->dragging &&
        IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
        Vector3 planeIntersection = {};

        if (IntersectRayPlane(
                mouseRay,
                controller->dragPlanePoint,
                controller->dragPlaneNormal,
                &planeIntersection))
        {
            controller->targetPosition = Vector3Add(
                planeIntersection,
                controller->dragOffset);
            SolveInverseKinematics(
                robot,
                controller->targetPosition);
        }
    }

    if (controller->dragging &&
        IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
    {
        controller->dragging = false;
    }

    return controller->uiMouseCaptured ||
           controller->dragging ||
           mouseOverButton;
}

void DrawInverseKinematics3D(
    const InverseKinematicsController &controller,
    const DhRobot &robot)
{
    if (controller.locked)
    {
        return;
    }

    const Vector3 endEffectorPosition =
        GetDhEndEffectorWorldPosition(robot);
    const Color selectionColor =
        controller.hoveringEndEffector ? YELLOW : ORANGE;

    DrawSphereWires(
        endEffectorPosition,
        pickRadius,
        12,
        12,
        selectionColor);

    if (controller.dragging)
    {
        DrawLine3D(
            endEffectorPosition,
            controller.targetPosition,
            Color { 255, 170, 0, 180 });
        DrawSphereWires(
            controller.targetPosition,
            targetRadius,
            8,
            8,
            YELLOW);
    }
}

void DrawInverseKinematicsUi(
    const InverseKinematicsController &controller)
{
    const Rectangle buttonBounds = GetLockButtonBounds();
    const bool hovered =
        CheckCollisionPointRec(GetMousePosition(), buttonBounds);
    const Color buttonColor = controller.locked
        ? Color { 204, 76, 76, 255 }
        : Color { 61, 153, 92, 255 };
    const Color displayedColor = hovered
        ? ColorBrightness(buttonColor, 0.12f)
        : buttonColor;

    DrawRectangleRounded(
        buttonBounds,
        0.22f,
        10,
        displayedColor);
    DrawRectangleRoundedLinesEx(
        buttonBounds,
        0.22f,
        10,
        1.5f,
        ColorBrightness(displayedColor, -0.25f));

    const char *buttonText =
        controller.locked ? "DESTRAVAR ROBO" : "TRAVAR ROBO";
    const float buttonFontSize = 18.0f;
    const Vector2 textSize = MeasureTextEx(
        GetUiFont(),
        buttonText,
        buttonFontSize,
        0.5f);

    DrawUiText(
        buttonText,
        buttonBounds.x +
            (buttonBounds.width - textSize.x) * 0.5f,
        buttonBounds.y +
            (buttonBounds.height - textSize.y) * 0.5f,
        buttonFontSize,
        WHITE);

    if (!controller.locked)
    {
        DrawUiText(
            controller.dragging
                ? "Movendo o orgao terminal..."
                : "Clique e arraste o orgao terminal",
            buttonBounds.x,
            buttonBounds.y + buttonBounds.height + 8.0f,
            15.0f,
            DARKGRAY);
    }
}
