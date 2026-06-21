#include "denavit_hartenberg.h"

#include <algorithm>
#include <cmath>

#include "ui.h"

namespace
{
constexpr float jointRadius = 0.16f;
constexpr float linkRadius = 0.065f;
constexpr float robotLinkLength = 1.5f;
constexpr int linkSides = 12;

HomogeneousMatrix IdentityMatrix()
{
    return {
        {
            { 1.0f, 0.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 0.0f, 1.0f }
        }
    };
}

HomogeneousMatrix Multiply(
    const HomogeneousMatrix &left,
    const HomogeneousMatrix &right)
{
    HomogeneousMatrix result = {};

    for (std::size_t row = 0; row < 4; ++row)
    {
        for (std::size_t column = 0; column < 4; ++column)
        {
            for (std::size_t index = 0; index < 4; ++index)
            {
                result.value[row][column] +=
                    left.value[row][index] * right.value[index][column];
            }
        }
    }

    return result;
}

HomogeneousMatrix CreateDhTransform(const DhParameters &parameters)
{
    const float alpha = parameters.alphaDegrees * DEG2RAD;
    const float theta = parameters.thetaDegrees * DEG2RAD;
    const float cosineAlpha = std::cos(alpha);
    const float sineAlpha = std::sin(alpha);
    const float cosineTheta = std::cos(theta);
    const float sineTheta = std::sin(theta);

    return {
        {
            {
                cosineTheta,
                -sineTheta * cosineAlpha,
                sineTheta * sineAlpha,
                parameters.a * cosineTheta
            },
            {
                sineTheta,
                cosineTheta * cosineAlpha,
                -cosineTheta * sineAlpha,
                parameters.a * sineTheta
            },
            {
                0.0f,
                sineAlpha,
                cosineAlpha,
                parameters.d
            },
            { 0.0f, 0.0f, 0.0f, 1.0f }
        }
    };
}

Vector3 GetDhPosition(const HomogeneousMatrix &transform)
{
    return {
        transform.value[0][3],
        transform.value[1][3],
        transform.value[2][3]
    };
}

Vector3 DhToWorld(const Vector3 &position)
{
    return { position.x, position.z, position.y };
}

void DrawMatrixSection(
    const char *name,
    const HomogeneousMatrix &matrix,
    int panelX,
    int sectionY,
    float fontScale)
{
    const float matrixFontSize = 17.0f * fontScale;
    const int rowHeight =
        static_cast<int>(20.0f * fontScale);
    const int columnWidth =
        static_cast<int>(88.0f * fontScale);

    const Vector3 position = GetDhPosition(matrix);
    DrawUiText(
        name,
        static_cast<float>(panelX + 20),
        static_cast<float>(sectionY),
        17.0f * fontScale,
        BLACK);
    DrawUiText(
        TextFormat(
            "p = (% .3f, % .3f, % .3f)",
            position.x,
            position.y,
            position.z),
        static_cast<float>(
            panelX + static_cast<int>(170.0f * fontScale)),
        static_cast<float>(sectionY + 1),
        14.0f * fontScale,
        DARKGRAY);

    const int matrixY =
        sectionY + static_cast<int>(25.0f * fontScale);

    for (std::size_t row = 0; row < 4; ++row)
    {
        for (std::size_t column = 0; column < 4; ++column)
        {
            DrawUiText(
                TextFormat("% .3f", matrix.value[row][column]),
                static_cast<float>(
                    panelX + 22 +
                    static_cast<int>(column) * columnWidth),
                static_cast<float>(
                    matrixY + static_cast<int>(row) * rowHeight),
                matrixFontSize,
                BLACK);
        }
    }
}
}

DhRobot CreateDhRobot()
{
    DhRobot robot = {};

    // a, alpha, d e theta seguem a convencao DH classica.
    robot.joints = {
        DhParameters { 0.0f, 90.0f, robotLinkLength, 35.0f },
        DhParameters { robotLinkLength, 0.0f, 0.0f, 25.0f },
        DhParameters { robotLinkLength, 0.0f, 0.0f, -45.0f },
        DhParameters { 0.0f, 90.0f, robotLinkLength, 30.0f },
        DhParameters { robotLinkLength, -90.0f, 0.0f, 20.0f },
        DhParameters { 0.0f, 0.0f, robotLinkLength, -25.0f }
    };
    robot.toolLength = 0.7f;

    UpdateDhRobot(&robot);
    return robot;
}

void UpdateDhRobot(DhRobot *robot)
{
    HomogeneousMatrix cumulativeTransform = IdentityMatrix();

    for (std::size_t index = 0; index < dhJointCount; ++index)
    {
        cumulativeTransform = Multiply(
            cumulativeTransform,
            CreateDhTransform(robot->joints[index]));
        robot->jointTransforms[index] = cumulativeTransform;
    }

    const DhParameters toolParameters = {
        robot->toolLength,
        0.0f,
        0.0f,
        0.0f
    };
    robot->endEffectorTransform = Multiply(
        cumulativeTransform,
        CreateDhTransform(toolParameters));
}

Vector3 GetDhEndEffectorWorldPosition(const DhRobot &robot)
{
    return DhToWorld(GetDhPosition(robot.endEffectorTransform));
}

void DrawDhRobot3D(const DhRobot &robot)
{
    Vector3 previousPosition = { 0.0f, 0.0f, 0.0f };
    DrawSphere(previousPosition, jointRadius, BLACK);

    for (const HomogeneousMatrix &transform : robot.jointTransforms)
    {
        const Vector3 jointPosition =
            DhToWorld(GetDhPosition(transform));

        DrawCylinderEx(
            previousPosition,
            jointPosition,
            linkRadius,
            linkRadius,
            linkSides,
            DARKGRAY);
        DrawSphere(jointPosition, jointRadius, BLACK);
        previousPosition = jointPosition;
    }

    const Vector3 endEffectorPosition =
        GetDhEndEffectorWorldPosition(robot);

    DrawCylinderEx(
        previousPosition,
        endEffectorPosition,
        linkRadius,
        linkRadius,
        linkSides,
        DARKGRAY);
    DrawSphere(endEffectorPosition, jointRadius * 1.15f, ORANGE);
}

void DrawDhRobotMatrices(const DhRobot &robot)
{
    constexpr int panelMargin = 14;
    constexpr int sectionCount =
        static_cast<int>(dhJointCount) + 1;

    const float fontScale = std::clamp(
        static_cast<float>(GetScreenHeight()) / 1080.0f,
        0.88f,
        1.15f);
    const int panelWidth =
        static_cast<int>(440.0f * fontScale);
    const int headerHeight =
        static_cast<int>(56.0f * fontScale);
    const int panelHeight =
        GetScreenHeight() - panelMargin * 2;
    const int sectionHeight =
        (panelHeight - headerHeight) / sectionCount;

    const int panelX = GetScreenWidth() - panelWidth - panelMargin;
    const Rectangle panel = {
        static_cast<float>(panelX),
        static_cast<float>(panelMargin),
        static_cast<float>(panelWidth),
        static_cast<float>(GetScreenHeight() - panelMargin * 2)
    };

    DrawRectangleRounded(
        panel,
        0.025f,
        8,
        Color { 248, 248, 248, 235 });
    DrawRectangleRoundedLinesEx(
        panel,
        0.025f,
        8,
        1.0f,
        Color { 205, 205, 205, 255 });

    DrawUiText(
        "Matrizes homogeneas DH",
        static_cast<float>(panelX + 20),
        static_cast<float>(panelMargin + 14),
        25.0f * fontScale,
        BLACK);

    int sectionY = panelMargin + headerHeight;

    for (std::size_t index = 0; index < dhJointCount; ++index)
    {
        DrawMatrixSection(
            TextFormat("Junta %i  (T0%i)",
                static_cast<int>(index + 1),
                static_cast<int>(index + 1)),
            robot.jointTransforms[index],
            panelX,
            sectionY,
            fontScale);
        sectionY += sectionHeight;
    }

    DrawMatrixSection(
        "Orgao terminal  (T0E)",
        robot.endEffectorTransform,
        panelX,
        sectionY,
        fontScale);
}
