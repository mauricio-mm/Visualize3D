#ifndef DENAVIT_HARTENBERG_H
#define DENAVIT_HARTENBERG_H

#include <array>
#include <cstddef>

#include "raylib.h"

constexpr std::size_t dhJointCount = 6;

struct DhParameters
{
    float a;
    float alphaDegrees;
    float d;
    float thetaDegrees;
};

struct HomogeneousMatrix
{
    float value[4][4];
};

struct DhRobot
{
    std::array<DhParameters, dhJointCount> joints;
    std::array<HomogeneousMatrix, dhJointCount> jointTransforms;
    HomogeneousMatrix endEffectorTransform;
    float toolLength;
};

DhRobot CreateDhRobot();
void UpdateDhRobot(DhRobot *robot);
Vector3 GetDhEndEffectorWorldPosition(const DhRobot &robot);
void DrawDhRobot3D(const DhRobot &robot);
void DrawDhRobotMatrices(const DhRobot &robot);

#endif
