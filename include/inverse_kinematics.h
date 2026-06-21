#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "denavit_hartenberg.h"
#include "raylib.h"

struct InverseKinematicsController
{
    bool locked;
    bool dragging;
    bool hoveringEndEffector;
    bool uiMouseCaptured;
    Vector3 targetPosition;
    Vector3 dragPlanePoint;
    Vector3 dragPlaneNormal;
    Vector3 dragOffset;
};

InverseKinematicsController CreateInverseKinematicsController(
    const DhRobot &robot);
bool UpdateInverseKinematics(
    InverseKinematicsController *controller,
    DhRobot *robot,
    const Camera3D &camera);
void DrawInverseKinematics3D(
    const InverseKinematicsController &controller,
    const DhRobot &robot);
void DrawInverseKinematicsUi(
    const InverseKinematicsController &controller);

#endif
