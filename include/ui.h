#ifndef UI_H
#define UI_H

#include "raylib.h"

void InitUi();
void ShutdownUi();
Font GetUiFont();
void DrawUiText(
    const char *text,
    float x,
    float y,
    float fontSize,
    Color color);

#endif
