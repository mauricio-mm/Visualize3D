#include "ui.h"

namespace
{
Font uiFont = {};
bool uiFontMustBeUnloaded = false;
}

void InitUi()
{
    const char *fontPath = nullptr;

    if (FileExists("C:/Windows/Fonts/segoeui.ttf"))
    {
        fontPath = "C:/Windows/Fonts/segoeui.ttf";
    }
    else if (FileExists("C:/Windows/Fonts/arial.ttf"))
    {
        fontPath = "C:/Windows/Fonts/arial.ttf";
    }

    if (fontPath != nullptr)
    {
        uiFont = LoadFontEx(fontPath, 32, nullptr, 0);
        uiFontMustBeUnloaded =
            IsFontValid(uiFont) &&
            uiFont.texture.id != GetFontDefault().texture.id;
    }

    if (!IsFontValid(uiFont))
    {
        uiFont = GetFontDefault();
        uiFontMustBeUnloaded = false;
    }

    SetTextureFilter(uiFont.texture, TEXTURE_FILTER_BILINEAR);
}

void ShutdownUi()
{
    if (uiFontMustBeUnloaded)
    {
        UnloadFont(uiFont);
    }

    uiFont = {};
    uiFontMustBeUnloaded = false;
}

Font GetUiFont()
{
    if (IsFontValid(uiFont))
    {
        return uiFont;
    }

    return GetFontDefault();
}

void DrawUiText(
    const char *text,
    float x,
    float y,
    float fontSize,
    Color color)
{
    DrawTextEx(
        GetUiFont(),
        text,
        Vector2 { x, y },
        fontSize,
        0.5f,
        color);
}
