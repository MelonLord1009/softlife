#include "raylib.h"

int main() {
    InitWindow(1280, 720, "SoftLife Sandbox");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground({18, 22, 28, 255}); // Dark slate background (R, G, B, A)
        DrawCircleV(GetMousePosition(), 12.0f, SKYBLUE);
        DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}