#include "raylib.h"
#include "sim_types.h"


int main() {
    InitWindow(1280, 720, "SoftLife Sandbox");
    SetTargetFPS(60);

    World world;

    world.spawn_creature({300.0f, 400.0f});

    int grabbed_node = -1;

    while (!WindowShouldClose()) {

        Vector2 m = GetMousePosition();
        Vec2 mouse_pos(m.x, m.y);

        
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            for (int i = 0; i < world.nodes.size(); i++) {
                Vec2 diff = world.nodes[i].pos - mouse_pos;
                if (diff.length() < 15.0f) { 
                    grabbed_node = i;
                    break; 
                }
            }
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && grabbed_node != -1) {
            world.nodes[grabbed_node].pos = mouse_pos;
            world.nodes[grabbed_node].vel = Vec2(0.0f, 0.0f); 
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            grabbed_node = -1;
        }
        world.step(GetFrameTime());
        BeginDrawing();
        ClearBackground({18, 22, 28, 255}); // Dark slate background (R, G, B, A)

        for(const Spring& spring: world.springs){
            Vector2 start = {world.nodes[spring.a].pos.x, world.nodes[spring.a].pos.y};
            Vector2 end = {world.nodes[spring.b].pos.x, world.nodes[spring.b].pos.y};
            DrawLineV(start, end, RAYWHITE);
        }

        for(const Node& node : world.nodes){
            Vector2 center = {node.pos.x, node.pos.y};
            DrawCircleV(center, node.radius, BLUE);
        }

        DrawCircleV(GetMousePosition(), 6.0f, SKYBLUE);
        DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}