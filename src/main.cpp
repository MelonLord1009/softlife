#include "raylib.h"
#include "sim_types.h"

void World::apply_spring_forces(){
    for(Spring& spring : springs){
        Node& node_a = nodes[spring.a];
        Node& node_b = nodes[spring.b];
        Vec2 spring_v = node_b.pos - node_a.pos;
        float spring_l = spring_v.length();
        Vec2 v_rel = node_b.vel - node_a.vel;
        if(spring_l > 0.00001f){
            Vec2 spring_uv = spring_v/spring_l;
            float F_hooke = (spring_l - spring.rest_len) * spring.stiffness;
            float F_damp = (v_rel.dot(spring_uv)) * spring.damping;
            Vec2 F_tot = spring_uv * (F_hooke + F_damp);
            node_a.force += F_tot;
            node_b.force -= F_tot;
        }
    }
}

void World::integrate_nodes(float dt){
    for(Node& node : nodes){
        Vec2 a = node.force/node.mass;
        node.vel += a * dt;
        node.pos += node.vel * dt;
        node.force = Vec2{0.0f, 0.0f};
    }    
}

void World::resolve_boundaries(){
    for(Node& node : nodes){
        if(node.pos.x - node.radius < 0.0f){
            node.pos.x = node.radius;
            node.vel.x *= -0.9f;
        }
        if(node.pos.y - node.radius < 0.0f){
            node.pos.y = node.radius;
            node.vel.y *= -0.9f;
        }
        if(node.pos.x + node.radius > 1280.0f){
            node.pos.x = 1280.0f - node.radius;
            node.vel.x *= -0.9f;
        }
        if(node.pos.y + node.radius > 720.0f){
            node.pos.y = 720.0f - node.radius;
            node.vel.y *= -0.9f;
        }
    }
}

void World::step(float dt){
    apply_spring_forces();
    integrate_nodes(dt);
    resolve_boundaries();
}

int main() {
    InitWindow(1280, 720, "SoftLife Sandbox");
    SetTargetFPS(60);

    World world;

    Node n0; n0.pos = {500.0f, 300.0f};
    Node n1; n1.pos = {600.0f, 300.0f};
    Node n2; n2.pos = {500.0f, 400.0f};
    Node n3; n3.pos = {600.0f, 400.0f};

    world.nodes.push_back(n0);
    world.nodes.push_back(n1);
    world.nodes.push_back(n2);
    world.nodes.push_back(n3);

    world.springs.push_back({0, 1, 100.0f, 50.0f, 2.0f}); 
    world.springs.push_back({1, 3, 100.0f, 50.0f, 2.0f}); 
    world.springs.push_back({3, 2, 100.0f, 50.0f, 2.0f}); 
    // world.springs.push_back({2, 0, 100.0f, 50.0f, 2.0f}); 
    world.springs.push_back({0, 3, 141.4f, 50.0f, 2.0f}); 
    // world.springs.push_back({1, 2, 141.4f, 50.0f, 2.0f}); 

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