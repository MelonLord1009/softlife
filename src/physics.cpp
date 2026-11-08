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


void World::apply_fluid_drag(){
    for(Node& node : nodes){
        float vel = node.vel.length();
        Vec2 F_drag = -node.vel * vel * viscosity;
        node.force += F_drag;
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
    apply_fluid_drag();
    integrate_nodes(dt);
    resolve_boundaries();
}


void World::spawn_creature(Vec2 center){
    int start_idx = nodes.size();
    Node n0; n0.pos = {center.x - 25.0f, center.y - 25.0f}; n0.vel = {2000.0f, 1500.0f};
    Node n1; n1.pos = {center.x + 25.0f, center.y - 25.0f};
    Node n2; n1.pos = {center.x - 25.0f, center.y + 25.0f};
    Node n3; n1.pos = {center.x + 25.0f, center.y + 25.0f};

    nodes.push_back(n0);
    nodes.push_back(n1);
    nodes.push_back(n2);
    nodes.push_back(n3);

    springs.push_back({start_idx + 0, start_idx + 1, 100.0f, 50.0f, 2.0f}); 
    springs.push_back({start_idx + 1, start_idx + 3, 100.0f, 50.0f, 2.0f}); 
    springs.push_back({start_idx + 3, start_idx + 2, 100.0f, 50.0f, 2.0f}); 
    springs.push_back({start_idx + 2, start_idx + 0, 100.0f, 50.0f, 2.0f}); 
}