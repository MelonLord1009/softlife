#pragma once
#include "cmath"
#include "vector"

struct Vec2{

    float x ;
    float y ;

    Vec2(float x_cord = 0.0f, float y_cord = 0.0f) : x(x_cord), y(y_cord) {}
    
    Vec2 operator+(const Vec2& other) const{
        return Vec2{x + other.x, y + other.y};
    }

    Vec2& operator+=(const Vec2& other){
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2 operator-(const Vec2& other) const{
        return Vec2{x - other.x, y - other.y};
    }

    Vec2& operator-=(const Vec2& other){
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2 operator-() const{
        return {-x, -y};
    }

    Vec2 operator*(float f) const{
        return Vec2{f*x, f*y};
    }

    Vec2& operator*=(float f){
        x *= f;
        y *= f;
        return *this;
    }

    Vec2 operator/(float f) const{
        float inv = 1.0f/f;
        return Vec2{x * inv, y * inv};
    }

    float dot(const Vec2& other) const{
        return (x*other.x)+(y*other.y);
    }

    float length() const{
        return std::sqrt(x*x + y*y);
    }
};

struct Node{
    Vec2 pos;
    Vec2 vel;
    Vec2 force;
    int group_id = 0;
    float mass = 1.0f;
    float radius = 5.0f;
};

struct Spring{
    int a;
    int b;
    
    float rest_len;
    float base_len;
    float stiffness;
    float damping;

    float amplitude = 0.0f;
    float frequency = 0.0f;
    float phase = 0.0f;
};

struct World{

    float global_time = 0.0f;

    int n_creatures = 0;
    float viscosity = 0.0001f;
    std::vector<Node> nodes;
    std::vector<Spring> springs;

    float cell_size = 10.0f;
    int grid_rows = 1280 / 20;
    int grid_cols = 720 / 20;
    std::vector<std::vector<int>> grid;

    World(){
        grid.resize(grid_cols * grid_rows);
    }

    void step(float dt);
    void spawn_creature(Vec2 center);
    void actuate_muscles(float dt);
private:
    void apply_spring_forces();
    void apply_fluid_drag();
    void integrate_nodes(float dt);
    void build_spatial_mesh();
    void resolve_node_collisions();
    void resolve_boundaries();
    
};
