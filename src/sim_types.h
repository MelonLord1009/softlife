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
    float mass = 1.0f;
    float radius = 6.0f;
};

struct Spring{
    int a;
    int b;
    float rest_len;
    float stiffness;
    float damping;
};

struct World{
    std::vector<Node> nodes;
    std::vector<Spring> springs;
    void step(float dt);
private:
    void apply_spring_forces();
    void integrate_nodes(float dt);
    void resolve_boundaries();
};
