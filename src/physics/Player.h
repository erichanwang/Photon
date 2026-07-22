#ifndef PLAYER_H
#define PLAYER_H

#include "../math/Vector3D.h"
#include "InputState.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

class Player {
public:
    Vector3D position;
    Vector3D velocity;
    bool onGround;
    float mass;
    float jumpSpeed;
    float moveSpeed;

    Player() : position(0, 1.8, 0), velocity(0, 0, 0), onGround(false), mass(1.0f), jumpSpeed(5.0f), moveSpeed(5.0f) {}

    void update(float dt, float gravity = -9.81f) {
        velocity.y += gravity * dt;
        position += velocity * dt;
        if (position.y <= 0) {
            position.y = 0;
            velocity.y = 0;
            onGround = true;
        } else {
            onGround = false;
        }
    }

    void jump() {
        if (onGround) {
            velocity.y = jumpSpeed;
            onGround = false;
        }
    }

    void move(const Vector3D& direction, float dt) {
        Vector3D dir = direction;
        dir.y = 0;
        if (dir.length() > 0) dir = dir.normalize();
        position += dir * moveSpeed * dt;
    }

    // The control loop, decoupled from any input source: whatever produced
    // this InputState (a real keyboard via raylib, or a scripted sequence in
    // a headless test/demo) drives move/jump/gravity identically. `yaw`
    // is the camera's facing so "forward" means the direction the player is
    // looking, matching Camera's own yaw convention.
    void applyInput(const InputState& input, float dt, float yaw = -static_cast<float>(M_PI) / 2.0f) {
        if (input.jump) jump();

        Vector3D forward(cos(yaw), 0, sin(yaw));
        Vector3D right(-sin(yaw), 0, cos(yaw));
        Vector3D dir = forward * input.moveForward + right * input.moveRight;
        move(dir, dt);

        update(dt);
    }
};

#endif // PLAYER_H
