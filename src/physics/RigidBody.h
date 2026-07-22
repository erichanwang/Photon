#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "../math/Vector3D.h"

class RigidBody {
public:
    Vector3D position;
    Vector3D velocity;
    Vector3D acceleration;
    double mass;
    double dragCoeff; // Drag coefficient for air resistance

    // Collision shape. Every body is treated as a sphere of this radius.
    // ponytail: sphere-only broad and narrow phase; give Object a collider
    // interface if boxes ever need to rest on their faces rather than wobble
    // on a bounding sphere.
    double radius = 0.5;
    // Bounciness of a collision: 0 keeps no normal velocity (a dead stop),
    // 1 reverses it entirely (a perfectly elastic bounce).
    double restitution = 0.4;
    // Infinite-mass bodies never move; they act as immovable obstacles.
    bool isStatic = false;

    RigidBody(const Vector3D& pos, double m, double drag = 0.0) : position(pos), velocity(0,0,0), acceleration(0,0,0), mass(m), dragCoeff(drag) {}

    double inverseMass() const { return (isStatic || mass <= 0.0) ? 0.0 : 1.0 / mass; }

    void applyForce(const Vector3D& force) {
        acceleration = acceleration + force / mass;
    }

    void update(double dt) {
        // Apply drag force: F_drag = -dragCoeff * velocity
        Vector3D dragForce = velocity * (-dragCoeff);
        applyForce(dragForce);

        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;
        acceleration = Vector3D(0,0,0); // Reset acceleration
    }
};

#endif // RIGIDBODY_H
