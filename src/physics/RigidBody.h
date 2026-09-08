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

    // Collision shape. Sphere (the default) uses `radius`; Box uses
    // `halfExtents` and resolves against its faces instead of a bounding
    // sphere, so a Block can rest flat rather than wobble on its corner.
    enum class Shape { Sphere, Box };
    Shape shape = Shape::Sphere;
    double radius = 0.5;
    Vector3D halfExtents = Vector3D(0.5, 0.5, 0.5); // used when shape == Box
    // Bounciness of a collision: 0 keeps no normal velocity (a dead stop),
    // 1 reverses it entirely (a perfectly elastic bounce).
    double restitution = 0.4;
    // Infinite-mass bodies never move; they act as immovable obstacles.
    bool isStatic = false;

    RigidBody(const Vector3D& pos, double m, double drag = 0.0) : position(pos), velocity(0,0,0), acceleration(0,0,0), mass(m), dragCoeff(drag) {}

    double inverseMass() const { return (isStatic || mass <= 0.0) ? 0.0 : 1.0 / mass; }

    // Radius of the sphere that fully encloses this body, used by the broad
    // phase regardless of shape. For a Box this is the half-diagonal, which
    // over-approximates the box; that's fine (and required) for a broad
    // phase, whose job is to never miss a pair, not to be tight.
    double boundingRadius() const {
        return shape == Shape::Box ? halfExtents.length() : radius;
    }

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
