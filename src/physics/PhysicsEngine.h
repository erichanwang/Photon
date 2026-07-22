#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include "RigidBody.h"
#include "../math/Vector3D.h"

class PhysicsEngine {
public:
    std::vector<RigidBody*> bodies;
    const double gravity = 9.81; // m/s^2

    // Collision response. Previously absent entirely, which is why bodies in
    // the demo scenes sank through the floor once the simulation ran long
    // enough.
    bool collisionsEnabled = true;
    bool hasGround = true;
    double groundY = 0.0;

    void addBody(RigidBody* body) {
        bodies.push_back(body);
    }

    void update(double dt) {
        for (auto body : bodies) {
            if (body->isStatic) continue;
            // Apply gravity
            body->applyForce(Vector3D(0, -body->mass * gravity, 0));
            body->update(dt);
        }
        if (collisionsEnabled) resolveCollisions();
    }

    void resolveCollisions() {
        if (hasGround)
            for (auto body : bodies) resolveGround(body);

        // ponytail: O(n^2) pair test, fine for the tens of bodies these demos
        // use; add a uniform grid or reuse the renderer's BVH past a few
        // hundred.
        for (size_t i = 0; i < bodies.size(); i++)
            for (size_t j = i + 1; j < bodies.size(); j++)
                resolvePair(bodies[i], bodies[j]);
    }

private:
    // A body resting on the ground plane at groundY.
    void resolveGround(RigidBody* body) {
        if (body->isStatic) return;
        double penetration = (groundY + body->radius) - body->position.y;
        if (penetration <= 0.0) return;

        body->position.y = groundY + body->radius;   // push out of the floor
        if (body->velocity.y < 0.0) {
            body->velocity.y = -body->velocity.y * body->restitution;
            // Without this, a body bounces forever by ever-smaller amounts and
            // visibly jitters against the floor instead of coming to rest.
            if (std::fabs(body->velocity.y) < 0.1) body->velocity.y = 0.0;
        }
    }

    // Impulse-based response between two spheres along the contact normal.
    void resolvePair(RigidBody* a, RigidBody* b) {
        double invA = a->inverseMass(), invB = b->inverseMass();
        double invSum = invA + invB;
        if (invSum == 0.0) return;   // two static bodies: nothing to resolve

        Vector3D delta = b->position - a->position;
        double distance = delta.length();
        double contact = a->radius + b->radius;
        if (distance >= contact) return;

        // Exactly coincident centers give no usable normal; pick one so the
        // pair still separates instead of producing NaN.
        Vector3D normal = (distance > 1e-9) ? delta / distance : Vector3D(0, 1, 0);
        double penetration = contact - (distance > 1e-9 ? distance : 0.0);

        // Positional correction split in proportion to inverse mass, so a light
        // body moves out of a heavy one rather than shoving it aside.
        a->position -= normal * (penetration * (invA / invSum));
        b->position += normal * (penetration * (invB / invSum));

        Vector3D relativeVelocity = b->velocity - a->velocity;
        double separating = relativeVelocity.dot(normal);
        if (separating > 0.0) return;   // already moving apart

        double restitution = std::min(a->restitution, b->restitution);
        double impulse = -(1.0 + restitution) * separating / invSum;

        a->velocity -= normal * (impulse * invA);
        b->velocity += normal * (impulse * invB);
    }
};

#endif // PHYSICSENGINE_H
