#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include <vector>
#include <cmath>
#include <algorithm>
#include "RigidBody.h"
#include "BroadPhase.h"
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

    // Broad phase: a spatial hash replacing the old O(n^2) pair loop. Cell
    // size defaults to a couple of these demos' body sizes; see
    // bench/broadphase_bench.cpp for where this actually starts winning.
    SpatialHashBroadPhase broadPhase{2.0};

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

        for (auto& pair : broadPhase.findPairs(bodies))
            resolvePair(bodies[pair.first], bodies[pair.second]);
    }

private:
    // A body resting on the ground plane at groundY. A sphere's lowest point
    // is `radius` below its center; a box's is `halfExtents.y` below.
    void resolveGround(RigidBody* body) {
        if (body->isStatic) return;
        double bottom = (body->shape == RigidBody::Shape::Box) ? body->halfExtents.y : body->radius;
        double penetration = (groundY + bottom) - body->position.y;
        if (penetration <= 0.0) return;

        body->position.y = groundY + bottom;   // push out of the floor
        if (body->velocity.y < 0.0) {
            body->velocity.y = -body->velocity.y * body->restitution;
            // Without this, a body bounces forever by ever-smaller amounts and
            // visibly jitters against the floor instead of coming to rest.
            if (std::fabs(body->velocity.y) < 0.1) body->velocity.y = 0.0;
        }
    }

    // Shared impulse + positional correction, once a (normal, penetration)
    // contact has been found by whichever shape pair produced it. `normal`
    // points from a toward b.
    void applyContact(RigidBody* a, RigidBody* b, const Vector3D& normal, double penetration) {
        double invA = a->inverseMass(), invB = b->inverseMass();
        double invSum = invA + invB;
        if (invSum == 0.0) return;

        a->position -= normal * (penetration * (invA / invSum));
        b->position += normal * (penetration * (invB / invSum));

        Vector3D relativeVelocity = b->velocity - a->velocity;
        double separating = relativeVelocity.dot(normal);
        if (separating > 0.0) return;   // already moving apart

        // Below this closing speed the contact is resting, not an impact.
        // One step of gravity is reintroduced every frame, so bouncing it back
        // with any restitution at all leaves a body buzzing against whatever
        // it is sitting on forever, never settling. resolveGround already
        // drops the same way for the same reason.
        double restitution = std::min(a->restitution, b->restitution);
        if (std::fabs(separating) < 0.1) restitution = 0.0;
        double impulse = -(1.0 + restitution) * separating / invSum;

        a->velocity -= normal * (impulse * invA);
        b->velocity += normal * (impulse * invB);
    }

    // Impulse-based response along the contact normal, dispatched by shape.
    void resolvePair(RigidBody* a, RigidBody* b) {
        if (a->inverseMass() == 0.0 && b->inverseMass() == 0.0) return;

        bool aBox = a->shape == RigidBody::Shape::Box;
        bool bBox = b->shape == RigidBody::Shape::Box;

        if (!aBox && !bBox) {
            resolveSphereSphere(a, b);
        } else if (aBox && bBox) {
            resolveBoxBox(a, b);
        } else if (aBox) {
            resolveSphereBox(b, a);
        } else {
            resolveSphereBox(a, b);
        }
    }

    void resolveSphereSphere(RigidBody* a, RigidBody* b) {
        Vector3D delta = b->position - a->position;
        double distance = delta.length();
        double contact = a->radius + b->radius;
        if (distance >= contact) return;

        // Exactly coincident centers give no usable normal; pick one so the
        // pair still separates instead of producing NaN.
        Vector3D normal = (distance > 1e-9) ? delta / distance : Vector3D(0, 1, 0);
        double penetration = contact - (distance > 1e-9 ? distance : 0.0);
        applyContact(a, b, normal, penetration);
    }

    // Sphere vs. axis-aligned box, contact normal from the closest point on
    // the box to the sphere center - not from center-to-center, which would
    // be wrong the moment the box isn't itself sphere-shaped (a sphere
    // resting on an off-center point of a wide flat box must push straight
    // up off that face, not back toward the box's center).
    void resolveSphereBox(RigidBody* sphere, RigidBody* box) {
        Vector3D min = box->position - box->halfExtents;
        Vector3D max = box->position + box->halfExtents;
        Vector3D closest(
            std::clamp(sphere->position.x, min.x, max.x),
            std::clamp(sphere->position.y, min.y, max.y),
            std::clamp(sphere->position.z, min.z, max.z));

        Vector3D delta = sphere->position - closest;
        double distance = delta.length();

        Vector3D normal;
        double penetration;
        if (distance > 1e-9) {
            if (distance >= sphere->radius) return;
            normal = delta / distance;
            penetration = sphere->radius - distance;
        } else {
            // Sphere center is inside the box: push out through whichever
            // face is nearest rather than leaving penetration undefined.
            double dx = std::min(sphere->position.x - min.x, max.x - sphere->position.x);
            double dy = std::min(sphere->position.y - min.y, max.y - sphere->position.y);
            double dz = std::min(sphere->position.z - min.z, max.z - sphere->position.z);
            if (dx <= dy && dx <= dz)
                normal = Vector3D(sphere->position.x < box->position.x ? -1 : 1, 0, 0);
            else if (dy <= dz)
                normal = Vector3D(0, sphere->position.y < box->position.y ? -1 : 1, 0);
            else
                normal = Vector3D(0, 0, sphere->position.z < box->position.z ? -1 : 1);
            penetration = sphere->radius + std::min({dx, dy, dz});
        }

        // applyContact takes (a, b) with normal pointing a -> b; here normal
        // points box -> sphere, so pass (box, sphere).
        applyContact(box, sphere, normal, penetration);
    }

    // Axis-aligned box vs. axis-aligned box: separate along whichever axis
    // has the least overlap, so a box resolves through the face it actually
    // hit rather than a corner.
    void resolveBoxBox(RigidBody* a, RigidBody* b) {
        Vector3D aMin = a->position - a->halfExtents, aMax = a->position + a->halfExtents;
        Vector3D bMin = b->position - b->halfExtents, bMax = b->position + b->halfExtents;

        double overlapX = std::min(aMax.x, bMax.x) - std::max(aMin.x, bMin.x);
        double overlapY = std::min(aMax.y, bMax.y) - std::max(aMin.y, bMin.y);
        double overlapZ = std::min(aMax.z, bMax.z) - std::max(aMin.z, bMin.z);
        if (overlapX <= 0.0 || overlapY <= 0.0 || overlapZ <= 0.0) return;

        // applyContact expects the normal to point a -> b, matching
        // resolveSphereSphere's (b->position - a->position). Pointing it the
        // other way does not merely skip the bounce: applyContact's positional
        // correction then drives the two boxes further into each other, and
        // its "already moving apart" test reads an approaching pair as
        // separating, so a falling box sinks through a static one instead of
        // landing on it.
        Vector3D normal;
        double penetration;
        if (overlapX <= overlapY && overlapX <= overlapZ) {
            normal = Vector3D(a->position.x < b->position.x ? 1 : -1, 0, 0);
            penetration = overlapX;
        } else if (overlapY <= overlapZ) {
            normal = Vector3D(0, a->position.y < b->position.y ? 1 : -1, 0);
            penetration = overlapY;
        } else {
            normal = Vector3D(0, 0, a->position.z < b->position.z ? 1 : -1);
            penetration = overlapZ;
        }
        applyContact(a, b, normal, penetration);
    }
};

#endif // PHYSICSENGINE_H
