// Minimal assert-based self-check for the math/physics core.
// No framework: run the binary, non-zero exit / assertion abort means a regression.
#include <cassert>
#include <cmath>
#include <iostream>
#include "../src/math/Vector3D.h"
#include "../src/math/Ray.h"
#include "../src/objects/Sphere.h"
#include "../src/physics/RigidBody.h"
#include "../src/physics/PhysicsEngine.h"

static bool approxEq(double a, double b, double eps = 1e-6) {
    return std::fabs(a - b) < eps;
}

static void testVector3D() {
    Vector3D a(1, 2, 3), b(4, -5, 6);

    Vector3D sum = a + b;
    assert(approxEq(sum.x, 5) && approxEq(sum.y, -3) && approxEq(sum.z, 9));

    Vector3D diff = a - b;
    assert(approxEq(diff.x, -3) && approxEq(diff.y, 7) && approxEq(diff.z, -3));

    Vector3D scaled = a * 2.0;
    assert(approxEq(scaled.x, 2) && approxEq(scaled.y, 4) && approxEq(scaled.z, 6));

    assert(approxEq(a.dot(b), 1 * 4 + 2 * -5 + 3 * 6)); // = 4 - 10 + 18 = 12

    Vector3D cross = a.cross(b);
    // a x b = (2*6-3*-5, 3*4-1*6, 1*-5-2*4) = (27, 6, -13)
    assert(approxEq(cross.x, 27) && approxEq(cross.y, 6) && approxEq(cross.z, -13));

    Vector3D v(3, 4, 0);
    assert(approxEq(v.length(), 5.0));
    Vector3D n = v.normalize();
    assert(approxEq(n.length(), 1.0));

    // Zero-vector normalize must not divide by zero / produce NaN.
    Vector3D zeroNorm = Vector3D(0, 0, 0).normalize();
    assert(approxEq(zeroNorm.length(), 0.0));

    std::cout << "testVector3D passed\n";
}

static void testRaySphereIntersection() {
    Sphere sphere(Vector3D(0, 0, -5), 1.0, Material(Vector3D(1, 0, 0)));

    // Ray straight at the sphere center should hit at t = 4 (5 - radius).
    Ray hitRay(Vector3D(0, 0, 0), Vector3D(0, 0, -1));
    HitRecord rec;
    bool hit = sphere.intersect(hitRay, 0.001, 1e9, rec);
    assert(hit);
    assert(approxEq(rec.t, 4.0));
    assert(approxEq(rec.point.z, -4.0));
    // Normal at the near intersection point should point back toward the ray origin.
    assert(approxEq(rec.normal.z, 1.0));

    // Ray pointing away from the sphere should miss.
    Ray missRay(Vector3D(0, 0, 0), Vector3D(0, 0, 1));
    HitRecord rec2;
    assert(!sphere.intersect(missRay, 0.001, 1e9, rec2));

    // Ray that passes to the side should miss.
    Ray sideRay(Vector3D(5, 0, 0), Vector3D(0, 0, -1));
    HitRecord rec3;
    assert(!sphere.intersect(sideRay, 0.001, 1e9, rec3));

    std::cout << "testRaySphereIntersection passed\n";
}

static void testRigidBodyFreefall() {
    // No drag: semi-implicit Euler integration of free fall under gravity.
    // v_n = n * g * dt (down), y_n = y0 - g*dt^2 * n*(n+1)/2.
    PhysicsEngine engine;
    RigidBody* body = new RigidBody(Vector3D(0, 100, 0), 2.0 /* mass */, 0.0 /* no drag */);
    engine.addBody(body);

    double dt = 0.1;
    int steps = 10;
    for (int i = 0; i < steps; ++i) {
        engine.update(dt);
    }

    double expectedVy = -engine.gravity * dt * steps;
    assert(approxEq(body->velocity.y, expectedVy, 1e-9));

    double expectedY = 100.0;
    double v = 0.0;
    for (int i = 0; i < steps; ++i) {
        v += -engine.gravity * dt;   // semi-implicit: velocity updates first
        expectedY += v * dt;
    }
    assert(approxEq(body->position.y, expectedY, 1e-9));

    // Mass must not affect free-fall trajectory (Galileo).
    PhysicsEngine engine2;
    RigidBody* heavy = new RigidBody(Vector3D(0, 100, 0), 50.0, 0.0);
    engine2.addBody(heavy);
    for (int i = 0; i < steps; ++i) engine2.update(dt);
    assert(approxEq(heavy->velocity.y, body->velocity.y));
    assert(approxEq(heavy->position.y, body->position.y));

    delete body;
    delete heavy;
    std::cout << "testRigidBodyFreefall passed\n";
}

static void testRigidBodyDrag() {
    // With drag, downward acceleration shrinks as speed rises; the body should
    // never fall faster than pure free-fall over the same time (drag opposes velocity).
    PhysicsEngine engine;
    RigidBody dragBody(Vector3D(0, 1000, 0), 1.0, 5.0);
    RigidBody freeBody(Vector3D(0, 1000, 0), 1.0, 0.0);

    double dt = 0.05;
    for (int i = 0; i < 200; ++i) {
        dragBody.applyForce(Vector3D(0, -dragBody.mass * engine.gravity, 0));
        dragBody.update(dt);
        freeBody.applyForce(Vector3D(0, -freeBody.mass * engine.gravity, 0));
        freeBody.update(dt);
    }

    assert(std::fabs(dragBody.velocity.y) < std::fabs(freeBody.velocity.y));
    assert(dragBody.position.y > freeBody.position.y);

    // Terminal velocity check: at equilibrium, drag force == gravity force,
    // i.e. dragCoeff * v_terminal == mass * gravity.
    double vTerminal = dragBody.mass * engine.gravity / dragBody.dragCoeff;
    assert(std::fabs(dragBody.velocity.y) <= vTerminal + 1e-6);

    std::cout << "testRigidBodyDrag passed\n";
}

int main() {
    testVector3D();
    testRaySphereIntersection();
    testRigidBodyFreefall();
    testRigidBodyDrag();
    std::cout << "All tests passed.\n";
    return 0;
}
