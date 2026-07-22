// Minimal assert-based self-check for the math/physics core.
// No framework: run the binary, non-zero exit / assertion abort means a regression.
#include <cassert>
#include <cmath>
#include <iostream>
#include "../src/math/Vector3D.h"
#include "../src/math/Ray.h"
#include "../src/objects/Sphere.h"
#include "../src/objects/Plane.h"
#include "../src/physics/RigidBody.h"
#include "../src/physics/PhysicsEngine.h"
#include "../src/rendering/Scene.h"
#include "../src/rendering/Camera.h"
#include "../src/rendering/RayTracer.h"
#include <vector>

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

// The BVH is only an optimization, so the one property that matters is that it
// changes nothing: for every ray, the accelerated result must equal the
// exhaustive scan's result exactly. A tree that is fast and subtly wrong is far
// worse than the linear scan it replaced.
static void testBVHMatchesLinearScan() {
    Scene scene;
    for (int i = 0; i < 60; i++) {
        double fx = ((i * 37) % 17) - 8.0;
        double fy = ((i * 53) % 13) - 6.0;
        double fz = -((i * 29) % 23) - 2.0;
        scene.addObject(new Sphere(Vector3D(fx, fy, fz), 0.4 + (i % 5) * 0.1,
                                   Material(Vector3D(1, 0, 0))));
    }
    // An unbounded plane too: it cannot live in the tree, so this also checks
    // that the separate list for infinite objects is consulted.
    scene.addObject(new Plane(Vector3D(0, -10, 0), Vector3D(0, 1, 0),
                              Material(Vector3D(0.5, 0.5, 0.5))));

    scene.buildAcceleration();
    assert(!scene.acceleration().empty());

    int compared = 0, hits = 0;
    for (int x = -12; x <= 12; x++) {
        for (int y = -8; y <= 8; y++) {
            Ray ray(Vector3D(x * 0.7, y * 0.7, 12), Vector3D(0, 0, -1));

            HitRecord fast, slow;
            bool hitFast = scene.intersect(ray, 0.001, 1e9, fast);
            bool hitSlow = scene.linearIntersect(ray, 0.001, 1e9, slow);

            assert(hitFast == hitSlow);
            if (hitSlow) {
                assert(approxEq(fast.t, slow.t, 1e-9));
                assert(approxEq(fast.point.x, slow.point.x, 1e-9));
                assert(approxEq(fast.point.y, slow.point.y, 1e-9));
                assert(approxEq(fast.point.z, slow.point.z, 1e-9));
                assert(approxEq(fast.normal.x, slow.normal.x, 1e-9));
                assert(approxEq(fast.normal.y, slow.normal.y, 1e-9));
                assert(approxEq(fast.normal.z, slow.normal.z, 1e-9));
                hits++;
            }
            compared++;
        }
    }
    // Guard against the test passing because every ray missed everything.
    assert(compared == 25 * 17);
    assert(hits > 100);

    for (Object* o : scene.objects) delete o;
    std::cout << "testBVHMatchesLinearScan passed (" << compared << " rays, "
              << hits << " hits)\n";
}

static void testGroundCollision() {
    PhysicsEngine engine;
    engine.groundY = 0.0;

    RigidBody* body = new RigidBody(Vector3D(0, 10, 0), 1.0);
    body->radius = 0.5;
    body->restitution = 0.5;
    engine.addBody(body);

    // Long enough to land and settle. Before collisions existed, this body
    // simply kept going: the failure the README used to document.
    for (int i = 0; i < 2000; ++i) engine.update(0.01);

    assert(body->position.y >= engine.groundY + body->radius - 1e-6);
    assert(approxEq(body->position.y, engine.groundY + body->radius, 1e-6));
    assert(approxEq(body->velocity.y, 0.0, 1e-6));

    delete body;
    std::cout << "testGroundCollision passed\n";
}

static void testSphereCollisionConservesMomentum() {
    PhysicsEngine engine;
    engine.hasGround = false;   // isolate the pair response from the floor

    // Head-on approach along x, gravity acts on y and so cannot affect the
    // x-momentum this checks.
    RigidBody* a = new RigidBody(Vector3D(-1.0, 0, 0), 2.0);
    RigidBody* b = new RigidBody(Vector3D(1.0, 0, 0), 3.0);
    a->radius = b->radius = 0.5;
    a->restitution = b->restitution = 1.0;   // elastic
    a->velocity = Vector3D(2.0, 0, 0);
    b->velocity = Vector3D(-1.0, 0, 0);
    engine.addBody(a);
    engine.addBody(b);

    double momentumBefore = a->mass * a->velocity.x + b->mass * b->velocity.x;

    for (int i = 0; i < 200; ++i) engine.update(0.01);

    double momentumAfter = a->mass * a->velocity.x + b->mass * b->velocity.x;
    assert(approxEq(momentumBefore, momentumAfter, 1e-9));

    // They must actually have collided and be separating, not merely drifted.
    assert(a->velocity.x < b->velocity.x);
    assert((b->position - a->position).length() >= a->radius + b->radius - 1e-6);

    delete a;
    delete b;
    std::cout << "testSphereCollisionConservesMomentum passed\n";
}

// Two renders of one scene must agree exactly whether threaded or not.
// Deterministic jitter is what makes this checkable; a data race in the
// row-sharing loop would show up here as a mismatched pixel.
static void testThreadedRenderMatchesSingleThreaded() {
    Scene scene;
    scene.addObject(new Sphere(Vector3D(0, 0.5, -4), 1.0, Material(Vector3D(0.8, 0.3, 0.3))));
    scene.addObject(new Sphere(Vector3D(2, 0.5, -6), 1.0, Material(Vector3D(0.3, 0.8, 0.3))));
    scene.addObject(new Plane(Vector3D(0, -1, 0), Vector3D(0, 1, 0), Material(Vector3D(0.6, 0.6, 0.6))));
    scene.addLight(Light(Vector3D(5, 8, 2), Vector3D(1, 1, 1), 1.2));
    scene.buildAcceleration();

    Camera camera(Vector3D(0, 1, 2), -M_PI / 2, -0.15f, 70, 16.0 / 9.0);

    RayTracer single(&scene, &camera);
    single.threadCount = 1;
    single.samplesPerPixel = 2;
    std::vector<Vector3D> imageSingle;
    single.render(scene, camera, 64, 48, imageSingle);

    RayTracer multi(&scene, &camera);
    multi.threadCount = 8;
    multi.samplesPerPixel = 2;
    std::vector<Vector3D> imageMulti;
    multi.render(scene, camera, 64, 48, imageMulti);

    assert(imageSingle.size() == imageMulti.size());
    for (size_t i = 0; i < imageSingle.size(); ++i) {
        assert(approxEq(imageSingle[i].x, imageMulti[i].x, 1e-12));
        assert(approxEq(imageSingle[i].y, imageMulti[i].y, 1e-12));
        assert(approxEq(imageSingle[i].z, imageMulti[i].z, 1e-12));
    }
    assert(single.rayCount.load() == multi.rayCount.load());
    assert(single.rayCount.load() > 0);

    for (Object* o : scene.objects) delete o;
    std::cout << "testThreadedRenderMatchesSingleThreaded passed ("
              << single.rayCount.load() << " rays each)\n";
}

// A point directly behind an occluder, relative to the light, must be shadowed.
static void testShadowRay() {
    Scene scene;
    Sphere* blocker = new Sphere(Vector3D(0, 2, 0), 1.0, Material(Vector3D(1, 1, 1)));
    scene.addObject(blocker);
    scene.buildAcceleration();

    // Light straight overhead: the origin sits in the sphere's shadow.
    Light overhead(Vector3D(0, 10, 0));
    double distance;
    Vector3D dir = overhead.directionFrom(Vector3D(0, 0, 0), distance);
    assert(scene.occluded(Vector3D(0, 0, 0), dir, distance));

    // Step aside and the light is visible again.
    Vector3D clearPoint(5, 0, 0);
    Vector3D clearDir = overhead.directionFrom(clearPoint, distance);
    assert(!scene.occluded(clearPoint, clearDir, distance));

    delete blocker;
    std::cout << "testShadowRay passed\n";
}

int main() {
    testVector3D();
    testRaySphereIntersection();
    testRigidBodyFreefall();
    testRigidBodyDrag();
    testBVHMatchesLinearScan();
    testGroundCollision();
    testSphereCollisionConservesMomentum();
    testThreadedRenderMatchesSingleThreaded();
    testShadowRay();
    std::cout << "All tests passed.\n";
    return 0;
}
