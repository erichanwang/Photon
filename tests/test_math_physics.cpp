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
#include "../src/physics/Player.h"
#include "../src/physics/InputDriver.h"
#include "../src/objects/Block.h"
#include "../src/rendering/Texture.h"
#include <vector>
#include <fstream>
#include <cstdio>

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

// Sphere UV against closed-form values at the poles and four equator points,
// where u = 0.5 + atan2(z,x)/(2pi), v = 0.5 + asin(y)/pi is exactly computable
// -- no need to eyeball a render to know the mapping is right.
static void testSphereUVMapping() {
    Sphere sphere(Vector3D(0, 0, 0), 2.0, Material(Vector3D(1, 1, 1)));
    HitRecord rec;

    auto uvAt = [&](const Vector3D& origin, const Vector3D& dir) {
        HitRecord r;
        bool hit = sphere.intersect(Ray(origin, dir), 0.001, 1e9, r);
        assert(hit);
        return r;
    };

    rec = uvAt(Vector3D(0, 10, 0), Vector3D(0, -1, 0));   // north pole
    assert(approxEq(rec.v, 1.0, 1e-9));
    assert(approxEq(rec.u, 0.5, 1e-9));

    rec = uvAt(Vector3D(0, -10, 0), Vector3D(0, 1, 0));   // south pole
    assert(approxEq(rec.v, 0.0, 1e-9));

    rec = uvAt(Vector3D(10, 0, 0), Vector3D(-1, 0, 0));   // equator, +x
    assert(approxEq(rec.u, 0.5, 1e-9));
    assert(approxEq(rec.v, 0.5, 1e-9));

    rec = uvAt(Vector3D(-10, 0, 0), Vector3D(1, 0, 0));   // equator, -x
    assert(approxEq(rec.u, 1.0, 1e-9));

    rec = uvAt(Vector3D(0, 0, 10), Vector3D(0, 0, -1));   // equator, +z
    assert(approxEq(rec.u, 0.75, 1e-9));

    rec = uvAt(Vector3D(0, 0, -10), Vector3D(0, 0, 1));   // equator, -z
    assert(approxEq(rec.u, 0.25, 1e-9));

    std::cout << "testSphereUVMapping passed\n";
}

// Plane UV feeds a CheckerTexture the same way the old isGrid/gridSize fields
// fed the ad-hoc floor(point/gridSize) formula. This checks the migration
// keeps the exact same checker phase for the axis-aligned ground plane, the
// only case the old code supported.
static void testPlaneCheckerMatchesOldGridFormula() {
    Plane plane(Vector3D(0, 0, 0), Vector3D(0, 1, 0), Material(Vector3D(0, 0, 0)));
    CheckerTexture checker(Vector3D(1, 1, 1), Vector3D(0, 0, 0), 1.0);

    double xs[] = {0.3, -0.3, 1.7, -1.7, 2.0, -2.0, 0.999, -0.001};
    double zs[] = {0.3, -0.3, 0.6, -0.6, 1.0, -1.0, 2.999, -2.001};
    for (double x : xs) {
        for (double z : zs) {
            HitRecord rec;
            bool hit = plane.intersect(Ray(Vector3D(x, 5, z), Vector3D(0, -1, 0)), 0.001, 1e9, rec);
            assert(hit);
            int ix = (int)std::floor(x / 1.0);
            int iz = (int)std::floor(z / 1.0);
            Vector3D expected = ((ix + iz) % 2 + 2) % 2 == 0 ? Vector3D(1, 1, 1) : Vector3D(0, 0, 0);
            Vector3D actual = checker.sample(rec.u, rec.v);
            assert(approxEq(actual.x, expected.x, 1e-9));
        }
    }

    std::cout << "testPlaneCheckerMatchesOldGridFormula passed\n";
}

// Hand-computed bilinear filtering over a 2x2 image:
//   (0,0)=red (1,1,0,0)  (1,0)=green (0,1,0)
//   (0,1)=blue (0,0,1)   (1,1)=yellow (1,1,0)
// (row-major, top row first, matching the PPM's storage order).
static void testBilinearImageTexture() {
    const char* path = "build/test_texture_fixture.ppm";
    {
        std::ofstream f(path);
        f << "P3\n2 2\n255\n";
        f << "255 0 0\n" << "0 255 0\n";
        f << "0 0 255\n" << "255 255 0\n";
    }

    ImageTexture tex(path);

    // Dead center of the image: equally weighted average of all four texels.
    Vector3D center = tex.sample(0.5, 0.5);
    assert(approxEq(center.x, 0.5, 1e-9));
    assert(approxEq(center.y, 0.5, 1e-9));
    assert(approxEq(center.z, 0.25, 1e-9));

    // u=0.5, v=0.75: exactly between the top-row texel centers (red, green),
    // no vertical blend (ty=0), so a pure 50/50 horizontal interpolation.
    Vector3D midTopRow = tex.sample(0.5, 0.75);
    assert(approxEq(midTopRow.x, 0.5, 1e-9));
    assert(approxEq(midTopRow.y, 0.5, 1e-9));
    assert(approxEq(midTopRow.z, 0.0, 1e-9));

    // Exact texel centers should reproduce that texel's color exactly.
    Vector3D topLeft = tex.sample(0.25, 0.75);
    assert(approxEq(topLeft.x, 1.0, 1e-9) && approxEq(topLeft.y, 0.0, 1e-9) && approxEq(topLeft.z, 0.0, 1e-9));

    std::remove(path);
    std::cout << "testBilinearImageTexture passed\n";
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
static void testBVHMatchesLinearScanFor(BVH::Heuristic heuristic, const char* label) {
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

    scene.buildAcceleration(heuristic);
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
    std::cout << "testBVHMatchesLinearScan[" << label << "] passed (" << compared
              << " rays, " << hits << " hits)\n";
}

static void testBVHMatchesLinearScan() {
    testBVHMatchesLinearScanFor(BVH::Heuristic::Median, "median");
    testBVHMatchesLinearScanFor(BVH::Heuristic::SAH, "sah");
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

// Refraction is checked against Snell's law directly rather than by eyeballing
// a render: a bent ray that merely looks plausible is exactly the kind of bug
// that survives to ship.
static void testRefraction() {
    const Vector3D n(0, 1, 0);

    // Air into glass at 45 degrees. Snell gives
    // sin(t) = sin(45)/1.5, so t = asin(0.7071/1.5) = 28.13 degrees.
    double inc = M_PI / 4.0;
    Vector3D d = Vector3D(std::sin(inc), -std::cos(inc), 0).normalize();
    Vector3D out;
    assert(RayTracer::refract(d, n, 1.0 / 1.5, out));

    double expected = std::asin(std::sin(inc) / 1.5);
    double actual = std::acos(std::min(1.0, -out.normalize().dot(n)));
    assert(std::fabs(actual - expected) < 1e-9);

    // Bending is toward the normal on the way in, so the outgoing angle is the
    // smaller one. This catches an inverted eta, which still produces a
    // unit-length direction and so passes any "is it NaN" check.
    assert(actual < inc);
    assert(std::fabs(out.length() - 1.0) < 1e-9);

    // Straight-on rays pass through undeviated at any index.
    Vector3D straight;
    assert(RayTracer::refract(Vector3D(0, -1, 0), n, 1.0 / 1.5, straight));
    assert(std::fabs(straight.x) < 1e-12 && std::fabs(straight.z) < 1e-12);

    // Glass into air past the critical angle, asin(1/1.5) = 41.8 degrees:
    // there is no transmitted ray, and refract() must say so rather than
    // return a NaN direction from the square root of a negative.
    double critical = std::asin(1.0 / 1.5);
    Vector3D steep = Vector3D(std::sin(critical + 0.05), -std::cos(critical + 0.05), 0).normalize();
    Vector3D none;
    assert(!RayTracer::refract(steep, n, 1.5 / 1.0, none));

    // Just inside the critical angle it must still transmit, so the boundary
    // is where physics puts it and not a few degrees off.
    Vector3D shallow = Vector3D(std::sin(critical - 0.05), -std::cos(critical - 0.05), 0).normalize();
    assert(RayTracer::refract(shallow, n, 1.5 / 1.0, none));

    // Fresnel: glass reflects little head-on and almost everything at a
    // grazing angle. Roughly 4% at normal incidence for an index of 1.5.
    assert(std::fabs(RayTracer::schlick(1.0, 1.0, 1.5) - 0.04) < 0.005);
    assert(RayTracer::schlick(0.0, 1.0, 1.5) > 0.99);
    assert(RayTracer::schlick(0.5, 1.0, 1.5) > RayTracer::schlick(1.0, 1.0, 1.5));

    std::cout << "testRefraction passed\n";
}

// A glass sphere must actually transmit what is behind it. Rendering the same
// scene with the sphere opaque and then transparent has to differ, or the
// refraction path is silently dead code.
static void testGlassTransmitsBackground() {
    Material opaque(Vector3D(0.9, 0.9, 0.9));
    Material glass = Material::dielectric(Vector3D(1, 1, 1), 1.5);

    auto renderCenterPixel = [](const Material& m) {
        Scene scene;
        Sphere* s = new Sphere(Vector3D(0, 0, -3), 1.0, m);
        // A red wall behind the sphere, visible only by transmission.
        Sphere* wall = new Sphere(Vector3D(0, 0, -12), 5.0, Material(Vector3D(1.0, 0.0, 0.0)));
        scene.addObject(s);
        scene.addObject(wall);
        scene.addLight(Light(Vector3D(0, 5, 2), Vector3D(1, 1, 1), 1.0));
        scene.buildAcceleration();

        Camera camera(Vector3D(0, 0, 0), -M_PI / 2, 0.0f, 60, 1.0);
        RayTracer tracer(&scene, &camera);
        tracer.maxDepth = 6;
        Vector3D c = tracer.trace(Ray(Vector3D(0, 0, 0), Vector3D(0, 0, -1)), 6);

        delete s;
        delete wall;
        return c;
    };

    Vector3D opaqueColor = renderCenterPixel(opaque);
    Vector3D glassColor = renderCenterPixel(glass);

    // The opaque sphere is lit by a white light, so it renders gray: no
    // channel can pick up the wall behind it.
    assert(std::fabs(opaqueColor.x - opaqueColor.y) < 1e-6);
    assert(std::fabs(opaqueColor.x - opaqueColor.z) < 1e-6);

    // The glass sphere is the same white material under the same white light,
    // so red can only lead here by way of the wall behind it. Comparing hue
    // rather than brightness matters: transmission tints the pixel, it does
    // not brighten it, and the glass pixel is in fact the darker of the two.
    assert(glassColor.x > glassColor.y + 0.1);
    assert(glassColor.x > glassColor.z + 0.1);
    assert(glassColor.x > opaqueColor.x - opaqueColor.y + 0.1);

    std::cout << "testGlassTransmitsBackground passed\n";
}

// Beer-Lambert: color(r) = wallColor * exp(-k * pathLength(r)) for a ray
// through the exact center of a dielectric sphere. Using ior = 1.0 makes
// Schlick's term exactly zero at every crossing (r0 = 0 and normal incidence
// gives cosi = 1 exactly), which removes reflection mixing from the picture
// entirely and leaves a clean closed form to check against, rather than one
// clouded by a Fresnel term this test would otherwise have to approximate.
static void testBeerLambertAbsorption() {
    Vector3D absorption(0.5, 0.0, 0.0);   // attenuate only the red channel

    auto renderThroughSphere = [&](double radius) {
        Scene scene;
        Material glass = Material::dielectric(Vector3D(1, 1, 1), 1.0, 1.0, absorption);
        Sphere* s = new Sphere(Vector3D(0, 0, -3), radius, glass);
        Sphere* wall = new Sphere(Vector3D(0, 0, -20), 5.0, Material(Vector3D(1, 1, 1)));
        scene.addObject(s);
        scene.addObject(wall);
        scene.addLight(Light(Vector3D(0, 5, 0), Vector3D(1, 1, 1), 1.0));
        scene.buildAcceleration();

        Camera camera(Vector3D(0, 0, 0), -M_PI / 2, 0.0f, 60, 1.0);
        RayTracer tracer(&scene, &camera);
        tracer.maxDepth = 6;
        Vector3D c = tracer.trace(Ray(Vector3D(0, 0, 0), Vector3D(0, 0, -1)), 6);

        delete s;
        delete wall;
        return c;
    };

    double r1 = 0.3, r2 = 1.2;
    Vector3D c1 = renderThroughSphere(r1);
    Vector3D c2 = renderThroughSphere(r2);

    // The wall-hit point, its shading, and the sky-reflection term are all
    // identical between the two radii (the ray is undeviated at ior = 1.0 and
    // always on-axis), so the epsilon surface offsets cancel in the ratio and
    // what remains is exactly exp(-k * (2*r2 - 2*r1)).
    double expectedRatio = std::exp(-absorption.x * (2.0 * r2 - 2.0 * r1));
    double actualRatio = c2.x / c1.x;
    assert(std::fabs(actualRatio - expectedRatio) < 1e-9);

    // Green and blue carry no absorption coefficient, so radius must not
    // affect them at all.
    assert(approxEq(c1.y, c2.y, 1e-9));
    assert(approxEq(c1.z, c2.z, 1e-9));
    assert(c1.y > 1e-6);   // sanity: the wall is actually visible through the glass

    std::cout << "testBeerLambertAbsorption passed (ratio " << actualRatio
              << " vs expected " << expectedRatio << ")\n";
}

// An area light must widen a shadow's edge into a penumbra: points near the
// occluder's shadow boundary should see a fraction of the emitter rather than
// the point light's hard 0-or-1 cutoff.
//
// The comparison has to be made against the SAME point with the occluder
// removed, not against some distant "obviously lit" point. Distance falloff
// and the Lambertian cosine both vary across the floor, so a far-off sample
// can easily be dimmer than a partially shadowed one near the light: at
// x=6 this scene shades to 0.46 while the penumbra at x=2 shades to 0.54.
// Comparing those two directly measures falloff, not shadowing.
static void testSoftShadowPenumbra() {
    Material floorMat(Vector3D(0.8, 0.8, 0.8));

    // Shade one floor point, optionally with the blocker present. Everything
    // else about the two scenes is identical, so any difference is shadowing.
    auto shadeAt = [&](double x, bool withBlocker) {
        Scene scene;
        Sphere* floor = new Sphere(Vector3D(0, -1000, 0), 1000.0, floorMat);
        scene.addObject(floor);
        if (withBlocker)
            scene.addObject(new Sphere(Vector3D(0, 3, 0), 1.0, Material(Vector3D(1, 1, 1))));
        scene.addLight(Light(Vector3D(0, 8, 0), Vector3D(1, 1, 1), 3.0, false, 2.0));
        scene.buildAcceleration();

        Camera camera(Vector3D(0, 5, 10), -M_PI / 2, -0.2f, 60, 1.0);
        RayTracer tracer(&scene, &camera);
        tracer.shadowSamples = 256;   // converge the average tightly

        HitRecord rec;
        rec.point = Vector3D(x, -0.0001, 0);
        rec.normal = Vector3D(0, 1, 0);
        rec.material = floorMat;
        Ray dummy(Vector3D(x, 10, 0), Vector3D(0, -1, 0));
        Vector3D c = tracer.shade(dummy, rec, floorMat.color, 0, 0, 0);

        for (Object* o : scene.objects) delete o;
        return c;
    };

    // Directly under the blocker: every point on the emitter is occluded, so
    // this is ambient-only.
    double umbra = shadeAt(0.0, true).x;
    // Partway out: part of the emitter is visible.
    double penumbra = shadeAt(2.0, true).x;
    // The identical point with nothing in the way.
    double unoccluded = shadeAt(2.0, false).x;

    assert(unoccluded > umbra + 0.1);
    // Strictly between the two extremes is the whole point: a hard point-light
    // shadow can only ever produce one or the other.
    assert(penumbra > umbra + 0.02);
    assert(penumbra < unoccluded - 0.02);

    // And the penumbra must actually be a gradient, not one intermediate step.
    double a = shadeAt(1.0, true).x, b = shadeAt(1.5, true).x;
    assert(a > umbra && b > a && penumbra > b);

    std::cout << "testSoftShadowPenumbra passed (umbra " << umbra << ", penumbra "
              << penumbra << ", unoccluded " << unoccluded << ")\n";
}

// Depth of field: at aperture > 0, only rays aimed at focusDistance should be
// unaffected by lens position; anything jittered off that exact ray direction
// must land somewhere else on the focal plane, which is precisely what
// produces blur for out-of-focus geometry. This checks the geometry directly
// rather than by eyeballing a render.
static void testDepthOfFieldFocusPlane() {
    Camera camera(Vector3D(0, 0, 0), -M_PI / 2, 0.0f, 60, 1.0);
    camera.aperture = 0.5;
    camera.focusDistance = 10.0;

    // The pinhole ray for this pixel, and the point it reaches on the focal
    // plane. Every lens sample for the same pixel must pass through exactly
    // that point - that is what "in focus at focusDistance" means.
    Camera pinholeRef(Vector3D(0, 0, 0), -M_PI / 2, 0.0f, 60, 1.0);
    Ray centre = pinholeRef.getRay(0.5, 0.5);
    Vector3D focusPoint = centre.origin + centre.direction * camera.focusDistance;

    // Two different lens samples for the same pixel (u,v).
    Ray a = camera.getRay(0.5, 0.5, 0.2, 0.7);
    Ray b = camera.getRay(0.5, 0.5, 0.8, 0.1);

    // They must leave from different points on the lens...
    assert((a.origin - b.origin).length() > 1e-6);

    // ...and both must point straight at the focus point. Checking
    // "origin + direction * focusDistance" instead would be wrong: an origin
    // offset sideways on the lens is further from the focus point than
    // focusDistance, so that lands short of the focal plane.
    assert(((focusPoint - a.origin).normalize() - a.direction).length() < 1e-9);
    assert(((focusPoint - b.origin).normalize() - b.direction).length() < 1e-9);

    // The lens offset must stay within the aperture.
    assert((a.origin - centre.origin).length() <= camera.aperture + 1e-9);
    assert((b.origin - centre.origin).length() <= camera.aperture + 1e-9);

    // Aperture <= 0 must reproduce the exact pinhole ray regardless of the
    // lens jitter passed in, so every old call site is untouched.
    Camera pinhole(Vector3D(1, 2, 3), 0.3f, -0.1f, 70, 1.5);
    Ray p1 = pinhole.getRay(0.4, 0.6);
    Ray p2 = pinhole.getRay(0.4, 0.6, 0.9, 0.9);
    assert(approxEq(p1.origin.x, p2.origin.x) && approxEq(p1.direction.x, p2.direction.x));
    assert(approxEq(p1.origin.y, p2.origin.y) && approxEq(p1.direction.y, p2.direction.y));
    assert(approxEq(p1.origin.z, p2.origin.z) && approxEq(p1.direction.z, p2.direction.z));

    std::cout << "testDepthOfFieldFocusPlane passed\n";
}

// Drives Player through a scripted input sequence exactly the way the
// headless control-loop demo does, proving move()/jump() are reachable
// through real per-frame input rather than only callable directly in a test.
static void testControlLoop() {
    Player player;
    const float yaw = -static_cast<float>(M_PI) / 2.0f;   // world-forward = -Z
    const float dt = 0.05f;

    // Player spawns above the ground (y=1.8, onGround=false), so the first
    // frames are plain free-fall before landing -- confirm that lands cleanly
    // and never sinks below the ground.
    double minY = player.position.y;
    int stepsToLand = 0;
    while (!player.onGround && stepsToLand < 200) {
        player.applyInput(InputState{}, dt, yaw);
        minY = std::min(minY, player.position.y);
        stepsToLand++;
    }
    assert(player.onGround);
    assert(approxEq(player.position.y, 0.0, 1e-9));
    assert(minY >= -1e-9);   // never fell through the ground

    // Movement input translates position: walk forward for a few frames.
    double zBefore = player.position.z;
    ScriptedInputDriver walkDriver(std::vector<InputState>(10, [] {
        InputState in; in.moveForward = 1.0; return in;
    }()));
    while (!walkDriver.exhausted()) {
        player.applyInput(walkDriver.poll(), dt, yaw);
        assert(player.position.y >= -1e-9);   // still resting on the ground
    }
    assert(player.position.z < zBefore - 1e-6);   // moved toward -Z (forward)
    assert(player.onGround);

    // Jump: leaves the ground, then gravity returns it exactly to y=0.
    InputState jumpInput;
    jumpInput.jump = true;
    player.applyInput(jumpInput, dt, yaw);
    assert(!player.onGround);
    assert(player.position.y > 0.0);
    assert(player.velocity.y > 0.0);

    int stepsToReland = 0;
    minY = player.position.y;
    while (!player.onGround && stepsToReland < 200) {
        player.applyInput(InputState{}, dt, yaw);
        minY = std::min(minY, player.position.y);
        stepsToReland++;
    }
    assert(player.onGround);
    assert(approxEq(player.position.y, 0.0, 1e-9));
    assert(approxEq(player.velocity.y, 0.0, 1e-9));
    assert(minY >= -1e-9);   // never sank through the ground on the way down

    std::cout << "testControlLoop passed (landed after " << stepsToLand
              << " steps, jump re-landed after " << stepsToReland << " steps)\n";
}

int main() {
    testVector3D();
    testRaySphereIntersection();
    testSphereUVMapping();
    testPlaneCheckerMatchesOldGridFormula();
    testBilinearImageTexture();
    testRigidBodyFreefall();
    testRigidBodyDrag();
    testBVHMatchesLinearScan();
    testGroundCollision();
    testSphereCollisionConservesMomentum();
    testThreadedRenderMatchesSingleThreaded();
    testShadowRay();
    testRefraction();
    testGlassTransmitsBackground();
    testBeerLambertAbsorption();
    testSoftShadowPenumbra();
    testDepthOfFieldFocusPlane();
    testControlLoop();
    std::cout << "All tests passed.\n";
    return 0;
}
