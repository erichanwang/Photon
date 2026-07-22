// Renderer benchmark: BVH vs linear scan, and thread scaling.
//
// Every figure quoted in README.md comes from running this. It renders the same
// scene each way and checks the images match before reporting any timing, so a
// "speedup" can never come from one configuration quietly doing less work.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>
#include <cmath>
#include "../src/rendering/Scene.h"
#include "../src/rendering/RayTracer.h"
#include "../src/rendering/Camera.h"
#include "../src/objects/Sphere.h"
#include "../src/objects/Plane.h"

static const int kWidth = 400;
static const int kHeight = 300;

// A scene dense enough for the acceleration structure to matter. A handful of
// spheres would make any BVH look pointless.
static void buildScene(Scene& scene, int sphereCount) {
    for (int i = 0; i < sphereCount; i++) {
        double angle = i * 2.399963;                 // golden angle, avoids rows
        double radius = 0.18 + (i % 7) * 0.03;
        double r = 3.0 + (i % 19) * 0.35;
        Vector3D center(std::cos(angle) * r,
                        -0.5 + (i % 11) * 0.45,
                        -6.0 + std::sin(angle) * r);
        Material m(Vector3D(0.2 + (i % 5) * 0.15, 0.35, 0.8 - (i % 4) * 0.15));
        m.specular = 0.4;
        m.shininess = 48.0;
        if (i % 9 == 0) m.reflectivity = 0.35;
        scene.addObject(new Sphere(center, radius, m));
    }
    Material ground(Vector3D(0.55, 0.55, 0.6));
    ground.isGrid = true;
    ground.gridColor1 = Vector3D(0.75, 0.75, 0.75);
    ground.gridColor2 = Vector3D(0.25, 0.25, 0.25);
    ground.gridSize = 1.0;
    scene.addObject(new Plane(Vector3D(0, -3, 0), Vector3D(0, 1, 0), ground));

    scene.addLight(Light(Vector3D(6, 10, 4), Vector3D(1.0, 0.95, 0.85), 1.3));
    scene.addLight(Light::sun(Vector3D(-0.4, -1.0, -0.3), 0.5, Vector3D(0.6, 0.7, 1.0)));
}

static int gRepeats = 3;

// Best of N. The minimum is the sample least contaminated by whatever else the
// machine was doing; averaging would let one scheduling hiccup swamp the result.
static double renderSeconds(const Scene& scene, const Camera& camera,
                            int threads, std::vector<Vector3D>& out,
                            long long& rays) {
    RayTracer tracer(&scene, &camera);
    tracer.threadCount = threads;
    tracer.samplesPerPixel = 2;
    tracer.maxDepth = 3;

    double best = 1e30;
    for (int i = 0; i < gRepeats; i++) {
        auto start = std::chrono::steady_clock::now();
        tracer.render(scene, camera, kWidth, kHeight, out);
        auto end = std::chrono::steady_clock::now();
        double seconds = std::chrono::duration<double>(end - start).count();
        if (seconds < best) best = seconds;
    }
    rays = tracer.rayCount.load();
    return best;
}

static bool imagesMatch(const std::vector<Vector3D>& a, const std::vector<Vector3D>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++) {
        if (std::fabs(a[i].x - b[i].x) > 1e-9) return false;
        if (std::fabs(a[i].y - b[i].y) > 1e-9) return false;
        if (std::fabs(a[i].z - b[i].z) > 1e-9) return false;
    }
    return true;
}

int main(int argc, char** argv) {
    int sphereCount = (argc > 1) ? std::atoi(argv[1]) : 500;
    if (argc > 2) gRepeats = std::atoi(argv[2]);

    Scene scene;
    buildScene(scene, sphereCount);
    Camera camera(Vector3D(0, 1.5, 6), -M_PI / 2, -0.12f, 75, double(kWidth) / kHeight);

    std::printf("Scene: %d spheres + 1 unbounded plane, %dx%d, 2 samples/pixel, depth 3\n",
                sphereCount, kWidth, kHeight);
    std::printf("Timings are the best of %d runs.\n", gRepeats);

    // --- BVH vs linear scan, both single-threaded to isolate the structure ---
    std::vector<Vector3D> linearImage, bvhImage;
    long long linearRays = 0, bvhRays = 0;

    scene.invalidateAcceleration();     // falls back to the exhaustive scan
    double linearTime = renderSeconds(scene, camera, 1, linearImage, linearRays);

    scene.buildAcceleration();
    double bvhTime = renderSeconds(scene, camera, 1, bvhImage, bvhRays);

    if (!imagesMatch(linearImage, bvhImage)) {
        std::printf("ABORT: BVH and linear scan produced different images\n");
        return 1;
    }

    std::printf("\n=== Intersection: BVH vs linear scan (1 thread) ===\n");
    std::printf("  linear scan : %7.3f s   (%.2f Mrays/s)\n",
                linearTime, linearRays / linearTime / 1e6);
    std::printf("  BVH         : %7.3f s   (%.2f Mrays/s)\n",
                bvhTime, bvhRays / bvhTime / 1e6);
    std::printf("  speedup     : %.1fx   (tree depth %d over %zu nodes)\n",
                linearTime / bvhTime, scene.acceleration().depth(),
                scene.acceleration().nodes.size());
    std::printf("  images identical: yes (%lld rays each)\n", bvhRays);

    // --- thread scaling, with the BVH on ---
    std::printf("\n=== Thread scaling (BVH enabled) ===\n");
    int hw = (int)std::max(1u, std::thread::hardware_concurrency());
    std::printf("  hardware_concurrency reports %d\n", hw);

    double baseline = 0.0;
    for (int threads = 1; threads <= hw; threads *= 2) {
        std::vector<Vector3D> image;
        long long rays = 0;
        double seconds = renderSeconds(scene, camera, threads, image, rays);
        if (threads == 1) baseline = seconds;
        if (!imagesMatch(bvhImage, image)) {
            std::printf("ABORT: %d threads produced a different image\n", threads);
            return 1;
        }
        std::printf("  %3d thread%s : %7.3f s   (%.2f Mrays/s)   %.1fx\n",
                    threads, threads == 1 ? " " : "s", seconds,
                    rays / seconds / 1e6, baseline / seconds);
    }

    for (Object* o : scene.objects) delete o;
    return 0;
}
