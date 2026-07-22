#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>
#include "Scene.h"
#include "Camera.h"
#include "Light.h"
#include "../math/Vector3D.h"

class RayTracer {
public:
    const Scene* scene;
    const Camera* camera;

    // Rendering options.
    int maxDepth = 4;            // reflection bounces; 0 disables reflection
    int samplesPerPixel = 1;     // >1 enables jittered supersampling
    int threadCount = 0;         // 0 = hardware_concurrency, 1 = single-threaded
    Vector3D ambient = Vector3D(0.1, 0.1, 0.1);

    // Total primary + shadow + reflection rays cast by the last render().
    // Lets the benchmark report throughput rather than just wall time.
    //
    // Counting happens in a thread-local, folded into this atomic once per
    // worker at the end. Incrementing a shared atomic on every ray looks
    // harmless but puts one cache line under contention from every core at the
    // hottest point in the renderer. Removing it roughly doubled measured
    // 16-core scaling (about 3x before, 6-7x after).
    mutable std::atomic<long long> rayCount{0};
    inline static thread_local long long localRayCount = 0;

    RayTracer() : scene(nullptr), camera(nullptr) {}
    RayTracer(const Scene* s, const Camera* c) : scene(s), camera(c) {}

    // The sky, for rays that hit nothing.
    static Vector3D background(const Ray& ray) {
        Vector3D unitDir = ray.direction.normalize();
        double t = 0.5 * (unitDir.y + 1.0);
        return Vector3D(1.0, 1.0, 1.0) * (1.0 - t) + Vector3D(0.5, 0.7, 1.0) * t;
    }

    // Surface color before lighting: the checker pattern for grid materials,
    // the flat material color otherwise.
    static Vector3D albedoAt(const HitRecord& rec) {
        if (!rec.material.isGrid) return rec.material.color;
        int ix = (int)std::floor(rec.point.x / rec.material.gridSize);
        int iz = (int)std::floor(rec.point.z / rec.material.gridSize);
        // Floor-based indices go negative and C++ '%' keeps the sign, so an
        // unguarded (ix+iz)%2 flips the checker's phase across the origin.
        return ((ix + iz) % 2 + 2) % 2 == 0 ? rec.material.gridColor1 : rec.material.gridColor2;
    }

    Vector3D trace(const Ray& ray, int depth) const {
        localRayCount++;
        HitRecord rec;
        if (!scene->intersect(ray, 0.001, 1e9, rec)) return background(ray);

        Vector3D albedo = albedoAt(rec);

        // A scene with no lights renders flat, exactly as this engine did
        // before shading existed, so the older demo scenes keep looking like
        // themselves instead of going black.
        if (scene->lights.empty()) return albedo;

        Vector3D color = shade(ray, rec, albedo);

        double refl = rec.material.reflectivity;
        if (refl > 0.0 && depth > 0) {
            Vector3D n = rec.normal.normalize();
            Vector3D d = ray.direction.normalize();
            Vector3D reflectedDir = d - n * (2.0 * d.dot(n));
            // Offset along the normal, or the reflected ray immediately re-hits
            // the surface it just left and the image self-shadows.
            Ray reflected(rec.point + n * 1e-4, reflectedDir);
            Vector3D reflectedColor = trace(reflected, depth - 1);
            color = color * (1.0 - refl) + reflectedColor * refl;
        }
        return color;
    }

    // Blinn-Phong: ambient + Lambertian diffuse + specular highlight, with one
    // shadow ray per light.
    Vector3D shade(const Ray& ray, const HitRecord& rec, const Vector3D& albedo) const {
        Vector3D n = rec.normal.normalize();
        Vector3D viewDir = -ray.direction.normalize();
        // Two-sided shading: a normal pointing away from the viewer means we
        // hit a back face, and lighting it with the outward normal renders it
        // black no matter where the light is.
        if (n.dot(viewDir) < 0) n = -n;

        Vector3D result(albedo.x * ambient.x, albedo.y * ambient.y, albedo.z * ambient.z);

        for (const Light& light : scene->lights) {
            double distance;
            Vector3D lightDir = light.directionFrom(rec.point, distance);

            double nDotL = n.dot(lightDir);
            if (nDotL <= 0.0) continue;   // facing away from the light

            localRayCount++;
            if (scene->occluded(rec.point + n * 1e-4, lightDir, distance)) continue;

            double falloff = light.directional ? 1.0
                                               : 1.0 / std::max(1.0, distance * distance * 0.05);
            double energy = light.intensity * falloff;

            result += Vector3D(albedo.x * light.color.x * nDotL * energy,
                               albedo.y * light.color.y * nDotL * energy,
                               albedo.z * light.color.z * nDotL * energy);

            if (rec.material.specular > 0.0) {
                Vector3D halfway = (lightDir + viewDir).normalize();
                double spec = std::pow(std::max(0.0, n.dot(halfway)), rec.material.shininess);
                double s = spec * rec.material.specular * energy;
                result += Vector3D(light.color.x * s, light.color.y * s, light.color.z * s);
            }
        }

        return Vector3D(std::min(1.0, result.x), std::min(1.0, result.y), std::min(1.0, result.z));
    }

    Vector3D renderPixel(int x, int y, int width, int height) const {
        if (samplesPerPixel <= 1) {
            double u = double(x) / double(width);
            double v = double(y) / double(height);
            return trace(camera->getRay(u, v), maxDepth);
        }

        Vector3D sum(0, 0, 0);
        for (int s = 0; s < samplesPerPixel; s++) {
            // Deterministic jitter: a given pixel and sample index always get
            // the same offset, so two renders of one scene are bit-identical
            // and the benchmark measures the renderer rather than the noise.
            double jx = hashToUnit(x, y, s * 2 + 0);
            double jy = hashToUnit(x, y, s * 2 + 1);
            double u = (double(x) + jx) / double(width);
            double v = (double(y) + jy) / double(height);
            sum += trace(camera->getRay(u, v), maxDepth);
        }
        return sum / double(samplesPerPixel);
    }

    void render(const Scene& s, const Camera& c, int width, int height,
                std::vector<Vector3D>& image) const {
        RayTracer* self = const_cast<RayTracer*>(this);
        const Scene* savedScene = scene;
        const Camera* savedCamera = camera;
        self->scene = &s;
        self->camera = &c;

        image.assign((size_t)width * height, Vector3D());
        rayCount = 0;

        int threads = threadCount > 0 ? threadCount
                                      : (int)std::max(1u, std::thread::hardware_concurrency());
        if (threads == 1) {
            localRayCount = 0;
            renderRows(0, height, width, height, image);
            rayCount += localRayCount;
        } else {
            // Row-striped work sharing through an atomic cursor. Rows differ
            // wildly in cost -- one through the middle of the scene casts far
            // more shadow and reflection rays than one through empty sky -- so
            // a static split would leave most threads idle waiting for one.
            std::atomic<int> nextRow{0};
            std::vector<std::thread> pool;
            pool.reserve(threads);
            for (int t = 0; t < threads; t++) {
                pool.emplace_back([&]() {
                    localRayCount = 0;
                    for (;;) {
                        int row = nextRow.fetch_add(1);
                        if (row >= height) break;
                        renderRows(row, row + 1, width, height, image);
                    }
                    rayCount += localRayCount;
                });
            }
            for (auto& th : pool) th.join();
        }

        self->scene = savedScene;
        self->camera = savedCamera;
    }

private:
    void renderRows(int rowBegin, int rowEnd, int width, int height,
                    std::vector<Vector3D>& image) const {
        for (int j = rowBegin; j < rowEnd; j++)
            for (int i = 0; i < width; i++)
                image[(size_t)j * width + i] = renderPixel(i, j, width, height);
    }

    // Small integer hash (a Wang-style mix) mapped into [0,1). Cheap, and
    // reproducible across threads because it depends only on its inputs.
    static double hashToUnit(int x, int y, int s) {
        uint32_t h = (uint32_t)x * 73856093u ^ (uint32_t)y * 19349663u ^ (uint32_t)s * 83492791u;
        h ^= h >> 16; h *= 0x7feb352du;
        h ^= h >> 15; h *= 0x846ca68bu;
        h ^= h >> 16;
        return (h & 0xFFFFFF) / double(0x1000000);
    }
};

#endif // RAYTRACER_H
