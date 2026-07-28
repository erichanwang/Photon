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
    // Shadow rays per light per shading sample. Only matters for lights with
    // radius > 0 (area lights); point/directional lights always use a single
    // ray, so this is a no-op cost-wise until an area light is in the scene.
    int shadowSamples = 1;
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
        // ray.direction is already unit -- Ray's constructor normalizes it --
        // so re-normalizing here was pure wasted sqrt work on the hottest path
        // in the renderer (every miss goes through this).
        const Vector3D& unitDir = ray.direction;
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

    // Snell's law. 'd' and 'n' must be unit, 'n' facing against 'd', and eta is
    // the ratio of the incoming medium's index to the outgoing one. Returns
    // false on total internal reflection, where no transmitted ray exists:
    // past the critical angle the geometry has no solution, and squaring a
    // negative here is what produces NaN directions in a naive implementation.
    static bool refract(const Vector3D& d, const Vector3D& n, double eta, Vector3D& out) {
        double cosi = -d.dot(n);
        double k = 1.0 - eta * eta * (1.0 - cosi * cosi);
        if (k < 0.0) return false;
        out = d * eta + n * (eta * cosi - std::sqrt(k));
        return true;
    }

    // Schlick's approximation of the Fresnel term: how much light reflects off
    // a dielectric rather than passing through, as a function of view angle.
    // This is why glass is a window head-on and a mirror at a glancing angle.
    static double schlick(double cosi, double etaI, double etaT) {
        double r0 = (etaI - etaT) / (etaI + etaT);
        r0 *= r0;
        return r0 + (1.0 - r0) * std::pow(1.0 - cosi, 5.0);
    }

    // px, py, s identify the pixel and antialiasing sample this ray belongs
    // to. They default to 0 so every pre-existing call site (tests calling
    // trace() directly) is unaffected; renderPixel() below passes the real
    // coordinates so area-light sampling stays deterministic per pixel.
    Vector3D trace(const Ray& ray, int depth, int px = 0, int py = 0, int s = 0) const {
        localRayCount++;
        HitRecord rec;
        bool hit = scene->intersect(ray, 0.001, 1e9, rec);
        return traceWithHit(ray, depth, hit, rec, px, py, s);
    }

    // Continues trace() from an already-computed primary intersection.
    // Split out so the batched 4-ray packet path (renderPixelPacket4) can
    // reuse all of the shading/reflection/refraction logic below without
    // re-testing the ray against the scene -- the packet's BVH traversal
    // already answered that. trace() above is the exact original body,
    // just handing off here after doing its own scene->intersect().
    Vector3D traceWithHit(const Ray& ray, int depth, bool hit, const HitRecord& rec,
                          int px = 0, int py = 0, int s = 0) const {
        if (!hit) return background(ray);

        Vector3D albedo = albedoAt(rec);

        // A scene with no lights renders flat, exactly as this engine did
        // before shading existed, so the older demo scenes keep looking like
        // themselves instead of going black.
        if (scene->lights.empty()) return albedo;

        Vector3D color = shade(ray, rec, albedo, px, py, s);

        double trans = rec.material.transparency;
        double refl = rec.material.reflectivity;

        if (trans > 0.0 && depth > 0) {
            Vector3D n = rec.normal.normalize();
            const Vector3D& d = ray.direction;   // already unit, see background()

            // A ray leaving the glass hits the same surface from inside, where
            // the stored normal points the wrong way and the two media are
            // swapped. Getting this backwards is what makes a sphere render as
            // a solid blob instead of something you can see through.
            bool exiting = d.dot(n) > 0.0;
            double etaI = 1.0, etaT = rec.material.refractiveIndex;
            if (exiting) { std::swap(etaI, etaT); n = -n; }
            double cosi = std::min(1.0, -d.dot(n));

            Vector3D reflectedDir = d - n * (2.0 * d.dot(n));
            Vector3D reflectedColor = trace(Ray(rec.point + n * 1e-4, reflectedDir), depth - 1, px, py, s);

            Vector3D refractedDir, through;
            if (refract(d, n, etaI / etaT, refractedDir)) {
                // Offset below the surface: the transmitted ray continues into
                // the object it just entered.
                Vector3D refractedColor = trace(Ray(rec.point - n * 1e-4, refractedDir), depth - 1, px, py, s);
                double f = schlick(cosi, etaI, etaT);
                through = refractedColor * (1.0 - f) + reflectedColor * f;
            } else {
                through = reflectedColor;   // total internal reflection
            }

            // Beer-Lambert: this hit is where the ray leaves the medium it was
            // traveling through, and rec.t is exactly the distance it covered
            // inside (the ray was spawned at the entry point). Thicker glass
            // along the path attenuates more; absorption of 0 leaves `through`
            // unchanged, matching every material that predates this field.
            if (exiting) {
                const Vector3D& a = rec.material.absorption;
                through = Vector3D(through.x * std::exp(-a.x * rec.t),
                                    through.y * std::exp(-a.y * rec.t),
                                    through.z * std::exp(-a.z * rec.t));
            }

            color = color * (1.0 - trans) + through * trans;
        } else if (refl > 0.0 && depth > 0) {
            Vector3D n = rec.normal.normalize();
            const Vector3D& d = ray.direction;   // already unit, see background()
            Vector3D reflectedDir = d - n * (2.0 * d.dot(n));
            // Offset along the normal, or the reflected ray immediately re-hits
            // the surface it just left and the image self-shadows.
            Ray reflected(rec.point + n * 1e-4, reflectedDir);
            Vector3D reflectedColor = trace(reflected, depth - 1, px, py, s);
            color = color * (1.0 - refl) + reflectedColor * refl;
        }
        return color;
    }

    // Blinn-Phong: ambient + Lambertian diffuse + specular highlight. Point
    // and directional lights cast one shadow ray, exactly as before; area
    // lights (radius > 0) average `shadowSamples` stratified rays across the
    // emitter's sphere, which is what turns their shadow edges into a
    // penumbra instead of a hard line.
    Vector3D shade(const Ray& ray, const HitRecord& rec, const Vector3D& albedo,
                   int px = 0, int py = 0, int s = 0) const {
        Vector3D n = rec.normal.normalize();
        Vector3D viewDir = -ray.direction;   // already unit, see background()
        // Two-sided shading: a normal pointing away from the viewer means we
        // hit a back face, and lighting it with the outward normal renders it
        // black no matter where the light is.
        if (n.dot(viewDir) < 0) n = -n;

        Vector3D result(albedo.x * ambient.x, albedo.y * ambient.y, albedo.z * ambient.z);

        for (size_t li = 0; li < scene->lights.size(); li++) {
            const Light& light = scene->lights[li];

            // Point/directional lights (radius == 0) always take 1 sample, so
            // this reduces to the exact old single-shadow-ray computation --
            // same distance, same direction, same energy -- when no area
            // light is present.
            int samples = (light.radius > 0.0 && !light.directional && shadowSamples > 1)
                              ? shadowSamples : 1;

            for (int k = 0; k < samples; k++) {
                Vector3D samplePos = light.position;
                if (samples > 1) {
                    // Stratified point on the emitter's sphere, uniform over
                    // its surface. Seeded from pixel + AA-sample + light +
                    // stratum only, never a thread id or global counter, so
                    // threaded and single-threaded renders stay bit-identical.
                    double u1 = hashToUnit(px * 92821 + py, (int)li * 131 + k, s * 197 + 11);
                    double u2 = hashToUnit(px, py * 92821 + (int)li * 131 + k, s * 197 + 37);
                    double z = 1.0 - 2.0 * u1;
                    double r = std::sqrt(std::max(0.0, 1.0 - z * z));
                    double phi = 2.0 * M_PI * u2;
                    Vector3D onSphere(r * std::cos(phi), r * std::sin(phi), z);
                    samplePos = light.position + onSphere * light.radius;
                }

                double distance;
                Vector3D lightDir;
                if (light.directional) {
                    lightDir = light.directionFrom(rec.point, distance);
                } else {
                    Vector3D toLight = samplePos - rec.point;
                    distance = toLight.length();
                    lightDir = toLight.normalize();
                }

                double nDotL = n.dot(lightDir);
                if (nDotL <= 0.0) continue;   // facing away from the light

                localRayCount++;
                if (scene->occluded(rec.point + n * 1e-4, lightDir, distance)) continue;

                double falloff = light.directional ? 1.0
                                                   : 1.0 / std::max(1.0, distance * distance * 0.05);
                double energy = light.intensity * falloff / samples;

                result += Vector3D(albedo.x * light.color.x * nDotL * energy,
                                   albedo.y * light.color.y * nDotL * energy,
                                   albedo.z * light.color.z * nDotL * energy);

                if (rec.material.specular > 0.0) {
                    Vector3D halfway = (lightDir + viewDir).normalize();
                    double spec = std::pow(std::max(0.0, n.dot(halfway)), rec.material.shininess);
                    double sp = spec * rec.material.specular * energy;
                    result += Vector3D(light.color.x * sp, light.color.y * sp, light.color.z * sp);
                }
            }
        }

        return Vector3D(std::min(1.0, result.x), std::min(1.0, result.y), std::min(1.0, result.z));
    }

    Vector3D renderPixel(int x, int y, int width, int height) const {
        if (samplesPerPixel <= 1) {
            double u = double(x) / double(width);
            double v = double(y) / double(height);
            double lensU = hashToUnit(x, y, 1000);
            double lensV = hashToUnit(x, y, 1001);
            return trace(camera->getRay(u, v, lensU, lensV), maxDepth, x, y, 0);
        }

        Vector3D sum(0, 0, 0);
        for (int s = 0; s < samplesPerPixel; s++) {
            // Deterministic jitter: a given pixel and sample index always get
            // the same offset, so two renders of one scene are bit-identical
            // and the benchmark measures the renderer rather than the noise.
            // Same reasoning covers the lens jitter used for depth of field.
            double jx = hashToUnit(x, y, s * 4 + 0);
            double jy = hashToUnit(x, y, s * 4 + 1);
            double lensU = hashToUnit(x, y, s * 4 + 2);
            double lensV = hashToUnit(x, y, s * 4 + 3);
            double u = (double(x) + jx) / double(width);
            double v = (double(y) + jy) / double(height);
            sum += trace(camera->getRay(u, v, lensU, lensV), maxDepth, x, y, s);
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
    // Renders 4 adjacent pixels' primary rays as one SIMD packet (see
    // BVH::intersect4): their camera rays are tested against the tree in a
    // single batched traversal instead of four separate ones. Everything
    // after that first hit -- shading, shadows, reflection/refraction rays
    // -- is unchanged scalar traceWithHit(), called once per lane with
    // exactly the px/py/s the equivalent renderPixel() call would have used,
    // so this produces bit-identical pixels to the non-packet path.
    void renderPixelPacket4(int x0, int y, int width, int height,
                            std::vector<Vector3D>& image) const {
        int samples = samplesPerPixel <= 1 ? 1 : samplesPerPixel;
        Vector3D sums[4];

        for (int s = 0; s < samples; s++) {
            Ray rays[4];
            for (int k = 0; k < 4; k++) {
                int x = x0 + k;
                double jx = 0.0, jy = 0.0, lensU, lensV;
                if (samplesPerPixel <= 1) {
                    lensU = hashToUnit(x, y, 1000);
                    lensV = hashToUnit(x, y, 1001);
                } else {
                    jx = hashToUnit(x, y, s * 4 + 0);
                    jy = hashToUnit(x, y, s * 4 + 1);
                    lensU = hashToUnit(x, y, s * 4 + 2);
                    lensV = hashToUnit(x, y, s * 4 + 3);
                }
                double u = (double(x) + jx) / double(width);
                double v = (double(y) + jy) / double(height);
                rays[k] = camera->getRay(u, v, lensU, lensV);
            }

            double tMax[4] = { 1e9, 1e9, 1e9, 1e9 };
            HitRecord rec[4];
            bool hit[4];
            scene->intersect4(rays, 0.001, tMax, rec, hit, 0xF);

            for (int k = 0; k < 4; k++) {
                localRayCount++;
                Vector3D c = traceWithHit(rays[k], maxDepth, hit[k], rec[k], x0 + k, y, s);
                if (s == 0) sums[k] = c; else sums[k] += c;
            }
        }

        for (int k = 0; k < 4; k++)
            image[(size_t)y * width + x0 + k] = samples > 1 ? sums[k] / double(samples) : sums[k];
    }

    void renderRows(int rowBegin, int rowEnd, int width, int height,
                    std::vector<Vector3D>& image) const {
        for (int j = rowBegin; j < rowEnd; j++) {
            int i = 0;
            for (; i + 4 <= width; i += 4)
                renderPixelPacket4(i, j, width, height, image);
            for (; i < width; i++)
                image[(size_t)j * width + i] = renderPixel(i, j, width, height);
        }
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
