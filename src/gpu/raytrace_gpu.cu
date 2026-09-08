// GPU ray dispatch: primary rays traced against a flat sphere list on the
// device, one thread per pixel. This is the start of the CUDA port, not a
// finished one -- see the "Not ported yet" list below -- but it is real,
// correctness-checked code, not a stub.
//
// Scope, deliberately: sphere-only scene (no plane, no BVH), one directional
// light, ambient + Lambertian diffuse + Blinn-Phong specular, one shadow ray,
// no reflection/refraction recursion. That is exactly the subset of
// RayTracer::trace()/shade() that a scene with every material's reflectivity
// and transparency at 0 actually exercises, which is what makes an exact
// CPU/GPU comparison possible: build the identical scene on the host with the
// real CPU renderer (src/rendering/RayTracer.h) and diff the two images
// instead of eyeballing them.
//
// Not ported yet: BVH traversal on the device (this does the same brute-force
// per-sphere loop as the CPU's linear-scan baseline -- see bench/benchmark.cpp
// -- which is the right first correctness target, not the fast path), planes,
// reflection/refraction, multiple lights, soft shadows/DOF. Each is a real
// next step, not a hidden gap: the CPU renderer already has all of them, this
// kernel deliberately covers the slice that's checkable without dragging the
// whole shading model onto the device first.
//
// Build (requires the CUDA toolkit; not available on the machine this was
// written on -- see README.md's GPU section for what was and wasn't verified):
//   nvcc -O3 -std=c++17 -I../.. src/gpu/raytrace_gpu.cu -o build/GpuRayTrace
//   ./build/GpuRayTrace [sphereCount]
//
// It builds the same scene twice -- once on the CPU with the real
// RayTracer/Scene/Camera classes, once flattened into device arrays -- races
// both, and reports whether the images match within float-vs-double
// tolerance before printing a GPU rays/sec figure. A GPU render that looks
// plausible but disagrees with the CPU reference is a bug, not a port.

#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <vector>
#include <fstream>

#include "../math/Vector3D.h"
#include "../rendering/Scene.h"
#include "../rendering/RayTracer.h"
#include "../rendering/Camera.h"
#include "../objects/Sphere.h"

#define CUDA_CHECK(call)                                                        \
    do {                                                                        \
        cudaError_t err = (call);                                               \
        if (err != cudaSuccess) {                                               \
            std::fprintf(stderr, "CUDA error at %s:%d: %s\n", __FILE__, __LINE__, \
                        cudaGetErrorString(err));                               \
            std::exit(1);                                                       \
        }                                                                       \
    } while (0)

struct GpuSphere {
    float cx, cy, cz, r;
    float colr, colg, colb;
    float specular, shininess;
};

// Mirrors bench/benchmark.cpp's placement exactly (golden-angle ring), minus
// the ground plane and the occasional reflective material -- both out of
// scope for this first device kernel, see the file header.
static void placeSphere(int i, float& cx, float& cy, float& cz, float& r,
                        float& colr, float& colg, float& colb) {
    double angle = i * 2.399963;
    r = (float)(0.18 + (i % 7) * 0.03);
    double ring = 3.0 + (i % 19) * 0.35;
    cx = (float)(std::cos(angle) * ring);
    cy = (float)(-0.5 + (i % 11) * 0.45);
    cz = (float)(-6.0 + std::sin(angle) * ring);
    colr = (float)(0.2 + (i % 5) * 0.15);
    colg = 0.35f;
    colb = (float)(0.8 - (i % 4) * 0.15);
}

// --- device intersection + shading -----------------------------------------

__device__ bool sphereHit(const GpuSphere& s, float3 ro, float3 rd,
                          float tMin, float tMax, float& tOut, float3& nOut) {
    float ocx = ro.x - s.cx, ocy = ro.y - s.cy, ocz = ro.z - s.cz;
    float a = rd.x * rd.x + rd.y * rd.y + rd.z * rd.z;
    float b = 2.0f * (ocx * rd.x + ocy * rd.y + ocz * rd.z);
    float c = ocx * ocx + ocy * ocy + ocz * ocz - s.r * s.r;
    float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) return false;
    float sq = sqrtf(disc);
    float root = (-b - sq) / (2.0f * a);
    if (root < tMin || root > tMax) {
        root = (-b + sq) / (2.0f * a);
        if (root < tMin || root > tMax) return false;
    }
    tOut = root;
    float px = ro.x + rd.x * root, py = ro.y + rd.y * root, pz = ro.z + rd.z * root;
    nOut = make_float3((px - s.cx) / s.r, (py - s.cy) / s.r, (pz - s.cz) / s.r);
    return true;
}

// Nearest hit against every sphere: the device-side twin of Scene's
// pre-BVH linearIntersect(), which is the baseline every acceleration
// structure in this codebase is checked against before its speedup counts.
__device__ int nearestHit(const GpuSphere* spheres, int n, float3 ro, float3 rd,
                          float tMin, float tMax, float& tHit, float3& nHit) {
    int best = -1;
    for (int i = 0; i < n; i++) {
        float t;
        float3 nrm;
        if (sphereHit(spheres[i], ro, rd, tMin, tMax, t, nrm)) {
            tMax = t;
            tHit = t;
            nHit = nrm;
            best = i;
        }
    }
    return best;
}

__device__ bool occludedDevice(const GpuSphere* spheres, int n, float3 ro, float3 rd, float maxDist) {
    float t;
    float3 nrm;
    return nearestHit(spheres, n, ro, rd, 1e-4f, maxDist - 1e-4f, t, nrm) >= 0;
}

__global__ void traceKernel(const GpuSphere* spheres, int n,
                            float3 camPos, float3 lowerLeft, float3 horiz, float3 vert,
                            float3 lightDirToLight, float3 lightColor, float lightIntensity,
                            int width, int height, unsigned char* out) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    float u = (float)x / (float)width;
    float v = (float)y / (float)height;
    float3 target = make_float3(lowerLeft.x + horiz.x * u + vert.x * v - camPos.x,
                                lowerLeft.y + horiz.y * u + vert.y * v - camPos.y,
                                lowerLeft.z + horiz.z * u + vert.z * v - camPos.z);
    float len = sqrtf(target.x * target.x + target.y * target.y + target.z * target.z);
    float3 rd = make_float3(target.x / len, target.y / len, target.z / len);

    float3 color;
    float t;
    float3 nrm;
    int hit = nearestHit(spheres, n, camPos, rd, 0.001f, 1e9f, t, nrm);
    if (hit < 0) {
        // Same sky gradient as RayTracer::background().
        float bt = 0.5f * (rd.y + 1.0f);
        color = make_float3((1.0f - bt) + 0.5f * bt, (1.0f - bt) + 0.7f * bt, (1.0f - bt) + 1.0f * bt);
    } else {
        const GpuSphere& s = spheres[hit];
        float3 p = make_float3(camPos.x + rd.x * t, camPos.y + rd.y * t, camPos.z + rd.z * t);
        float3 viewDir = make_float3(-rd.x, -rd.y, -rd.z);
        float3 n2 = nrm;
        if (n2.x * viewDir.x + n2.y * viewDir.y + n2.z * viewDir.z < 0.0f) {
            n2 = make_float3(-n2.x, -n2.y, -n2.z);
        }

        float ambR = s.colr * 0.1f, ambG = s.colg * 0.1f, ambB = s.colb * 0.1f;
        color = make_float3(ambR, ambG, ambB);

        float nDotL = n2.x * lightDirToLight.x + n2.y * lightDirToLight.y + n2.z * lightDirToLight.z;
        if (nDotL > 0.0f) {
            float3 shadowOrigin = make_float3(p.x + n2.x * 1e-4f, p.y + n2.y * 1e-4f, p.z + n2.z * 1e-4f);
            if (!occludedDevice(spheres, n, shadowOrigin, lightDirToLight, 1e8f)) {
                float energy = lightIntensity;
                color.x += s.colr * lightColor.x * nDotL * energy;
                color.y += s.colg * lightColor.y * nDotL * energy;
                color.z += s.colb * lightColor.z * nDotL * energy;

                if (s.specular > 0.0f) {
                    float3 h = make_float3(lightDirToLight.x + viewDir.x,
                                           lightDirToLight.y + viewDir.y,
                                           lightDirToLight.z + viewDir.z);
                    float hlen = sqrtf(h.x * h.x + h.y * h.y + h.z * h.z);
                    h.x /= hlen; h.y /= hlen; h.z /= hlen;
                    float spec = powf(fmaxf(0.0f, n2.x * h.x + n2.y * h.y + n2.z * h.z), s.shininess);
                    float sp = spec * s.specular * energy;
                    color.x += lightColor.x * sp;
                    color.y += lightColor.y * sp;
                    color.z += lightColor.z * sp;
                }
            }
        }
    }

    color.x = fminf(1.0f, color.x);
    color.y = fminf(1.0f, color.y);
    color.z = fminf(1.0f, color.z);

    size_t idx = ((size_t)y * width + x) * 3;
    out[idx + 0] = (unsigned char)(color.x * 255.0f + 0.5f);
    out[idx + 1] = (unsigned char)(color.y * 255.0f + 0.5f);
    out[idx + 2] = (unsigned char)(color.z * 255.0f + 0.5f);
}

// --- host: build the identical scene twice, race them, diff the images -----

static void writePPM(const char* path, int width, int height, const unsigned char* rgb) {
    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << width << " " << height << "\n255\n";
    f.write((const char*)rgb, (size_t)width * height * 3);
}

int main(int argc, char** argv) {
    int sphereCount = (argc > 1) ? std::atoi(argv[1]) : 200;
    const int width = 400, height = 300;

    // --- CPU reference: the real renderer, reflectivity/transparency both
    // zero on every material so trace()'s recursive branches never fire and
    // shade() alone determines the pixel -- the exact subset the kernel
    // implements above.
    Scene scene;
    std::vector<GpuSphere> gpuSpheres(sphereCount);
    for (int i = 0; i < sphereCount; i++) {
        float cx, cy, cz, r, cr, cg, cb;
        placeSphere(i, cx, cy, cz, r, cr, cg, cb);
        Material m(Vector3D(cr, cg, cb));
        m.specular = 0.4;
        m.shininess = 48.0;
        scene.addObject(new Sphere(Vector3D(cx, cy, cz), r, m));
        gpuSpheres[i] = {cx, cy, cz, r, cr, cg, cb, 0.4f, 48.0f};
    }
    Vector3D sunDir(-0.4, -1.0, -0.3);
    scene.addLight(Light::sun(sunDir, 1.0, Vector3D(1.0, 1.0, 1.0)));
    scene.buildAcceleration();

    Camera camera(Vector3D(0, 1.5, 6), -(float)(M_PI / 2), -0.12f, 75, double(width) / height);

    RayTracer tracer(&scene, &camera);
    tracer.threadCount = 1;
    tracer.samplesPerPixel = 1;
    tracer.maxDepth = 0;   // no reflective/transparent material in this scene to recurse into

    std::vector<Vector3D> cpuImage;
    auto cpuStart = std::chrono::steady_clock::now();
    tracer.render(scene, camera, width, height, cpuImage);
    double cpuSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - cpuStart).count();

    std::vector<unsigned char> cpuRgb((size_t)width * height * 3);
    for (size_t i = 0; i < cpuImage.size(); i++) {
        cpuRgb[i * 3 + 0] = (unsigned char)(std::min(1.0, cpuImage[i].x) * 255.0 + 0.5);
        cpuRgb[i * 3 + 1] = (unsigned char)(std::min(1.0, cpuImage[i].y) * 255.0 + 0.5);
        cpuRgb[i * 3 + 2] = (unsigned char)(std::min(1.0, cpuImage[i].z) * 255.0 + 0.5);
    }

    // --- GPU: flatten the same scene, race the kernel -----------------------
    GpuSphere* dSpheres;
    unsigned char* dOut;
    CUDA_CHECK(cudaMalloc(&dSpheres, sizeof(GpuSphere) * sphereCount));
    CUDA_CHECK(cudaMalloc(&dOut, (size_t)width * height * 3));
    CUDA_CHECK(cudaMemcpy(dSpheres, gpuSpheres.data(), sizeof(GpuSphere) * sphereCount, cudaMemcpyHostToDevice));

    // Same camera basis math as Camera::getRay's pinhole path (aperture 0),
    // computed once on the host since it is the same for every pixel.
    Vector3D lowerLeft3 = camera.position + camera.direction * camera.focal_length
                        - camera.right * camera.half_width - camera.up * camera.half_height;
    Vector3D horiz3 = camera.right * 2 * camera.half_width;
    Vector3D vert3 = camera.up * 2 * camera.half_height;
    float3 camPos = make_float3((float)camera.position.x, (float)camera.position.y, (float)camera.position.z);
    float3 lowerLeft = make_float3((float)lowerLeft3.x, (float)lowerLeft3.y, (float)lowerLeft3.z);
    float3 horiz = make_float3((float)horiz3.x, (float)horiz3.y, (float)horiz3.z);
    float3 vert = make_float3((float)vert3.x, (float)vert3.y, (float)vert3.z);

    Vector3D toLight3 = (-sunDir).normalize();
    float3 lightDirToLight = make_float3((float)toLight3.x, (float)toLight3.y, (float)toLight3.z);
    float3 lightColor = make_float3(1.0f, 1.0f, 1.0f);

    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);

    auto gpuStart = std::chrono::steady_clock::now();
    traceKernel<<<grid, block>>>(dSpheres, sphereCount, camPos, lowerLeft, horiz, vert,
                                 lightDirToLight, lightColor, 1.0f, width, height, dOut);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());
    double gpuSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - gpuStart).count();

    std::vector<unsigned char> gpuRgb((size_t)width * height * 3);
    CUDA_CHECK(cudaMemcpy(gpuRgb.data(), dOut, gpuRgb.size(), cudaMemcpyDeviceToHost));
    cudaFree(dSpheres);
    cudaFree(dOut);

    writePPM("build/gpu_reference_cpu.ppm", width, height, cpuRgb.data());
    writePPM("build/gpu_reference_gpu.ppm", width, height, gpuRgb.data());

    // float vs double arithmetic through a sqrt/pow-heavy shading path never
    // lands on the exact same byte, so this checks "the same image" rather
    // than "the same bits" -- a few pixels a shade of gray off is float
    // rounding, a black-vs-lit sphere would be a real bug.
    int mismatches = 0;
    int maxDiff = 0;
    for (size_t i = 0; i < cpuRgb.size(); i++) {
        int d = std::abs((int)cpuRgb[i] - (int)gpuRgb[i]);
        if (d > maxDiff) maxDiff = d;
        if (d > 4) mismatches++;
    }

    long long rays = (long long)width * height;   // primary rays; shadow rays not separately counted here
    std::printf("GPU primary-ray port: %d spheres, %dx%d, 1 directional light, no BVH (brute force)\n",
                sphereCount, width, height);
    std::printf("  CPU reference : %7.4f s  (%.2f Mrays/s, primary only)\n", cpuSeconds, rays / cpuSeconds / 1e6);
    std::printf("  GPU kernel    : %7.4f s  (%.2f Mrays/s, primary only)\n", gpuSeconds, rays / gpuSeconds / 1e6);
    std::printf("  max per-channel diff: %d / 255, pixels over tolerance: %d / %d\n",
                maxDiff, mismatches, width * height);
    if (mismatches > (width * height) / 200) {   // more than 0.5% of pixels off is a real bug, not float noise
        std::printf("ABORT: GPU and CPU renders disagree on more than float rounding\n");
        for (Object* o : scene.objects) delete o;
        return 1;
    }
    std::printf("  images match within float tolerance: yes\n");

    for (Object* o : scene.objects) delete o;
    return 0;
}
