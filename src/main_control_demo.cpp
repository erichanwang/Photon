// Proves Player::move()/jump() are reachable through a real control loop,
// not just unit-tested in isolation: a scripted InputState sequence drives
// the player through the physics update each frame, and each frame is
// actually ray-traced so the player's motion shows up in the render, not
// just in a printed trajectory.
//
// This is the headless half of "no interactive control loop" (see README).
// Swapping ScriptedInputDriver for RaylibInputDriver (guarded behind
// PHOTON_USE_RAYLIB in InputDriver.h) is the only change a real windowed
// build would need; the control loop itself does not know or care which
// driver fed it.
#include <iostream>
#include <fstream>
#include <string>
#include "rendering/Scene.h"
#include "rendering/RayTracer.h"
#include "rendering/Camera.h"
#include "objects/Sphere.h"
#include "objects/Plane.h"
#include "physics/Player.h"
#include "physics/InputDriver.h"

int main() {
    Scene scene;
    Material groundMat;
    groundMat.isGrid = true;
    groundMat.gridColor1 = Vector3D(0.8, 0.8, 0.8);
    groundMat.gridColor2 = Vector3D(0.2, 0.2, 0.2);
    scene.addObject(new Plane(Vector3D(0, 0, 0), Vector3D(0, 1, 0), groundMat));
    scene.addObject(new Sphere(Vector3D(0, 1, -8), 1.0, Material(Vector3D(0.8, 0.3, 0.3))));
    scene.addLight(Light(Vector3D(4, 8, 1), Vector3D(1.0, 0.95, 0.9), 1.5));
    scene.addLight(Light::sun(Vector3D(-0.5, -1.0, -0.2), 0.4, Vector3D(0.6, 0.7, 1.0)));

    Player player;
    const float yaw = -static_cast<float>(M_PI) / 2.0f; // faces -Z, toward the sphere

    // Scripted script: walk forward, jump mid-stride, keep walking while
    // airborne and after landing. Long enough to show takeoff, apex, and
    // landing without needing an actual window.
    std::vector<InputState> script;
    for (int i = 0; i < 30; i++) {
        InputState in;
        in.moveForward = 1.0;
        in.jump = (i == 10);   // player lands around frame 5; jump once grounded
        script.push_back(in);
    }
    ScriptedInputDriver driver(std::move(script));

    RayTracer tracer(&scene, nullptr);
    tracer.samplesPerPixel = 1;
    tracer.maxDepth = 2;
    int width = 320, height = 180;
    std::vector<Vector3D> image(width * height);

    const float dt = 0.1f;
    int frame = 0;
    while (!driver.exhausted()) {
        InputState in = driver.poll();
        player.applyInput(in, dt, yaw);

        Camera camera(player.position + Vector3D(0, 1.8, 0), yaw, -0.1f, 90, 16.0 / 9.0);
        scene.buildAcceleration();
        tracer.render(scene, camera, width, height, image);

        std::string filename = "output_control_" + std::to_string(frame) + ".ppm";
        std::ofstream file(filename);
        file << "P3\n" << width << " " << height << "\n255\n";
        for (int j = height - 1; j >= 0; --j) {
            for (int i = 0; i < width; ++i) {
                Vector3D color = image[(size_t)j * width + i];
                int r = std::min(255, (int)(color.x * 255));
                int g = std::min(255, (int)(color.y * 255));
                int b = std::min(255, (int)(color.z * 255));
                file << r << " " << g << " " << b << "\n";
            }
        }

        std::cout << "Frame " << frame << ": pos=(" << player.position.x << ", "
                   << player.position.y << ", " << player.position.z << ") "
                   << "onGround=" << player.onGround << "\n";
        frame++;
    }

    return 0;
}
