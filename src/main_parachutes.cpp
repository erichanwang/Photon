#include <iostream>
#include <vector>
#include <fstream>
#include "rendering/Scene.h"
#include "rendering/RayTracer.h"
#include "rendering/Camera.h"
#include "objects/Sphere.h"
#include "objects/Parachute.h"
#include "physics/PhysicsEngine.h"
// #include "rendering/UI.h"  // Commented out to avoid raylib dependency for testing

int main() {
    // Create scene
    Scene scene;

    // Add ground
    scene.addObject(new Sphere(Vector3D(0, -1001, 0), 1000, Material(Vector3D(0.5, 0.5, 0.5))));

    // Add parachutes
    for (int i = 0; i < 5; ++i) {
        Parachute* parachute = new Parachute(Vector3D(i*2 - 4, 5 + i, 0), 0.5, Material(Vector3D(0,1,0)));
        scene.addObject(parachute);
        // Add physics with high drag
        RigidBody* rb = new RigidBody(parachute->position, 1.0, 5.0);
        rb->radius = 0.5;
        rb->restitution = 0.1;     // canopies land, they do not bounce
        scene.physics.addBody(rb);
    }

    // The ground sphere's surface sits at y = -1; see main_blocks.cpp.
    scene.physics.groundY = -1.0;

    scene.addLight(Light(Vector3D(6, 12, 8), Vector3D(1.0, 0.95, 0.9), 1.4));
    scene.addLight(Light::sun(Vector3D(-0.3, -1.0, -0.4), 0.35, Vector3D(0.6, 0.7, 1.0)));

    // Camera: yaw=-pi/2 faces -Z (toward the parachutes at z=0), pitch tilts up to frame them.
    Camera camera(Vector3D(0, 2, 10), -M_PI / 2, 0.46f, 90, 16.0/9.0);

    // RayTracer
    RayTracer rayTracer(&scene, &camera);

    // Simulate physics and render
    int width = 800, height = 600;
    std::vector<Vector3D> image(width * height);

    for (int frame = 0; frame < 100; ++frame) {
        // Update physics
        scene.physics.update(0.1);

        // Update object positions
        for (size_t i = 0; i < scene.objects.size(); ++i) {
            if (i >= 1 && i <= 5) { // parachutes
                Parachute* parachute = dynamic_cast<Parachute*>(scene.objects[i]);
                if (parachute) {
                    parachute->position = scene.physics.bodies[i-1]->position;
                }
            }
        }

        // Objects moved this frame, so the BVH built for the previous one is
        // stale. Rebuilding is cheap next to the render it accelerates.
        scene.buildAcceleration();

        // Render
        rayTracer.render(scene, camera, width, height, image);

        // Save or display
        std::string filename = "output_parachutes_" + std::to_string(frame) + ".ppm";
        std::ofstream file(filename);
        file << "P3\n" << width << " " << height << "\n255\n";
        for (int j = height - 1; j >= 0; --j) {
            for (int i = 0; i < width; ++i) {
                Vector3D color = image[j * width + i];
                int r = std::min(255, (int)(color.x * 255));
                int g = std::min(255, (int)(color.y * 255));
                int b = std::min(255, (int)(color.z * 255));
                file << r << " " << g << " " << b << "\n";
            }
        }
        file.close();

        std::cout << "Frame " << frame << " rendered." << std::endl;
    }

    // Display last frame
    // DisplayImage("output_parachutes_99.ppm");  // Commented out to avoid raylib dependency for testing

    return 0;
}
