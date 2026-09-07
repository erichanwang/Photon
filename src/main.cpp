#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "rendering/RayTracer.h"
#include "rendering/Texture.h"
#include "physics/Player.h"
#include "objects/Sphere.h"
#include "objects/Plane.h"

int main() {
    // Set up scene
    Scene scene;
    Material sphereMat(Vector3D(0.8, 0.3, 0.3));
    sphereMat.specular = 0.6;
    sphereMat.shininess = 64.0;
    sphereMat.reflectivity = 0.3;
    scene.addObject(new Sphere(Vector3D(0, 2, -5), 1.0, sphereMat));

    Material chromeMat(Vector3D(0.35, 0.4, 0.5));
    chromeMat.specular = 0.9;
    chromeMat.shininess = 128.0;
    chromeMat.reflectivity = 0.6;
    scene.addObject(new Sphere(Vector3D(2.2, 1.2, -6.5), 1.0, chromeMat));
    // Glass, in front of the checkered ground so the refraction is obvious:
    // the pattern behind it inverts through the sphere.
    scene.addObject(new Sphere(Vector3D(-2.0, 1.3, -4.0), 1.1,
                               Material::dielectric(Vector3D(1.0, 1.0, 1.0), 1.5)));

    Material groundMat;
    groundMat.texture = new CheckerTexture(Vector3D(0.8, 0.8, 0.8), Vector3D(0.2, 0.2, 0.2), 1.0);
    scene.addObject(new Plane(Vector3D(0, 0, 0), Vector3D(0, 1, 0), groundMat));

    scene.addLight(Light(Vector3D(4, 8, 1), Vector3D(1.0, 0.95, 0.9), 1.5));
    scene.addLight(Light::sun(Vector3D(-0.5, -1.0, -0.2), 0.4, Vector3D(0.6, 0.7, 1.0)));
    scene.buildAcceleration();

    // Set up player and camera
    Player player;
    // yaw=-pi/2 faces -Z (toward the sphere at z=-5); pitch tilts down slightly to frame it.
    Camera camera(player.position + Vector3D(0, 1.8, 0), -M_PI / 2, -0.3f, 90, 16.0 / 9.0);

    // Set up ray tracer
    RayTracer tracer(&scene, &camera);
    tracer.samplesPerPixel = 4;   // anti-aliasing
    tracer.maxDepth = 4;          // reflection bounces

    int width = 800;
    int height = 600;
    std::vector<Vector3D> image(width * height);

    // Render
    tracer.render(scene, camera, width, height, image);

    // Save to PPM
    std::ofstream file("output_interactive.ppm");
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

    std::cout << "Rendered output_interactive.ppm" << std::endl;

    return 0;
}
