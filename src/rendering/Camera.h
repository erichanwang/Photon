#ifndef CAMERA_H
#define CAMERA_H

#include "../math/Vector3D.h"
#include "../math/Ray.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

class Camera {
public:
    Vector3D position;
    Vector3D direction;
    Vector3D right;
    Vector3D up;
    double focal_length;
    double half_width;
    double half_height;
    float yaw;
    float pitch;

    Camera() : position(0,0,0), yaw(0), pitch(0) {
        updateDirection();
        right = direction.cross(Vector3D(0, 1, 0)).normalize();
        up = right.cross(direction).normalize();
        half_height = tan(90 * acos(-1.0) / 180.0 / 2.0);
        half_width = half_height * 1.0;
        focal_length = 1.0;
    }

    Camera(Vector3D pos, float y, float p, double fov, double aspect) : position(pos), yaw(y), pitch(p) {
        updateDirection();
        right = direction.cross(Vector3D(0, 1, 0)).normalize();
        up = right.cross(direction).normalize();
        half_height = tan(fov * acos(-1.0) / 180.0 / 2.0);
        half_width = half_height * aspect;
        focal_length = 1.0;
    }

    void updateDirection() {
        direction.x = cos(yaw) * cos(pitch);
        direction.y = sin(pitch);
        direction.z = sin(yaw) * cos(pitch);
        direction = direction.normalize();
    }

    Ray getRay(double u, double v) const {
        Vector3D lower_left = position + direction * focal_length - right * half_width - up * half_height;
        Vector3D horizontal = right * 2 * half_width;
        Vector3D vertical = up * 2 * half_height;
        return Ray(position, (lower_left + horizontal * u + vertical * v - position).normalize());
    }
};

#endif // CAMERA_H
