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
    // Thin-lens depth of field. 0 (default) is a pinhole camera: every
    // getRay() call ignores lensU/lensV entirely and behaves exactly as
    // before. Above 0, rays are jittered across a disk of this radius and
    // re-aimed at focusDistance, so only that distance renders sharp.
    double aperture = 0.0;
    double focusDistance = 5.0;

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

    // lensU/lensV are a jitter pair in [0,1); ignored when aperture <= 0, so
    // every pre-existing two-argument call site still gets the exact old
    // pinhole ray.
    Ray getRay(double u, double v, double lensU = 0.0, double lensV = 0.0) const {
        Vector3D lower_left = position + direction * focal_length - right * half_width - up * half_height;
        Vector3D horizontal = right * 2 * half_width;
        Vector3D vertical = up * 2 * half_height;
        Vector3D dir = (lower_left + horizontal * u + vertical * v - position).normalize();
        if (aperture <= 0.0) return Ray(position, dir);

        // Area-preserving disk sample (sqrt of a uniform radius), re-aimed at
        // the point the pinhole ray would have hit on the focal plane.
        double r = std::sqrt(lensU) * aperture;
        double theta = 2.0 * M_PI * lensV;
        Vector3D lensOffset = right * (r * std::cos(theta)) + up * (r * std::sin(theta));
        Vector3D focusPoint = position + dir * focusDistance;
        Vector3D newOrigin = position + lensOffset;
        return Ray(newOrigin, (focusPoint - newOrigin).normalize());
    }
};

#endif // CAMERA_H
