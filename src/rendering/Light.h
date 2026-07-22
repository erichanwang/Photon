#ifndef LIGHT_H
#define LIGHT_H

#include "../math/Vector3D.h"

// A point light, or a directional one when `directional` is set (in which case
// `position` is read as the direction the light travels *from*, and distance
// falloff does not apply -- the sun is not meaningfully closer to one object).
class Light {
public:
    Vector3D position;
    Vector3D color;
    double intensity;
    bool directional;

    Light()
        : position(0, 10, 0), color(1, 1, 1), intensity(1.0), directional(false) {}

    Light(const Vector3D& pos, const Vector3D& col = Vector3D(1, 1, 1),
          double intens = 1.0, bool dir = false)
        : position(pos), color(col), intensity(intens), directional(dir) {}

    static Light sun(const Vector3D& direction, double intens = 1.0,
                     const Vector3D& col = Vector3D(1, 1, 1)) {
        return Light(-direction.normalize(), col, intens, true);
    }

    // Unit vector from `point` toward the light, and how far away it is.
    // The distance bounds the shadow ray: geometry behind the light must not
    // cast a shadow.
    Vector3D directionFrom(const Vector3D& point, double& distance) const {
        if (directional) {
            distance = 1e30;
            return position.normalize();
        }
        Vector3D toLight = position - point;
        distance = toLight.length();
        return toLight.normalize();
    }
};

#endif // LIGHT_H
