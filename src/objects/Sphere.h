#ifndef SPHERE_H
#define SPHERE_H

#include <cmath>
#include <algorithm>
#include "../math/Vector3D.h"
#include "../math/Ray.h"
#include "../rendering/Material.h"
#include "../rendering/HitRecord.h"
#include "Object.h"

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

class Sphere : public Object {
public:
    Vector3D center;
    double radius;
    Material material;

    Sphere(const Vector3D& c, double r, const Material& mat) : center(c), radius(r), material(mat) {}

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const override {
        Vector3D oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return false;
        double sqrtD = std::sqrt(discriminant);
        double root = (-b - sqrtD) / (2 * a);
        if (root < tMin || root > tMax) {
            root = (-b + sqrtD) / (2 * a);
            if (root < tMin || root > tMax) return false;
        }
        rec.t = root;
        rec.point = ray.at(root);
        rec.normal = (rec.point - center) / radius;
        // Standard spherical UV: u wraps around the equator (longitude), v
        // runs 0 at the south pole to 1 at the north pole (latitude). rec.normal
        // is already the unit vector from center to point, so reuse it rather
        // than recomputing.
        rec.u = 0.5 + std::atan2(rec.normal.z, rec.normal.x) / (2.0 * M_PI);
        rec.v = 0.5 + std::asin(std::max(-1.0, std::min(1.0, rec.normal.y))) / M_PI;
        rec.material = material;
        return true;
    }

    bool boundingBox(AABB& out) const override {
        Vector3D r(radius, radius, radius);
        out = AABB(center - r, center + r);
        return true;
    }
};

#endif // SPHERE_H
