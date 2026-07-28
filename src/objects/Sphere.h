#ifndef SPHERE_H
#define SPHERE_H

#include <cmath>
#include "../math/Vector3D.h"
#include "../math/Ray.h"
#include "../rendering/Material.h"
#include "../rendering/HitRecord.h"
#include "Object.h"

class Sphere : public Object {
public:
    Vector3D center;
    double radius;
    Material material;

    Sphere(const Vector3D& c, double r, const Material& mat) : center(c), radius(r), material(mat) {}

    // Geometry-only intersection, split out from intersect() so the BVH's
    // SoA leaf path can call the exact same math against a center/radius
    // pulled from flat arrays instead of a heap-scattered Sphere. Leaves
    // rec.material untouched -- callers that own a Sphere fill that in.
    static bool intersectAt(const Vector3D& center, double radius, const Ray& ray,
                            double tMin, double tMax, HitRecord& rec) {
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
        return true;
    }

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const override {
        if (!intersectAt(center, radius, ray, tMin, tMax, rec)) return false;
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
