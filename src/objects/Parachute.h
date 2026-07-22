#ifndef PARACHUTE_H
#define PARACHUTE_H

#include <cmath>
#include "../math/Vector3D.h"
#include "../math/Ray.h"
#include "../rendering/Material.h"
#include "../rendering/HitRecord.h"
#include "Object.h"

class Parachute : public Object {
public:
    Vector3D position;
    double radius;
    Material material;

    Parachute(const Vector3D& pos, double r, const Material& mat) : position(pos), radius(r), material(mat) {}

    // Ray intersection with parachute (treated as sphere)
    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const override {
        Vector3D oc = ray.origin - position;
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
        rec.normal = (rec.point - position) / radius;
        rec.material = material;
        return true;
    }

    bool boundingBox(AABB& out) const override {
        Vector3D r(radius, radius, radius);
        out = AABB(position - r, position + r);
        return true;
    }
};

#endif // PARACHUTE_H
