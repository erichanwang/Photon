#ifndef BLOCK_H
#define BLOCK_H

#include <utility>
#include "../math/Vector3D.h"
#include "../math/Ray.h"
#include "../rendering/Material.h"
#include "../rendering/HitRecord.h"
#include "Object.h"

class Block : public Object {
public:
    Vector3D position;
    Vector3D size; // width, height, depth
    Material material;

    Block(const Vector3D& pos, const Vector3D& sz, const Material& mat) : position(pos), size(sz), material(mat) {}

    // Ray intersection with block (axis-aligned bounding box)
    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const override {
        Vector3D min = position - size * 0.5;
        Vector3D max = position + size * 0.5;

        double tmin = (min.x - ray.origin.x) / ray.direction.x;
        double tmax = (max.x - ray.origin.x) / ray.direction.x;

        if (tmin > tmax) std::swap(tmin, tmax);

        double tymin = (min.y - ray.origin.y) / ray.direction.y;
        double tymax = (max.y - ray.origin.y) / ray.direction.y;

        if (tymin > tymax) std::swap(tymin, tymax);

        if ((tmin > tymax) || (tymin > tmax))
            return false;

        if (tymin > tmin)
            tmin = tymin;

        if (tymax < tmax)
            tmax = tymax;

        double tzmin = (min.z - ray.origin.z) / ray.direction.z;
        double tzmax = (max.z - ray.origin.z) / ray.direction.z;

        if (tzmin > tzmax) std::swap(tzmin, tzmax);

        if ((tmin > tzmax) || (tzmin > tmax))
            return false;

        if (tzmin > tmin)
            tmin = tzmin;

        if (tzmax < tmax)
            tmax = tzmax;

        double t = tmin;
        if (t < tMin || t > tMax) return false;

        rec.t = t;
        rec.point = ray.at(t);
        rec.normal = Vector3D(0, 1, 0); // Simple normal, assume top
        rec.material = material;
        return true;
    }
};

#endif // BLOCK_H
