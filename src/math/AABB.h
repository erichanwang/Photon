#ifndef AABB_H
#define AABB_H

#include <algorithm>
#include <cmath>
#include "Vector3D.h"
#include "Ray.h"

// Axis-aligned bounding box. Exists to make the BVH possible: the whole point
// of the tree is that rejecting a box is far cheaper than testing everything
// inside it.
class AABB {
public:
    Vector3D min, max;

    AABB()
        : min(1e30, 1e30, 1e30), max(-1e30, -1e30, -1e30) {}   // empty
    AABB(const Vector3D& lo, const Vector3D& hi) : min(lo), max(hi) {}

    bool isEmpty() const { return min.x > max.x || min.y > max.y || min.z > max.z; }

    Vector3D centroid() const { return (min + max) * 0.5; }

    void expand(const AABB& other) {
        min = Vector3D(std::min(min.x, other.min.x),
                       std::min(min.y, other.min.y),
                       std::min(min.z, other.min.z));
        max = Vector3D(std::max(max.x, other.max.x),
                       std::max(max.y, other.max.y),
                       std::max(max.z, other.max.z));
    }

    // Longest axis: 0 = x, 1 = y, 2 = z. Splitting along it keeps child boxes
    // as cube-like as possible, which is what makes the rejection test tight.
    int longestAxis() const {
        double dx = max.x - min.x, dy = max.y - min.y, dz = max.z - min.z;
        if (dx >= dy && dx >= dz) return 0;
        return (dy >= dz) ? 1 : 2;
    }

    // Slab test. Division by a zero direction component yields +/-inf, and the
    // min/max comparisons below handle those correctly, so axis-aligned rays
    // need no special case. NaN only arises for a zero component with the
    // origin exactly on the slab, which the tMax >= tMin test rejects safely.
    bool hit(const Ray& ray, double tMin, double tMax) const {
        const double o[3] = { ray.origin.x, ray.origin.y, ray.origin.z };
        const double d[3] = { ray.direction.x, ray.direction.y, ray.direction.z };
        const double lo[3] = { min.x, min.y, min.z };
        const double hi[3] = { max.x, max.y, max.z };

        for (int a = 0; a < 3; a++) {
            double invD = 1.0 / d[a];
            double t0 = (lo[a] - o[a]) * invD;
            double t1 = (hi[a] - o[a]) * invD;
            if (invD < 0.0) std::swap(t0, t1);
            tMin = t0 > tMin ? t0 : tMin;
            tMax = t1 < tMax ? t1 : tMax;
            if (tMax <= tMin) return false;
        }
        return true;
    }
};

#endif // AABB_H
