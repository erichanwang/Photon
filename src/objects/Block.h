#ifndef BLOCK_H
#define BLOCK_H

#include <utility>
#include <cmath>
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
        if (t < tMin) t = tmax;          // origin inside the box: use the exit face
        if (t < tMin || t > tMax) return false;

        rec.t = t;
        rec.point = ray.at(t);
        rec.normal = faceNormal(rec.point, min, max);
        // Box UV: project the hit onto whichever two axes the face's normal
        // isn't along, normalized by that face's extent so each face covers
        // UV [0,1]x[0,1] independently (seams at face edges, as usual for box
        // mapping).
        Vector3D local = rec.point - min;
        if (std::fabs(rec.normal.x) > 0.5) {
            rec.u = local.z / size.z;
            rec.v = local.y / size.y;
        } else if (std::fabs(rec.normal.y) > 0.5) {
            rec.u = local.x / size.x;
            rec.v = local.z / size.z;
        } else {
            rec.u = local.x / size.x;
            rec.v = local.y / size.y;
        }
        rec.material = material;
        return true;
    }

    bool boundingBox(AABB& out) const override {
        out = AABB(position - size * 0.5, position + size * 0.5);
        return true;
    }

private:
    // Which of the six faces the hit point lies on, by whichever coordinate is
    // closest to its slab boundary. The previous hardcoded (0,1,0) was
    // invisible under flat shading but makes every face light like the top one
    // the moment a real lighting model is introduced.
    static Vector3D faceNormal(const Vector3D& p, const Vector3D& min, const Vector3D& max) {
        double dxMin = std::fabs(p.x - min.x), dxMax = std::fabs(p.x - max.x);
        double dyMin = std::fabs(p.y - min.y), dyMax = std::fabs(p.y - max.y);
        double dzMin = std::fabs(p.z - min.z), dzMax = std::fabs(p.z - max.z);

        double best = dxMin;
        Vector3D n(-1, 0, 0);
        if (dxMax < best) { best = dxMax; n = Vector3D(1, 0, 0); }
        if (dyMin < best) { best = dyMin; n = Vector3D(0, -1, 0); }
        if (dyMax < best) { best = dyMax; n = Vector3D(0, 1, 0); }
        if (dzMin < best) { best = dzMin; n = Vector3D(0, 0, -1); }
        if (dzMax < best) { n = Vector3D(0, 0, 1); }
        return n;
    }
};

#endif // BLOCK_H
