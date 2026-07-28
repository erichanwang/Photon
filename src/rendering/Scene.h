#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include "../objects/Object.h"
#include "../physics/PhysicsEngine.h"
#include "HitRecord.h"
#include "BVH.h"
#include "Light.h"

class Scene {
public:
    std::vector<Object*> objects;
    std::vector<Light> lights;
    PhysicsEngine physics;

    void addObject(Object* obj) {
        objects.push_back(obj);
        accelerationValid = false;
    }

    void addLight(const Light& light) { lights.push_back(light); }

    // Build once, then reuse. Physics moves objects between frames, so an
    // animated scene has to rebuild -- hence an explicit call rather than a
    // build hidden inside the first intersect(), which would silently go stale.
    void buildAcceleration(BVH::Heuristic heuristic = BVH::Heuristic::SAH) {
        bvh.build(objects, heuristic);
        unbounded.clear();
        for (Object* o : objects) {
            AABB b;
            if (!o->boundingBox(b)) unbounded.push_back(o);
        }
        accelerationValid = true;
    }

    void invalidateAcceleration() { accelerationValid = false; }
    const BVH& acceleration() const { return bvh; }
    bool accelerated() const { return accelerationValid; }

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        if (!accelerationValid) return linearIntersect(ray, tMin, tMax, rec);

        bool hit = false;
        double closest = tMax;
        HitRecord temp;

        if (bvh.intersect(ray, tMin, closest, temp)) {
            hit = true;
            closest = temp.t;
            rec = temp;
        }
        // Infinite-extent objects have no finite box to put in the tree, so
        // they stay on a short linear list. There are only ever a couple.
        for (Object* o : unbounded) {
            if (o->intersect(ray, tMin, closest, temp)) {
                hit = true;
                closest = temp.t;
                rec = temp;
            }
        }
        return hit;
    }

    // Batched primary-ray counterpart of intersect(), for the renderer's
    // packet path (see RayTracer::renderPixelPacket4). Same two-stage shape
    // as intersect() above -- BVH first, then the short unbounded list --
    // just with the BVH stage vectorized across the 4 rays. Each lane's
    // tMax/rec updates in the same order intersect() would produce them in,
    // so calling this once on a 4-ray packet is equivalent to calling
    // intersect() four times.
    void intersect4(const Ray rays[4], double tMin, double tMax[4], HitRecord rec[4],
                    bool hit[4], int activeMask) const {
        for (int k = 0; k < 4; k++) hit[k] = false;
        if (!accelerationValid) {
            for (int k = 0; k < 4; k++) {
                if (!(activeMask & (1 << k))) continue;
                hit[k] = linearIntersect(rays[k], tMin, tMax[k], rec[k]);
            }
            return;
        }

        bvh.intersect4(rays, tMin, tMax, rec, hit, activeMask);
        for (Object* o : unbounded) {
            for (int k = 0; k < 4; k++) {
                if (!(activeMask & (1 << k))) continue;
                HitRecord temp;
                if (o->intersect(rays[k], tMin, tMax[k], temp)) {
                    hit[k] = true;
                    tMax[k] = temp.t;
                    rec[k] = temp;
                }
            }
        }
    }

    // The pre-BVH path, kept so the test suite can assert the accelerated
    // result is identical to the exhaustive one rather than merely plausible.
    bool linearIntersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        HitRecord tempRec;
        bool hitAnything = false;
        double closestSoFar = tMax;

        for (const auto& object : objects) {
            if (object->intersect(ray, tMin, closestSoFar, tempRec)) {
                hitAnything = true;
                closestSoFar = tempRec.t;
                rec = tempRec;
            }
        }
        return hitAnything;
    }

    // True if anything blocks the segment from `origin` along `dir` up to
    // `maxDistance`. A shadow ray only asks whether the light is occluded, not
    // by what, so this can stop at the first hit.
    bool occluded(const Vector3D& origin, const Vector3D& dir, double maxDistance) const {
        HitRecord rec;
        Ray shadowRay(origin, dir);
        return intersect(shadowRay, 1e-4, maxDistance - 1e-4, rec);
    }

private:
    BVH bvh;
    std::vector<Object*> unbounded;
    bool accelerationValid = false;
};

#endif // SCENE_H
