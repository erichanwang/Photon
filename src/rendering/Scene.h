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
    void buildAcceleration() {
        bvh.build(objects);
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
