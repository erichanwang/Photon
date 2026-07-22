#ifndef OBJECT_H
#define OBJECT_H

#include "../math/Ray.h"
#include "../math/AABB.h"
#include "../rendering/HitRecord.h"

class Object {
public:
    virtual ~Object() {}
    virtual bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const = 0;

    // Returns false for objects of infinite extent (an unbounded Plane), which
    // cannot be put in a BVH -- no finite box encloses them. Scene keeps those
    // in a short linear list instead. Defaulting to false means an object that
    // forgets to implement this is merely slow, never wrong.
    virtual bool boundingBox(AABB& out) const { (void)out; return false; }
};

#endif // OBJECT_H
