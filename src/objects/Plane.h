#ifndef PLANE_H
#define PLANE_H

#include "../math/Vector3D.h"
#include "../math/Ray.h"
#include "../rendering/Material.h"
#include "../rendering/HitRecord.h"
#include "Object.h"

class Plane : public Object {
public:
    Vector3D point;
    Vector3D normal;
    Material material;

    Plane(const Vector3D& p, const Vector3D& n, const Material& m) : point(p), normal(n), material(m) {}

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& record) const override {
        double denom = normal.dot(ray.direction);
        if (fabs(denom) > 1e-6) {
            Vector3D p0l0 = point - ray.origin;
            double t = p0l0.dot(normal) / denom;
            if (t > tMin && t < tMax) {
                record.t = t;
                record.point = ray.at(t);
                record.normal = normal;
                Vector3D uAxis, vAxis;
                tangentBasis(normal, uAxis, vAxis);
                Vector3D local = record.point - point;
                record.u = local.dot(uAxis);
                record.v = local.dot(vAxis);
                record.material = material;
                return true;
            }
        }
        return false;
    }

    // An infinite plane has no natural UV origin or axes, so pick any pair
    // orthogonal to the normal and to each other. For the ground plane's
    // normal (0,1,0) this must come out to exactly uAxis=(1,0,0),
    // vAxis=(0,0,1) -- not just lined up with the world axes but sign-for-sign
    // identical -- so that u,v equal x,z exactly and a checker texture floors
    // them the same as the old world-space formula, including at exact grid
    // boundaries. A basis that only matched up to a sign flip looked right on
    // non-integer test points but silently flipped the checker's phase for
    // any point sitting exactly on a grid line.
    static void tangentBasis(const Vector3D& n, Vector3D& uAxis, Vector3D& vAxis) {
        Vector3D unitN = n.normalize();
        Vector3D helper = std::fabs(unitN.y) < 0.999 ? Vector3D(0, 1, 0) : Vector3D(0, 0, 1);
        uAxis = unitN.cross(helper).normalize();
        vAxis = uAxis.cross(unitN).normalize();
    }
};

#endif // PLANE_H
