#ifndef HITRECORD_H
#define HITRECORD_H

#include "../math/Vector3D.h"
#include "Material.h"

class Object;

struct HitRecord {
    double t = 0.0;
    Vector3D point;
    Vector3D normal;
    double u = 0.0, v = 0.0;   // surface parameterization, for Material::texture
    Material material;
    Object* object = nullptr;
};

#endif // HITRECORD_H
