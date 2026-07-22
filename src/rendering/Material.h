#ifndef MATERIAL_H
#define MATERIAL_H

#include "../math/Vector3D.h"
#include "Texture.h"

class Material {
public:
    Vector3D color;
    // Surface color source: null means "use `color`" (every pre-existing
    // material keeps behaving exactly as before); set means sample the
    // texture at the hit's UV instead. Replaces the old isGrid/gridColor
    // special case with something a Material can hold any implementation of.
    //
    // Raw, non-owning pointer rather than shared_ptr: Material is copied by
    // value on every single ray-object test (Scene/BVH pass HitRecord around
    // by value), and a shared_ptr's atomic refcount bump on each of those
    // copies measurably cost more than the texture sampling itself did (see
    // the render-time note in the texture task's commit). Textures live for
    // the process lifetime, same as the Objects that hold materials, which
    // this codebase already leaves un-freed (see Scene::addObject) -- so a
    // raw pointer matches the existing ownership style, not a new one.
    Texture* texture = nullptr;
    double reflectivity;
    double refractiveIndex;
    double specular;   // strength of the Blinn-Phong highlight; 0 = matte
    double shininess;  // highlight exponent: larger is tighter
    // How much light passes through rather than scattering off the surface.
    // 0 is opaque; refractiveIndex only means anything once this is above 0.
    double transparency;
    // Beer-Lambert absorption coefficient per channel, applied over the
    // distance a ray travels inside the medium (0 = perfectly clear, the old
    // default). Thicker glass along the ray's path attenuates more.
    Vector3D absorption;

    Material() : color(0,0,0), reflectivity(0.0), refractiveIndex(1.0), specular(0.0), shininess(32.0), transparency(0.0), absorption(0,0,0) {}
    Material(const Vector3D& col, double refl = 0.0, double refr = 1.0)
        : color(col), reflectivity(refl), refractiveIndex(refr), specular(0.0), shininess(32.0), transparency(0.0), absorption(0,0,0) {}

    // Glass, water, diamond: anything the renderer should see through.
    static Material dielectric(const Vector3D& tint, double ior, double trans = 0.95,
                                const Vector3D& absorb = Vector3D(0, 0, 0)) {
        Material m(tint, 0.0, ior);
        m.transparency = trans;
        m.specular = 0.9;
        m.shininess = 200.0;
        m.absorption = absorb;
        return m;
    }
};

#endif // MATERIAL_H
