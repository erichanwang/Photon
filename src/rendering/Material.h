#ifndef MATERIAL_H
#define MATERIAL_H

#include "../math/Vector3D.h"

class Material {
public:
    Vector3D color;
    double reflectivity;
    double refractiveIndex;
    bool isGrid;
    Vector3D gridColor1;
    Vector3D gridColor2;
    double gridSize;
    double specular;   // strength of the Blinn-Phong highlight; 0 = matte
    double shininess;  // highlight exponent: larger is tighter
    // How much light passes through rather than scattering off the surface.
    // 0 is opaque; refractiveIndex only means anything once this is above 0.
    double transparency;
    // Beer-Lambert absorption coefficient per channel, applied over the
    // distance a ray travels inside the medium (0 = perfectly clear, the old
    // default). Thicker glass along the ray's path attenuates more.
    Vector3D absorption;

    Material() : color(0,0,0), reflectivity(0.0), refractiveIndex(1.0), isGrid(false), gridColor1(0,0,0), gridColor2(0,0,0), gridSize(1.0), specular(0.0), shininess(32.0), transparency(0.0), absorption(0,0,0) {}
    Material(const Vector3D& col, double refl = 0.0, double refr = 1.0)
        : color(col), reflectivity(refl), refractiveIndex(refr), isGrid(false), gridColor1(0,0,0), gridColor2(0,0,0), gridSize(1.0), specular(0.0), shininess(32.0), transparency(0.0), absorption(0,0,0) {}

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
