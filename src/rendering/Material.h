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

    Material() : color(0,0,0), reflectivity(0.0), refractiveIndex(1.0), isGrid(false), gridColor1(0,0,0), gridColor2(0,0,0), gridSize(1.0), specular(0.0), shininess(32.0) {}
    Material(const Vector3D& col, double refl = 0.0, double refr = 1.0)
        : color(col), reflectivity(refl), refractiveIndex(refr), isGrid(false), gridColor1(0,0,0), gridColor2(0,0,0), gridSize(1.0), specular(0.0), shininess(32.0) {}
};

#endif // MATERIAL_H
