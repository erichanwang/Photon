#ifndef TEXTURE_H
#define TEXTURE_H

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "../math/Vector3D.h"

// What a Material samples for its surface color at a UV coordinate. Replaces
// the old Material::isGrid/gridColor special case with something a Material
// can hold any implementation of.
class Texture {
public:
    virtual ~Texture() {}
    virtual Vector3D sample(double u, double v) const = 0;
};

// Procedural checkerboard. 'scale' is squares per unit of UV: for the ground
// Plane, whose UV is laid out in world units (see Plane::intersect), a scale
// of 1 reproduces the old floor(point/gridSize) pattern exactly.
class CheckerTexture : public Texture {
public:
    Vector3D color1, color2;
    double scale;

    CheckerTexture(const Vector3D& c1, const Vector3D& c2, double sc = 1.0)
        : color1(c1), color2(c2), scale(sc) {}

    Vector3D sample(double u, double v) const override {
        int iu = (int)std::floor(u * scale);
        int iv = (int)std::floor(v * scale);
        // Same sign-safe parity trick as the old grid code: floor() on a
        // negative UV keeps the sign, so an unguarded %2 flips phase across 0.
        return ((iu + iv) % 2 + 2) % 2 == 0 ? color1 : color2;
    }
};

// Image texture loaded from a PPM file (P3 ascii or P6 binary -- this repo's
// own writer emits P3, but real-world texture files are usually the smaller
// P6, so both are read), sampled with bilinear filtering. UV wraps (tiles)
// rather than clamping at the edges.
class ImageTexture : public Texture {
public:
    explicit ImageTexture(const std::string& path) { load(path); }

    Vector3D sample(double u, double v) const override {
        if (width == 0 || height == 0) return Vector3D(1, 0, 1); // missing-texture magenta
        u -= std::floor(u);
        v -= std::floor(v);
        // PPM rows are stored top-down; v=0 is the bottom of the texture by
        // convention, so flip before indexing into the row-major pixel array.
        double fx = u * width - 0.5;
        double fy = (1.0 - v) * height - 0.5;
        int x0 = (int)std::floor(fx), y0 = (int)std::floor(fy);
        double tx = fx - x0, ty = fy - y0;
        Vector3D c00 = texel(x0, y0),     c10 = texel(x0 + 1, y0);
        Vector3D c01 = texel(x0, y0 + 1), c11 = texel(x0 + 1, y0 + 1);
        Vector3D top = c00 * (1.0 - tx) + c10 * tx;
        Vector3D bot = c01 * (1.0 - tx) + c11 * tx;
        return top * (1.0 - ty) + bot * ty;
    }

private:
    int width = 0, height = 0;
    std::vector<Vector3D> pixels; // row-major, top row first, each channel in [0,1]

    Vector3D texel(int x, int y) const {
        x = ((x % width) + width) % width;
        y = ((y % height) + height) % height;
        return pixels[(size_t)y * width + x];
    }

    static int readToken(std::istream& in) {
        // PPM headers allow '#' comments anywhere whitespace is allowed.
        for (;;) {
            in >> std::ws;
            if (in.peek() == '#') { std::string line; std::getline(in, line); continue; }
            break;
        }
        int val;
        in >> val;
        return val;
    }

    void load(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file) throw std::runtime_error("ImageTexture: cannot open " + path);
        std::string magic;
        file >> magic;
        bool binary = (magic == "P6");
        if (!binary && magic != "P3")
            throw std::runtime_error("ImageTexture: unsupported PPM magic in " + path);
        width = readToken(file);
        height = readToken(file);
        int maxVal = readToken(file);
        if (width <= 0 || height <= 0 || maxVal <= 0)
            throw std::runtime_error("ImageTexture: bad header in " + path);
        pixels.resize((size_t)width * height);
        if (binary) {
            file.get(); // the single whitespace byte separating header from binary data
            std::vector<unsigned char> raw((size_t)width * height * 3);
            file.read((char*)raw.data(), (std::streamsize)raw.size());
            if (!file) throw std::runtime_error("ImageTexture: truncated data in " + path);
            for (size_t i = 0; i < pixels.size(); i++) {
                pixels[i] = Vector3D(raw[i * 3] / (double)maxVal,
                                      raw[i * 3 + 1] / (double)maxVal,
                                      raw[i * 3 + 2] / (double)maxVal);
            }
        } else {
            for (size_t i = 0; i < pixels.size(); i++) {
                int r, g, b;
                file >> r >> g >> b;
                if (!file) throw std::runtime_error("ImageTexture: truncated data in " + path);
                pixels[i] = Vector3D(r / (double)maxVal, g / (double)maxVal, b / (double)maxVal);
            }
        }
    }
};

#endif // TEXTURE_H
