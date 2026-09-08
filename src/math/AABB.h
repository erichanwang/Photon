#ifndef AABB_H
#define AABB_H

#include <algorithm>
#include <cmath>
#include "Vector3D.h"
#include "Ray.h"

#if defined(__AVX2__)
#include <immintrin.h>
#endif

// Axis-aligned bounding box. Exists to make the BVH possible: the whole point
// of the tree is that rejecting a box is far cheaper than testing everything
// inside it.
class AABB {
public:
    Vector3D min, max;

    AABB()
        : min(1e30, 1e30, 1e30), max(-1e30, -1e30, -1e30) {}   // empty
    AABB(const Vector3D& lo, const Vector3D& hi) : min(lo), max(hi) {}

    bool isEmpty() const { return min.x > max.x || min.y > max.y || min.z > max.z; }

    Vector3D centroid() const { return (min + max) * 0.5; }

    // Surface area, for the SAH cost function: cost is proportional to the
    // probability a random ray through the parent box also passes through a
    // child, and that probability is the child's area over the parent's.
    double surfaceArea() const {
        double dx = max.x - min.x, dy = max.y - min.y, dz = max.z - min.z;
        return 2.0 * (dx * dy + dy * dz + dz * dx);
    }

    void expand(const AABB& other) {
        min = Vector3D(std::min(min.x, other.min.x),
                       std::min(min.y, other.min.y),
                       std::min(min.z, other.min.z));
        max = Vector3D(std::max(max.x, other.max.x),
                       std::max(max.y, other.max.y),
                       std::max(max.z, other.max.z));
    }

    // Longest axis: 0 = x, 1 = y, 2 = z. Splitting along it keeps child boxes
    // as cube-like as possible, which is what makes the rejection test tight.
    int longestAxis() const {
        double dx = max.x - min.x, dy = max.y - min.y, dz = max.z - min.z;
        if (dx >= dy && dx >= dz) return 0;
        return (dy >= dz) ? 1 : 2;
    }

    // Slab test. Division by a zero direction component yields +/-inf, and the
    // min/max comparisons below handle those correctly, so axis-aligned rays
    // need no special case. NaN only arises for a zero component with the
    // origin exactly on the slab, which the tMax >= tMin test rejects safely.
    bool hit(const Ray& ray, double tMin, double tMax) const {
        Vector3D invDir(1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z);
        return hit(ray, invDir, tMin, tMax);
    }

    // Same test, but takes 1/direction precomputed by the caller. A BVH
    // traversal calls this once per node visited for a ray, and the ray's
    // direction does not change between those calls, so recomputing three
    // divisions at every node -- easily dozens per ray -- was pure repeated
    // work. Hoisting it to once per ray at the top of BVH::intersect is a
    // classic slab-test optimization, not a change in what gets tested.
    bool hit(const Ray& ray, const Vector3D& invDir, double tMin, double tMax) const {
        const double o[3] = { ray.origin.x, ray.origin.y, ray.origin.z };
        const double invD[3] = { invDir.x, invDir.y, invDir.z };
        const double lo[3] = { min.x, min.y, min.z };
        const double hi[3] = { max.x, max.y, max.z };

        for (int a = 0; a < 3; a++) {
            double t0 = (lo[a] - o[a]) * invD[a];
            double t1 = (hi[a] - o[a]) * invD[a];
            if (invD[a] < 0.0) std::swap(t0, t1);
            tMin = t0 > tMin ? t0 : tMin;
            tMax = t1 < tMax ? t1 : tMax;
            if (tMax <= tMin) return false;
        }
        return true;
    }

    // Batched 4-ray version of hit(), for the BVH's primary-ray packet
    // traversal (see BVH::intersect4). Every lane does exactly the same
    // per-axis "t0 > tMin ? t0 : tMin" / "t1 < tMax ? t1 : tMax" the scalar
    // hit() above does, just four at a time -- built from explicit
    // compare-then-blend rather than hardware max/min, so a NaN in one
    // lane's t0/t1 (an axis-aligned ray exactly on a slab) leaves that
    // lane's bound unchanged the same way the scalar ternary does, instead
    // of picking up whatever NaN tie-break behavior MAXPD/MINPD happen to
    // have. No lane's arithmetic depends on any other lane's, so this is
    // bit-for-bit identical to calling hit() four times separately. Early
    // exit per axis is dropped (tMin only grows, tMax only shrinks, so a
    // rejection after axis 0 is still a rejection after axis 2) -- see
    // buildRange's kBoxPad comment for why exact equivalence, not just
    // "close enough", is the bar here.
    //
    // `activeMask` marks which of the 4 lanes hold a real ray; a partial
    // packet at a row's tail pads the rest so they can never register a hit.
    // Returns the subset of activeMask whose ray currently clears the box.
    static int hit4(const AABB& box, const double ox[4], const double oy[4], const double oz[4],
                    const double idx[4], const double idy[4], const double idz[4],
                    const double tMinIn[4], const double tMaxIn[4], int activeMask) {
#if defined(__AVX2__)
        __m256d tmin = _mm256_loadu_pd(tMinIn);
        __m256d tmax = _mm256_loadu_pd(tMaxIn);
        const double lo[3] = { box.min.x, box.min.y, box.min.z };
        const double hi[3] = { box.max.x, box.max.y, box.max.z };
        const double* org[3] = { ox, oy, oz };
        const double* invd[3] = { idx, idy, idz };
        for (int a = 0; a < 3; a++) {
            __m256d o = _mm256_loadu_pd(org[a]);
            __m256d id = _mm256_loadu_pd(invd[a]);
            __m256d loV = _mm256_set1_pd(lo[a]);
            __m256d hiV = _mm256_set1_pd(hi[a]);
            __m256d t0 = _mm256_mul_pd(_mm256_sub_pd(loV, o), id);
            __m256d t1 = _mm256_mul_pd(_mm256_sub_pd(hiV, o), id);
            __m256d neg = _mm256_cmp_pd(id, _mm256_setzero_pd(), _CMP_LT_OQ);
            __m256d t0s = _mm256_blendv_pd(t0, t1, neg);
            __m256d t1s = _mm256_blendv_pd(t1, t0, neg);
            __m256d gt = _mm256_cmp_pd(t0s, tmin, _CMP_GT_OQ);
            tmin = _mm256_blendv_pd(tmin, t0s, gt);
            __m256d lt = _mm256_cmp_pd(t1s, tmax, _CMP_LT_OQ);
            tmax = _mm256_blendv_pd(tmax, t1s, lt);
        }
        __m256d valid = _mm256_cmp_pd(tmax, tmin, _CMP_GT_OQ);
        int mask = _mm256_movemask_pd(valid);
        return mask & activeMask;
#else
        int mask = 0;
        for (int k = 0; k < 4; k++) {
            if (!(activeMask & (1 << k))) continue;
            double o[3] = { ox[k], oy[k], oz[k] };
            double invD[3] = { idx[k], idy[k], idz[k] };
            double lo[3] = { box.min.x, box.min.y, box.min.z };
            double hi[3] = { box.max.x, box.max.y, box.max.z };
            double tMin = tMinIn[k], tMax = tMaxIn[k];
            for (int a = 0; a < 3; a++) {
                double t0 = (lo[a] - o[a]) * invD[a];
                double t1 = (hi[a] - o[a]) * invD[a];
                if (invD[a] < 0.0) std::swap(t0, t1);
                tMin = t0 > tMin ? t0 : tMin;
                tMax = t1 < tMax ? t1 : tMax;
            }
            if (tMax > tMin) mask |= (1 << k);
        }
        return mask;
#endif
    }
};

#endif // AABB_H
