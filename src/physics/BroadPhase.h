#ifndef BROADPHASE_H
#define BROADPHASE_H

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "RigidBody.h"

// Uniform spatial hash broad phase, replacing the O(n^2) all-pairs loop.
// Every body is dropped into every grid cell its bounding sphere touches;
// candidate pairs are collected from bodies that ever share a cell.
//
// This can only ever produce a superset of the pairs whose bounding spheres
// actually overlap (see testBroadPhaseCoversAllCollidingPairs): if two
// spheres overlap they share at least one cell, so the pair is reported.
// It can report false positives (bodies in the same cell that don't
// actually overlap) but the narrow phase already filters those out cheaply,
// same as the old loop did.
//
// ponytail: fixed cell size, tuned by whoever calls this for their bodies'
// typical size. A body much larger than the cell just lands in more cells -
// still correct, just less of a win. A hierarchical grid would help bodies
// of wildly varying scale, but nothing here needs that.
class SpatialHashBroadPhase {
public:
    double cellSize;
    explicit SpatialHashBroadPhase(double cellSize_ = 2.0) : cellSize(cellSize_) {}

    // Returns each candidate pair once, as (i, j) with i < j indexing `bodies`.
    std::vector<std::pair<size_t, size_t>> findPairs(const std::vector<RigidBody*>& bodies) {
        cells.clear();
        cells.reserve(bodies.size() * 2);

        for (size_t i = 0; i < bodies.size(); i++) {
            RigidBody* body = bodies[i];
            double r = body->boundingRadius();
            long x0 = cellCoord(body->position.x - r), x1 = cellCoord(body->position.x + r);
            long y0 = cellCoord(body->position.y - r), y1 = cellCoord(body->position.y + r);
            long z0 = cellCoord(body->position.z - r), z1 = cellCoord(body->position.z + r);

            for (long x = x0; x <= x1; x++)
                for (long y = y0; y <= y1; y++)
                    for (long z = z0; z <= z1; z++)
                        cells[key(x, y, z)].push_back(i);
        }

        std::vector<std::pair<size_t, size_t>> pairs;
        std::unordered_set<uint64_t> seen;
        seen.reserve(bodies.size() * 4);

        for (auto& entry : cells) {
            const std::vector<size_t>& occupants = entry.second;
            for (size_t a = 0; a < occupants.size(); a++) {
                for (size_t b = a + 1; b < occupants.size(); b++) {
                    size_t i = occupants[a], j = occupants[b];
                    if (i > j) std::swap(i, j);
                    uint64_t pairKey = (uint64_t(i) << 32) | uint64_t(j);
                    if (seen.insert(pairKey).second) pairs.emplace_back(i, j);
                }
            }
        }
        return pairs;
    }

private:
    std::unordered_map<int64_t, std::vector<size_t>> cells;

    long cellCoord(double v) const {
        return (long)std::floor(v / cellSize);
    }

    // Packs three cell coordinates into one key. Cell ranges here are tiny
    // relative to a 64-bit int, so a coarse bit-interleave with generous
    // headroom per axis is enough to avoid collisions.
    static int64_t key(long x, long y, long z) {
        return (int64_t(x) & 0x1FFFFF) | ((int64_t(y) & 0x1FFFFF) << 21) | ((int64_t(z) & 0x1FFFFF) << 42);
    }
};

#endif // BROADPHASE_H
