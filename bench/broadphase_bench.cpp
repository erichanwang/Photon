// Broad-phase benchmark: O(n^2) all-pairs loop vs. the spatial hash that
// replaced it. An O(n^2) loop over a handful of bodies is pure arithmetic
// with no bookkeeping overhead, so it wins at small n; the point of this
// benchmark is to find and report the real body count where the spatial
// hash starts winning, not to assert it always does.
#include <chrono>
#include <cstdio>
#include <random>
#include <set>
#include <vector>
#include "../src/physics/RigidBody.h"
#include "../src/physics/BroadPhase.h"

// Scatters bodies through a volume that scales with body count, so density
// (and therefore the fraction of pairs that are actual neighbors) stays
// roughly comparable across the counts tested, rather than a fixed volume
// making 10k bodies one solid overlapping mass.
static std::vector<RigidBody*> makeBodies(int n, unsigned seed) {
    std::mt19937 rng(seed);
    double side = std::cbrt((double)n) * 3.0;
    std::uniform_real_distribution<double> pos(-side, side);
    std::uniform_real_distribution<double> rad(0.3, 1.0);

    std::vector<RigidBody*> bodies;
    bodies.reserve(n);
    for (int i = 0; i < n; i++) {
        RigidBody* b = new RigidBody(Vector3D(pos(rng), pos(rng), pos(rng)), 1.0);
        b->radius = rad(rng);
        bodies.push_back(b);
    }
    return bodies;
}

static std::set<std::pair<size_t, size_t>> bruteForcePairs(const std::vector<RigidBody*>& bodies) {
    std::set<std::pair<size_t, size_t>> pairs;
    for (size_t i = 0; i < bodies.size(); i++)
        for (size_t j = i + 1; j < bodies.size(); j++) {
            double d = (bodies[j]->position - bodies[i]->position).length();
            if (d < bodies[i]->boundingRadius() + bodies[j]->boundingRadius())
                pairs.insert({i, j});
        }
    return pairs;
}

static const int kRepeats = 5;

int main() {
    std::printf("Broad phase: O(n^2) all-pairs loop vs. spatial hash, best of %d runs.\n\n", kRepeats);
    std::printf("%8s %14s %14s %10s %12s\n", "bodies", "O(n^2) ms", "hash ms", "speedup", "candidates");

    int counts[] = {100, 1000, 10000};
    for (int n : counts) {
        std::vector<RigidBody*> bodies = makeBodies(n, 42);

        double bruteBest = 1e30;
        std::set<std::pair<size_t, size_t>> bruteResult;
        for (int r = 0; r < kRepeats; r++) {
            auto start = std::chrono::steady_clock::now();
            auto result = bruteForcePairs(bodies);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            if (ms < bruteBest) { bruteBest = ms; bruteResult = std::move(result); }
        }

        SpatialHashBroadPhase broad(2.0);
        double hashBest = 1e30;
        size_t candidateCount = 0;
        for (int r = 0; r < kRepeats; r++) {
            auto start = std::chrono::steady_clock::now();
            auto pairs = broad.findPairs(bodies);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            if (ms < hashBest) { hashBest = ms; candidateCount = pairs.size(); }
        }

        // Correctness gate, not just a speed number: every pair the brute
        // force found as actually overlapping must be among the hash's
        // candidates, or this "optimization" is a tunnelling bug.
        auto pairs = broad.findPairs(bodies);
        std::set<std::pair<size_t, size_t>> candidateSet(pairs.begin(), pairs.end());
        for (auto& pr : bruteResult) {
            if (!candidateSet.count(pr)) {
                std::printf("ABORT: spatial hash missed a real colliding pair at n=%d\n", n);
                return 1;
            }
        }

        std::printf("%8d %14.4f %14.4f %9.2fx %12zu\n",
                    n, bruteBest, hashBest, bruteBest / hashBest, candidateCount);

        for (auto b : bodies) delete b;
    }

    // Binary-search the crossover between the two counts already measured
    // above, rather than eyeballing which row flipped sign.
    std::printf("\nLocating the crossover body count...\n");
    int lo = 100, hi = 10000;
    // Establish that lo favors brute force and hi favors the hash; otherwise
    // there is no crossover in this range to report.
    auto timeBoth = [](int n) {
        std::vector<RigidBody*> bodies = makeBodies(n, 42);
        double bruteBest = 1e30, hashBest = 1e30;
        SpatialHashBroadPhase broad(2.0);
        for (int r = 0; r < kRepeats; r++) {
            auto start = std::chrono::steady_clock::now();
            bruteForcePairs(bodies);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            bruteBest = std::min(bruteBest, ms);

            start = std::chrono::steady_clock::now();
            broad.findPairs(bodies);
            ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            hashBest = std::min(hashBest, ms);
        }
        for (auto b : bodies) delete b;
        return std::make_pair(bruteBest, hashBest);
    };

    auto [loBrute, loHash] = timeBoth(lo);
    auto [hiBrute, hiHash] = timeBoth(hi);
    if (loBrute >= loHash) {
        std::printf("hash already wins at n=%d (%.4f ms vs %.4f ms) - no lower crossover in this range\n",
                    lo, loBrute, loHash);
    } else if (hiBrute <= hiHash) {
        std::printf("brute force still wins at n=%d (%.4f ms vs %.4f ms) - no crossover in this range\n",
                    hi, hiBrute, hiHash);
    } else {
        while (hi - lo > 25) {
            int mid = (lo + hi) / 2;
            auto [brute, hash] = timeBoth(mid);
            std::printf("  n=%6d  O(n^2)=%9.4f ms  hash=%9.4f ms  %s\n",
                        mid, brute, hash, brute < hash ? "brute wins" : "hash wins");
            if (brute < hash) lo = mid; else hi = mid;
        }
        std::printf("crossover is between n=%d (brute force wins) and n=%d (hash wins)\n", lo, hi);
    }

    return 0;
}
