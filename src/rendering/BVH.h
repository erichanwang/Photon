#ifndef BVH_H
#define BVH_H

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>
#include "../math/AABB.h"
#include "../objects/Object.h"
#include "../objects/Sphere.h"

// Bounding volume hierarchy over the scene's bounded objects.
//
// The linear scan it replaces tests every object against every ray: O(n) per
// ray, so doubling the object count doubles the render time. The tree tests a
// box first and skips everything beneath it on a miss, which is O(log n) for
// the rays that miss most of the scene -- nearly all of them.
//
// Nodes live in one flat vector rather than as individually allocated objects:
// traversal is the innermost loop in the renderer, and chasing pointers spread
// across the heap is what makes naive BVHs slower than they should be.
class BVH {
public:
    // 32 bytes so two nodes share one 64-byte cache line, instead of the
    // previous 64-byte node (a double-precision AABB alone was 48 bytes).
    // Two changes get there:
    //  - box bounds stored as float, not double. Traversal only ever uses
    //    them to reject a ray, and a box that's rounded OUTWARD (min down,
    //    max up, see roundDown/roundUp below) can only become more
    //    permissive than the true double-precision box, never reject a ray
    //    the real intersection would accept -- same guarantee buildRange's
    //    kBoxPad already relies on, just covering float rounding too.
    //  - left/right and firstObject/objectCount never coexist (a node is
    //    either a leaf or has two children), so they share two int32
    //    fields. leftOrNegFirst < 0 marks a leaf, mirroring the old
    //    `left == -1` check; its magnitude encodes firstObject so 0 is
    //    still representable (-(begin) - 1, decoded as -(v) - 1).
    struct Node {
        float boxMin[3] = {0, 0, 0};
        float boxMax[3] = {0, 0, 0};
        int32_t leftOrNegFirst = -1;
        int32_t rightOrCount = 0;

        bool isLeaf() const { return leftOrNegFirst < 0; }
        int left() const { return leftOrNegFirst; }
        int right() const { return rightOrCount; }
        int firstObject() const { return -leftOrNegFirst - 1; }
        int objectCount() const { return rightOrCount; }

        // Promotes the stored float box back to the double AABB the ray
        // test math is written in terms of. float -> double widening is
        // exact, so this loses nothing beyond what storing as float already
        // costs (and roundDown/roundUp already made that conservative).
        AABB box() const {
            return AABB(Vector3D(boxMin[0], boxMin[1], boxMin[2]),
                        Vector3D(boxMax[0], boxMax[1], boxMax[2]));
        }
    };

    std::vector<Node> nodes;
    std::vector<Object*> order;   // objects permuted into leaf-contiguous order

    // Leaf-contiguous SoA mirror of `order`, one entry per index, populated
    // once after the tree is built. Spheres are the common case in these
    // scenes, and a leaf test that reads four contiguous doubles per
    // candidate beats one that dereferences a heap-scattered Sphere through
    // a vtable call -- same math (Sphere::intersectAt), fewer cache misses.
    std::vector<double> sphereCenterX, sphereCenterY, sphereCenterZ, sphereRadius;
    std::vector<bool> isSphere;

    static const int kLeafSize = 2;

    // Median split just picks the middle element of the longest axis, no
    // matter how the objects are distributed. SAH bins objects by centroid and
    // picks the split that minimizes expected traversal cost, which handles
    // clustered/uneven distributions median split gets visibly wrong.
    enum class Heuristic { Median, SAH };

    void build(const std::vector<Object*>& objects, Heuristic heuristic = Heuristic::SAH) {
        nodes.clear();
        order.clear();
        for (Object* o : objects) {
            AABB b;
            if (o->boundingBox(b)) order.push_back(o);   // unbounded ones are the caller's problem
        }
        if (order.empty()) return;
        nodes.reserve(order.size() * 2);
        buildRange(0, (int)order.size(), heuristic);

        // order's final permutation is stable once buildRange returns --
        // objects only move while a range is still being partitioned, never
        // after it settles into a leaf -- so this pass runs once per build.
        size_t n = order.size();
        sphereCenterX.assign(n, 0.0);
        sphereCenterY.assign(n, 0.0);
        sphereCenterZ.assign(n, 0.0);
        sphereRadius.assign(n, 0.0);
        isSphere.assign(n, false);
        for (size_t i = 0; i < n; i++) {
            if (Sphere* s = dynamic_cast<Sphere*>(order[i])) {
                sphereCenterX[i] = s->center.x;
                sphereCenterY[i] = s->center.y;
                sphereCenterZ[i] = s->center.z;
                sphereRadius[i] = s->radius;
                isSphere[i] = true;
            }
        }
    }

    bool empty() const { return nodes.empty(); }

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        if (nodes.empty()) return false;
        // Computed once per ray rather than once per node visited -- see
        // AABB::hit's precomputed-invDir overload.
        Vector3D invDir(1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z);
        return intersectNode(0, ray, invDir, tMin, tMax, rec);
    }

    // Batched 4-ray primary-ray traversal. Walks the tree once for all four
    // rays instead of four separate top-to-bottom passes: a node is visited
    // if any active lane's box test (AABB::hit4) still wants it, and each
    // lane's tMax/rec only ever updates from that lane's own box and object
    // tests. That makes this the union of what four independent intersect()
    // calls would visit, with identical per-lane arithmetic throughout (see
    // AABB::hit4's comment) -- so the result is bit-for-bit what calling
    // intersect() four times would produce, just cheaper when the four rays'
    // paths through the tree overlap, which adjacent camera rays' do.
    //
    // `tMax` is read (as the caller's search bound) and written in place
    // (shrunk on every closer hit, exactly like the `closest` variable in
    // the scalar leaf loop below). `activeMask` marks which of the 4 lanes
    // hold a real ray.
    void intersect4(const Ray rays[4], double tMin, double tMax[4], HitRecord rec[4],
                    bool hit[4], int activeMask) const {
        for (int k = 0; k < 4; k++) hit[k] = false;
        if (nodes.empty() || activeMask == 0) return;

        double ox[4], oy[4], oz[4], idx[4], idy[4], idz[4], tMinArr[4];
        for (int k = 0; k < 4; k++) {
            tMinArr[k] = tMin;
            if (!(activeMask & (1 << k))) { ox[k] = oy[k] = oz[k] = idx[k] = idy[k] = idz[k] = 0.0; continue; }
            ox[k] = rays[k].origin.x; oy[k] = rays[k].origin.y; oz[k] = rays[k].origin.z;
            idx[k] = 1.0 / rays[k].direction.x;
            idy[k] = 1.0 / rays[k].direction.y;
            idz[k] = 1.0 / rays[k].direction.z;
        }
        intersectNode4(0, rays, ox, oy, oz, idx, idy, idz, tMinArr, tMax, rec, hit, activeMask);
    }

    // Depth of the built tree, for the benchmark's reporting.
    int depth(int node = 0) const {
        if (nodes.empty()) return 0;
        const Node& n = nodes[node];
        if (n.isLeaf()) return 1;
        return 1 + std::max(depth(n.left()), depth(n.right()));
    }

private:
    static double axisOf(const Vector3D& v, int axis) {
        return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
    }

    // Median split along the longest axis of the centroid bounds.
    int medianSplit(int begin, int end, const AABB& bounds) {
        int index = (int)nodes.size();
        nodes.push_back(Node{});

        AABB centroidBounds;
        for (int i = begin; i < end; i++) {
            AABB b;
            order[i]->boundingBox(b);
            Vector3D c = b.centroid();
            centroidBounds.expand(AABB(c, c));
        }
        int axis = centroidBounds.longestAxis();

        int mid = begin + (end - begin) / 2;
        std::nth_element(order.begin() + begin, order.begin() + mid, order.begin() + end,
                         [axis](Object* a, Object* b) {
                             AABB ba, bb;
                             a->boundingBox(ba);
                             b->boundingBox(bb);
                             return axisOf(ba.centroid(), axis) < axisOf(bb.centroid(), axis);
                         });

        int l = buildRange(begin, mid, Heuristic::Median);
        int r = buildRange(mid, end, Heuristic::Median);
        setBox(index, bounds);
        nodes[index].leftOrNegFirst = l;
        nodes[index].rightOrCount = r;
        return index;
    }

    static const int kSAHBins = 16;
    // Cost model constants. Both are in "cost of one box test" units; their
    // absolute values don't matter, only the ratio between traversal and
    // intersection cost that decides whether splitting is worth it.
    static constexpr double kTraversalCost = 1.0;
    static constexpr double kIntersectionCost = 1.0;

    // Bins objects by centroid position along `axis`, then sweeps the bin
    // boundaries once from each side to get running (area, count) totals.
    // That turns what would be an O(n^2) split search into O(n + bins).
    // Returns true and fills `bestCost`/`bestBoundary` if this axis has any
    // candidate split; false if every centroid falls in the same bin (a
    // degenerate axis, e.g. a flat layer of objects), in which case there is
    // nothing to sweep.
    bool bestSplitOnAxis(int begin, int end, int axis, const AABB& centroidBounds,
                         double& bestCost, double& bestBoundary) const {
        double cMin = axisOf(centroidBounds.min, axis);
        double cMax = axisOf(centroidBounds.max, axis);
        if (cMax - cMin < 1e-12) return false;

        struct Bin { AABB box; int count = 0; };
        Bin bins[kSAHBins];

        auto binIndex = [&](Object* o) {
            AABB b;
            o->boundingBox(b);
            double t = (axisOf(b.centroid(), axis) - cMin) / (cMax - cMin);
            int bi = (int)(t * kSAHBins);
            return std::min(bi, kSAHBins - 1);
        };

        for (int i = begin; i < end; i++) {
            AABB b;
            order[i]->boundingBox(b);
            int bi = binIndex(order[i]);
            bins[bi].box.expand(b);
            bins[bi].count++;
        }

        // Running totals swept from the left and from the right, so the cost
        // of splitting after bin k only needs one lookup on each side rather
        // than re-scanning the whole range for every candidate split.
        AABB leftBox[kSAHBins], rightBox[kSAHBins];
        int leftCount[kSAHBins], rightCount[kSAHBins];

        AABB running;
        int runningCount = 0;
        for (int k = 0; k < kSAHBins; k++) {
            running.expand(bins[k].box);
            runningCount += bins[k].count;
            leftBox[k] = running;
            leftCount[k] = runningCount;
        }
        running = AABB();
        runningCount = 0;
        for (int k = kSAHBins - 1; k >= 0; k--) {
            running.expand(bins[k].box);
            runningCount += bins[k].count;
            rightBox[k] = running;
            rightCount[k] = runningCount;
        }

        bool found = false;
        for (int k = 0; k < kSAHBins - 1; k++) {
            int lc = leftCount[k], rc = rightCount[k + 1];
            if (lc == 0 || rc == 0) continue;
            double cost = kIntersectionCost *
                          (leftBox[k].surfaceArea() * lc + rightBox[k + 1].surfaceArea() * rc);
            if (!found || cost < bestCost) {
                bestCost = cost;
                bestBoundary = cMin + (cMax - cMin) * (double)(k + 1) / kSAHBins;
                found = true;
            }
        }
        return found;
    }

    // Padding applied to every stored node box. SAH's whole point is to fit
    // boxes as tightly as the objects allow, and a tight box can land exactly
    // tangent to a ray that grazes an object's silhouette -- at that point the
    // box-rejection test and the real object intersection are computing the
    // same geometric boundary through different arithmetic, and rounding can
    // make them disagree by an ulp. Padding the box a hair larger than the
    // objects it holds means the box test can only ever be too permissive,
    // never wrongly reject a ray that the real intersection would accept, so
    // it can't produce a different answer from the linear scan.
    static constexpr double kBoxPad = 1e-6;

    // Round a double down/up to the nearest float, nudging by one float ULP
    // if the default (round-to-nearest) cast landed on the wrong side. That
    // guarantees the stored float box is never tighter than the true bounds
    // -- see the Node comment above for why that matters.
    static float roundDown(double v) {
        float f = (float)v;
        if ((double)f > v) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
        return f;
    }
    static float roundUp(double v) {
        float f = (float)v;
        if ((double)f < v) f = std::nextafter(f, std::numeric_limits<float>::infinity());
        return f;
    }

    void setBox(int index, const AABB& b) {
        nodes[index].boxMin[0] = roundDown(b.min.x);
        nodes[index].boxMin[1] = roundDown(b.min.y);
        nodes[index].boxMin[2] = roundDown(b.min.z);
        nodes[index].boxMax[0] = roundUp(b.max.x);
        nodes[index].boxMax[1] = roundUp(b.max.y);
        nodes[index].boxMax[2] = roundUp(b.max.z);
    }

    int buildRange(int begin, int end, Heuristic heuristic) {
        AABB bounds;
        for (int i = begin; i < end; i++) {
            AABB b;
            order[i]->boundingBox(b);
            bounds.expand(b);
        }
        Vector3D pad(kBoxPad, kBoxPad, kBoxPad);
        bounds.min -= pad;
        bounds.max += pad;

        int count = end - begin;
        if (count <= kLeafSize) {
            int index = (int)nodes.size();
            nodes.push_back(Node{});
            setBox(index, bounds);
            nodes[index].leftOrNegFirst = -begin - 1;
            nodes[index].rightOrCount = count;
            return index;
        }

        if (heuristic == Heuristic::Median) {
            return medianSplit(begin, end, bounds);
        }

        AABB centroidBounds;
        for (int i = begin; i < end; i++) {
            AABB b;
            order[i]->boundingBox(b);
            Vector3D c = b.centroid();
            centroidBounds.expand(AABB(c, c));
        }

        int bestAxis = -1;
        double bestCost = 0.0, bestBoundary = 0.0;
        for (int axis = 0; axis < 3; axis++) {
            double cost, boundary;
            if (bestSplitOnAxis(begin, end, axis, centroidBounds, cost, boundary)) {
                if (bestAxis < 0 || cost < bestCost) {
                    bestAxis = axis;
                    bestCost = cost;
                    bestBoundary = boundary;
                }
            }
        }

        // Compare against the cost of not splitting at all. parentArea can be
        // zero for a degenerate (flat) box; that makes every split "free" by
        // this formula, which is fine since a flat box has no good axis to
        // divide on anyway and the partition below will still separate it.
        double parentArea = bounds.surfaceArea();
        double leafCost = kIntersectionCost * count;
        double splitCost = kTraversalCost +
                           (parentArea > 0.0 ? bestCost / parentArea : bestCost);

        if (bestAxis < 0 || splitCost >= leafCost) {
            // No axis had a usable split, or splitting isn't worth it by SAH.
            // A leaf here could still be arbitrarily large (e.g. every
            // centroid identical), so fall back to median split rather than
            // stopping -- that guarantees progress every recursion.
            return medianSplit(begin, end, bounds);
        }

        int axis = bestAxis;
        double boundary = bestBoundary;
        Object** mid = std::partition(order.data() + begin, order.data() + end,
                                      [axis, boundary](Object* o) {
                                          AABB b;
                                          o->boundingBox(b);
                                          return axisOf(b.centroid(), axis) < boundary;
                                      });
        int splitIndex = (int)(mid - order.data());
        // The cost sweep guarantees a boundary with objects on both sides, but
        // guard anyway: floating point bin-boundary math and the partition's
        // own comparator aren't required to agree bit-for-bit.
        if (splitIndex == begin || splitIndex == end) {
            return medianSplit(begin, end, bounds);
        }

        int index = (int)nodes.size();
        nodes.push_back(Node{});
        int l = buildRange(begin, splitIndex, Heuristic::SAH);
        int r = buildRange(splitIndex, end, Heuristic::SAH);
        setBox(index, bounds);
        nodes[index].leftOrNegFirst = l;
        nodes[index].rightOrCount = r;
        return index;
    }

    bool intersectNode(int index, const Ray& ray, const Vector3D& invDir,
                       double tMin, double tMax, HitRecord& rec) const {
        const Node& n = nodes[index];
        if (!n.box().hit(ray, invDir, tMin, tMax)) return false;

        if (n.isLeaf()) {
            bool hit = false;
            HitRecord temp;
            double closest = tMax;
            int firstObject = n.firstObject(), objectCount = n.objectCount();
            for (int i = firstObject; i < firstObject + objectCount; i++) {
                bool candidateHit;
                if (isSphere[i]) {
                    Vector3D center(sphereCenterX[i], sphereCenterY[i], sphereCenterZ[i]);
                    candidateHit = Sphere::intersectAt(center, sphereRadius[i], ray, tMin, closest, temp);
                    if (candidateHit) temp.material = static_cast<Sphere*>(order[i])->material;
                } else {
                    candidateHit = order[i]->intersect(ray, tMin, closest, temp);
                }
                if (candidateHit) {
                    hit = true;
                    closest = temp.t;
                    rec = temp;
                }
            }
            return hit;
        }

        // Shrinking tMax after a left hit lets the right subtree's box test
        // reject everything farther away, which is where the speedup comes from.
        bool hitLeft = intersectNode(n.left(), ray, invDir, tMin, tMax, rec);
        if (hitLeft) tMax = rec.t;
        bool hitRight = intersectNode(n.right(), ray, invDir, tMin, tMax, rec);
        return hitLeft || hitRight;
    }

    // Packet counterpart of intersectNode. `mask` is which lanes are still
    // live for this subtree (already narrowed by the parent's box test);
    // this node's own box test narrows it further before deciding whether to
    // recurse or, at a leaf, which lanes' tMax/rec to update.
    void intersectNode4(int index, const Ray rays[4],
                        const double ox[4], const double oy[4], const double oz[4],
                        const double idx[4], const double idy[4], const double idz[4],
                        const double tMin[4], double tMax[4], HitRecord rec[4], bool hit[4],
                        int mask) const {
        const Node& n = nodes[index];
        AABB box = n.box();
        int active = AABB::hit4(box, ox, oy, oz, idx, idy, idz, tMin, tMax, mask);
        if (active == 0) return;

        if (n.isLeaf()) {
            int firstObject = n.firstObject(), objectCount = n.objectCount();
            for (int k = 0; k < 4; k++) {
                if (!(active & (1 << k))) continue;
                bool anyHit = false;
                HitRecord temp;
                double closest = tMax[k];
                for (int i = firstObject; i < firstObject + objectCount; i++) {
                    bool candidateHit;
                    if (isSphere[i]) {
                        Vector3D center(sphereCenterX[i], sphereCenterY[i], sphereCenterZ[i]);
                        candidateHit = Sphere::intersectAt(center, sphereRadius[i], rays[k], tMin[k], closest, temp);
                        if (candidateHit) temp.material = static_cast<Sphere*>(order[i])->material;
                    } else {
                        candidateHit = order[i]->intersect(rays[k], tMin[k], closest, temp);
                    }
                    if (candidateHit) {
                        anyHit = true;
                        closest = temp.t;
                        rec[k] = temp;
                    }
                }
                if (anyHit) {
                    hit[k] = true;
                    tMax[k] = closest;
                }
            }
            return;
        }

        // Same shrink-then-test-right ordering as the scalar path, applied
        // per lane: each lane's own tMax narrows independently as it finds
        // hits in the left subtree, and the right subtree's box test for
        // that lane sees the narrowed value.
        intersectNode4(n.left(), rays, ox, oy, oz, idx, idy, idz, tMin, tMax, rec, hit, active);
        intersectNode4(n.right(), rays, ox, oy, oz, idx, idy, idz, tMin, tMax, rec, hit, active);
    }
};

#endif // BVH_H
