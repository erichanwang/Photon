#ifndef BVH_H
#define BVH_H

#include <algorithm>
#include <vector>
#include "../math/AABB.h"
#include "../objects/Object.h"

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
    struct Node {
        AABB box;
        int left = -1;       // index of left child, or -1 for a leaf
        int right = -1;
        int firstObject = 0; // leaves only: range into `order`
        int objectCount = 0;
    };

    std::vector<Node> nodes;
    std::vector<Object*> order;   // objects permuted into leaf-contiguous order

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
    }

    bool empty() const { return nodes.empty(); }

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        if (nodes.empty()) return false;
        return intersectNode(0, ray, tMin, tMax, rec);
    }

    // Depth of the built tree, for the benchmark's reporting.
    int depth(int node = 0) const {
        if (nodes.empty()) return 0;
        const Node& n = nodes[node];
        if (n.left < 0) return 1;
        return 1 + std::max(depth(n.left), depth(n.right));
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
        nodes[index].box = bounds;
        nodes[index].left = l;
        nodes[index].right = r;
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
            nodes[index].box = bounds;
            nodes[index].firstObject = begin;
            nodes[index].objectCount = count;
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
        nodes[index].box = bounds;
        nodes[index].left = l;
        nodes[index].right = r;
        return index;
    }

    bool intersectNode(int index, const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        const Node& n = nodes[index];
        if (!n.box.hit(ray, tMin, tMax)) return false;

        if (n.left < 0) {
            bool hit = false;
            HitRecord temp;
            double closest = tMax;
            for (int i = n.firstObject; i < n.firstObject + n.objectCount; i++) {
                if (order[i]->intersect(ray, tMin, closest, temp)) {
                    hit = true;
                    closest = temp.t;
                    rec = temp;
                }
            }
            return hit;
        }

        // Shrinking tMax after a left hit lets the right subtree's box test
        // reject everything farther away, which is where the speedup comes from.
        bool hitLeft = intersectNode(n.left, ray, tMin, tMax, rec);
        if (hitLeft) tMax = rec.t;
        bool hitRight = intersectNode(n.right, ray, tMin, tMax, rec);
        return hitLeft || hitRight;
    }
};

#endif // BVH_H
