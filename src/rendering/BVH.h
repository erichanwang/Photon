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

    void build(const std::vector<Object*>& objects) {
        nodes.clear();
        order.clear();
        for (Object* o : objects) {
            AABB b;
            if (o->boundingBox(b)) order.push_back(o);   // unbounded ones are the caller's problem
        }
        if (order.empty()) return;
        nodes.reserve(order.size() * 2);
        buildRange(0, (int)order.size());
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
    // Median split along the longest axis of the centroid bounds. Cheaper to
    // build than a full surface-area-heuristic sweep and close enough on the
    // scene sizes here; SAH is the upgrade if scenes get much larger.
    int buildRange(int begin, int end) {
        int index = (int)nodes.size();
        nodes.push_back(Node{});

        AABB bounds;
        for (int i = begin; i < end; i++) {
            AABB b;
            order[i]->boundingBox(b);
            bounds.expand(b);
        }

        int count = end - begin;
        if (count <= kLeafSize) {
            nodes[index].box = bounds;
            nodes[index].firstObject = begin;
            nodes[index].objectCount = count;
            return index;
        }

        AABB centroidBounds;
        for (int i = begin; i < end; i++) {
            AABB b;
            order[i]->boundingBox(b);
            Vector3D c = b.centroid();
            centroidBounds.expand(AABB(c, c));
        }
        int axis = centroidBounds.longestAxis();

        int mid = begin + count / 2;
        std::nth_element(order.begin() + begin, order.begin() + mid, order.begin() + end,
                         [axis](Object* a, Object* b) {
                             AABB ba, bb;
                             a->boundingBox(ba);
                             b->boundingBox(bb);
                             Vector3D ca = ba.centroid(), cb = bb.centroid();
                             double va = axis == 0 ? ca.x : (axis == 1 ? ca.y : ca.z);
                             double vb = axis == 0 ? cb.x : (axis == 1 ? cb.y : cb.z);
                             return va < vb;
                         });

        int l = buildRange(begin, mid);
        int r = buildRange(mid, end);
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
