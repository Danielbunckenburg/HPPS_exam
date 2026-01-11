#pragma once

#include "scene.h"
#include "geometry.h"
#include "random.h"

struct bvh_node {
    struct aabb box;
    struct bvh_node *left;
    struct bvh_node *right;
    struct object *object; // Only for leaf nodes
    bool is_leaf;
};

// Construct a BVH from a list of objects.
// The objects array might be reordered during construction.
struct bvh_node *build_bvh(struct object *objects, size_t n, double t0, double t1, struct rng *rng);

// Check for intersection with the BVH.
bool bvh_hit(struct bvh_node *node, struct ray *r, double t0, double t1, struct hit *hit);

// Free the BVH memory.
void free_bvh(struct bvh_node *node);
