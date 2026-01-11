#include <stdlib.h>
#include <stdio.h>
#include "bvh.h"

int box_x_compare(const void *a, const void *b) {
    struct object *oa = (struct object *)a;
    struct object *ob = (struct object *)b;
    struct aabb box_a, box_b;
    if (!object_aabb(oa, 0, 0, &box_a) || !object_aabb(ob, 0, 0, &box_b)) {
        return 0; 
    }
    if (box_a.min.x - box_b.min.x < 0.0) return -1;
    else return 1;
}

int box_y_compare(const void *a, const void *b) {
    struct object *oa = (struct object *)a;
    struct object *ob = (struct object *)b;
    struct aabb box_a, box_b;
    if (!object_aabb(oa, 0, 0, &box_a) || !object_aabb(ob, 0, 0, &box_b)) {
        return 0;
    }
    if (box_a.min.y - box_b.min.y < 0.0) return -1;
    else return 1;
}

int box_z_compare(const void *a, const void *b) {
    struct object *oa = (struct object *)a;
    struct object *ob = (struct object *)b;
    struct aabb box_a, box_b;
    if (!object_aabb(oa, 0, 0, &box_a) || !object_aabb(ob, 0, 0, &box_b)) {
        return 0;
    }
    if (box_a.min.z - box_b.min.z < 0.0) return -1;
    else return 1;
}

struct bvh_node *build_bvh(struct object *objects, size_t n, double t0, double t1, struct rng *rng) {
    struct bvh_node *node = malloc(sizeof(struct bvh_node));
    
    int axis = random_int(rng) % 3;

    if (n == 1) {
        node->left = NULL;
        node->right = NULL;
        node->object = &objects[0]; // Point to the actual object in the array
        node->is_leaf = true;
        object_aabb(&objects[0], t0, t1, &node->box);
    } else if (n == 2) {
        // Optimization for 2 objects: just split them
        node->left = build_bvh(objects, 1, t0, t1, rng);
        node->right = build_bvh(objects + 1, 1, t0, t1, rng);
        node->is_leaf = false;
        node->box = aabb_enclosing(&node->left->box, &node->right->box);
    } else {
        if (axis == 0) qsort(objects, n, sizeof(struct object), box_x_compare);
        else if (axis == 1) qsort(objects, n, sizeof(struct object), box_y_compare);
        else qsort(objects, n, sizeof(struct object), box_z_compare);

        size_t mid = n / 2;
        node->left = build_bvh(objects, mid, t0, t1, rng);
        node->right = build_bvh(objects + mid, n - mid, t0, t1, rng);
        node->is_leaf = false;
        node->box = aabb_enclosing(&node->left->box, &node->right->box);
    }

    return node;
}

bool bvh_hit(struct bvh_node *node, struct ray *r, double t0, double t1, struct hit *hit) {
    if (!aabb_hit(&node->box, r, t0, t1)) {
        return false;
    }

    if (node->is_leaf) {
        return object_hit(node->object, r, t0, t1, hit);
    }

    bool hit_left = bvh_hit(node->left, r, t0, t1, hit);
    bool hit_right = bvh_hit(node->right, r, t0, hit_left ? hit->t : t1, hit);

    return hit_left || hit_right;
}

void free_bvh(struct bvh_node *node) {
    if (!node) return;
    if (!node->is_leaf) {
        free_bvh(node->left);
        free_bvh(node->right);
    }
    free(node);
}
