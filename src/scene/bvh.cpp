#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

    //iterate over all objects, count the number of primitives, and compute the mean centroid of all objects for split

    Vector3D mean_centroid;
    vector<Vector3D> centroids; //store all centroid locations for split axis decision
    BBox overall_boundingbox;

    for (std::vector<Primitive *>::iterator p = start; p != end; p++) {
        BBox bounding_box = (*p)->get_bbox();
        centroids.push_back(bounding_box.centroid());
        mean_centroid += bounding_box.centroid();
        overall_boundingbox.expand(bounding_box);
    }

    BVHNode *node = new BVHNode(overall_boundingbox);

    if (centroids.size() < max_leaf_size) {
        node->start = start;
        node->end = end;
        return node;
    }

    mean_centroid /= (float) centroids.size();
    int left_x_count = 0;
    int left_y_count = 0;
    int left_z_count = 0;
    int right_x_count = 0;
    int right_y_count = 0;
    int right_z_count = 0;

    for (int i = 0; i < centroids.size(); i++) {
        Vector3D current_centroid = centroids[i];
        if (current_centroid[0] < mean_centroid[0]) {
            left_x_count += 1;
        } else {
            right_x_count += 1;
        }

        if (current_centroid[1] < mean_centroid[1]) {
            left_y_count += 1;
        } else {
            right_y_count += 1;
        }

        if (current_centroid[2] < mean_centroid[2]) {
            left_z_count += 1;
        } else {
            right_z_count += 1;
        }
    }


    int x_delta = abs(left_x_count - right_x_count);
    int y_delta = abs(left_y_count - right_y_count);
    int z_delta = abs(left_z_count - right_z_count);
    int minimum_diff = std::min(x_delta, std::min(y_delta, z_delta));

    //choose to split on the most balanced axis
    int axis_to_split;
    if (minimum_diff == x_delta) {
        axis_to_split = 0;
    } else if (minimum_diff == y_delta) {
        axis_to_split = 1;
    } else {
        axis_to_split = 2;
    }

    vector<Primitive *> *left = new vector<Primitive *>();
    vector<Primitive *> *right = new vector<Primitive *>();

    for (std::vector<Primitive *>::iterator p = start; p != end; p++) {
        Vector3D current_centroid = (*p)->get_bbox().centroid();
        if (current_centroid[axis_to_split] < mean_centroid[axis_to_split]) {
            left->push_back(*p);
        } else {
            right->push_back(*p);
        }
    }

    node->l = construct_bvh(start, start + left->size(), max_leaf_size);
    node->r = construct_bvh(start + left->size(), end, max_leaf_size);


    return node;


}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

    if (!node->bb.intersect(ray, ray.min_t, ray.max_t)) {
        return false;
    }

    if (node->isLeaf()) {
        for (auto p: primitives) {
            if (p->has_intersection(ray)) {
                return true;
            }
        }
        return false;
    } else {
        return has_intersection(ray, node->l) || has_intersection(ray, node->r);
    }

//
//  for (auto p : primitives) {
//    total_isects++;
//    if (p->has_intersection(ray))
//      return true;
//  }
//  return false;

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
    

    double curr_max_t = ray.max_t;
    double curr_min_t = ray.min_t;
    bool bbIntersect = node->bb.intersect(ray, curr_min_t, curr_max_t);
    if (!bbIntersect) {
          return false;
    }

    if (node->isLeaf()) {
        bool hit = false;
        Intersection *nearest;
        double nearest_t = std::numeric_limits<double>::infinity();
        for (auto p = node->start; p != node->end; p++) {
            if ((*p)->intersect(ray, i)) {
                total_isects++;
                if (i->t < nearest_t) {
                    nearest_t = i->t;
                    nearest = i;
                }
            }
            hit = (*p)->intersect(ray, i) || hit;
        }
        i = nearest;
        return hit;
    } else {
        bool left_check = intersect(ray, i, node->l);
        bool right_check =  intersect(ray, i, node->r);
        return left_check || right_check;
    }

    
//
//
//  bool hit = false;
//  for (auto p : primitives) {
//    total_isects++;
//    hit = p->intersect(ray, i) || hit;
//  }
//  return hit;


}

} // namespace SceneObjects
} // namespace CGL
