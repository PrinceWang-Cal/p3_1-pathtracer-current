#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
    
    Vector3D bbox_min_t = (this->min - r.o) / r.d;
    Vector3D bbox_max_t = (this->max - r.o) / r.d;
    
    double xmin = std::min(bbox_min_t[0], bbox_max_t[0]);
    double xmax = std::max(bbox_min_t[0], bbox_max_t[0]);
    
    double ymin = std::min(bbox_min_t[1], bbox_max_t[1]);
    double ymax = std::max(bbox_min_t[1], bbox_max_t[1]);
    
    double zmin = std::min(bbox_min_t[2], bbox_max_t[2]);
    double zmax = std::max(bbox_min_t[2], bbox_max_t[2]);
    
    double tmin = std::max(xmin, std::max(ymin, zmin));
    double tmax = std::min(xmax, std::min(ymax, zmax));
    
    if (tmin > t1 || tmax < t0) {
        return false;
    } else if (tmax < tmin) {
        return false;
    }
    t0 = std::max(t0, tmin);
    t1 = std::min(t1, tmax);

    return true;

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
