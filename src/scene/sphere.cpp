#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.


  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
    
    double a = dot(r.d, r.d);
    double b = 2 * dot((r.o - o), r.d);
    double c = dot((r.o - o), (r.o - o)) - r2;
    
    double discriminant = (b*b) - (4*a*c);
    
    if (discriminant < 0) {
        //early exit
        return false;
    } else {
        double t1 = (-b - sqrt(discriminant)) / (2 * a);
        double t2 = (-b + sqrt(discriminant)) / (2 * a);
        
        bool t1_inbound = (t1 >= r.min_t && t1 <= r.max_t);
        bool t2_inbound = (t2 >= r.min_t && t2 <= r.max_t);
        
        return t1_inbound || t2_inbound;
        
    }
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
    
    double a = dot(r.d, r.d);
    double b = 2 * dot((r.o - o), r.d);
    double c = dot((r.o - o), (r.o - o)) - r2;
    
    double discriminant = (b*b) - (4*a*c);
    if (discriminant < 0) {
        //early exit
        return false;
    } else {
        double t1 = (-b - sqrt(discriminant)) / (2 * a);
        if (t1 >= r.min_t && t1 <= r.max_t) {
            r.max_t = t1;
            i->bsdf = get_bsdf();
            
            Vector3D sphere_normal = (r.o + (t1 * r.d) - o);
            sphere_normal.normalize();
            i->n = sphere_normal;
            i->primitive = this;
            i->t = t1;
            return true;
        }
        
        double t2 = (-b + sqrt(discriminant)) / (2 * a);
        if (t2 >= r.min_t && t2 <= r.max_t) {
            r.max_t = t2;
            i->bsdf = get_bsdf();
            
            Vector3D sphere_normal = (r.o + (t2 * r.d) - o);
            sphere_normal.normalize();
            i->n = sphere_normal;
            i->primitive = this;
            i->t = t2;
            return true;
        }
        
        return false;
        

    }
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
