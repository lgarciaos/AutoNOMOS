// libccd
#include <ccd/ccd.h>
#include <ccd/quat.h>
#include <ccd/vec3.h>
// own
#include "obstacle_t.hpp"
// extern int ccdGJKIntersect(const void *obj1, const void *obj2, const ccd_t *ccd) ;

void support(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    // assume that obj_t is user-defined structure that holds info about
    // object (in this case box: x, y, z, pos, quat - dimensions of box,
    // position and rotation)
    obstacle_t *obj = (obstacle_t *)_obj;
    ccd_vec3_t dir, pos;
    ccd_quat_t qinv, q_aux;

    // apply rotation on direction vector
    // ccdVec3Copy(&dir, _dir);
    double x, y, z;
    double qx, qy, qz, qw;
    obj -> get_xyz(x, y, z);
    obj -> get_quat_xyzw(qx, qy, qz, qw);
    ccdVec3Set(&pos, x, y, z);
    ccdVec3Copy(&dir, _dir);
    ccdQuatSet(&q_aux, qx, qy, qz, qw); 
    ccdQuatInvert2(&qinv, &q_aux);
    ccdQuatRotVec(&dir, &qinv);

    // // compute support point in specified direction
    ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * x * CCD_REAL(0.5),
                  ccdSign(ccdVec3Y(&dir)) * y * CCD_REAL(0.5),
                  ccdSign(ccdVec3Z(&dir)) * z * CCD_REAL(0.5));

    // // transform support point according to position and rotation of object
    ccdQuatRotVec(v, &q_aux);
    ccdVec3Add(v, &pos);
}

int main(int argc, char *argv[])
{

  ccd_t ccd;
  CCD_INIT(&ccd); // initialize ccd_t struct

      // set up ccd_t struct
  ccd.support1       = support; // support function for first object
  ccd.support2       = support; // support function for second object
  ccd.max_iterations = 100;     // maximal number of iterations
  obstacle_t obj1, obj2;
  obj1.set_pose(0.5, 0.5, 0, 0, 0, 0, 1);
  obj1.set_bounding_box_point(0, 0, 0);
  obj1.set_bounding_box_point(1, 0, 0);
  obj1.set_bounding_box_point(0, 1, 0);
  obj1.set_bounding_box_point(1, 1, 0);

  obj2.set_pose(0.5, 0.5, 0, 0, 0, 0, 1);
  obj2.set_bounding_box_point(0, 0, 0);
  obj2.set_bounding_box_point(1, 0, 0);
  obj2.set_bounding_box_point(0, 1, 0);
  obj2.set_bounding_box_point(1, 1, 0);

  int intersect = ccdGJKIntersect(&obj1, &obj2, &ccd);

  printf("Intersection is: %d\n", intersect);
  // now intersect holds true if obj1 and obj2 intersect, false otherwise
}