/*
 * PnPProblem.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#ifndef PNPPROBLEM_H_
#define PNPPROBLEM_H_

#include <iostream>

#include <cv.h>
#include <highgui.h>

#include "ObjectMesh.h"
#include "ModelRegistration.h"

using namespace cv;
using namespace std;

class PnPProblem
{
  Mat _A_matrix;
  Mat _P_matrix;
public:
  PnPProblem();                                    // default constructor
  PnPProblem(const double param[]);  // custom constructor
  PnPProblem(const PnPProblem& P);                // copy constructor
  virtual ~PnPProblem();

  Mat get_Amatrix() const { return _A_matrix; }
  void set_Amatrix(const double params[]);

  bool estimatePose(const vector<pair<int,pair<Point2f,Point3f> > > &correspondences, int flags = CV_EPNP);

  bool backproject2DPoint(const ObjectMesh *objMesh, const Point2f point2d, Point3f point3d) const;

  vector<Point2f> verify_points(ObjectMesh *objMesh);
  Point2f backproject3DPoint(const Point3f &point);
  bool intersect3D_RayTriangle(Ray R, Triangle T, Point2f *I);
  void check_intersect3D(Point2f ray_dir, ObjectMesh *objMesh);


};

#endif /* PNPPROBLEM_H_ */
