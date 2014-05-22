/*
 * PnPProblem.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#ifndef PNPPROBLEM_H_
#define PNPPROBLEM_H_

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "ObjectMesh.h"
#include "ModelRegistration.h"

using namespace cv;
using namespace std;

class PnPProblem
{
  cv::Mat _A_matrix;
  cv::Mat _R_matrix;
  cv::Mat _t_matrix;
  cv::Mat _P_matrix;
public:
  PnPProblem();                                    // default constructor
  PnPProblem(const double param[]);  // custom constructor
  PnPProblem(const PnPProblem& P);                // copy constructor
  virtual ~PnPProblem();

  cv::Mat get_Amatrix() const { return _A_matrix; }
  void set_Amatrix(const double params[]);

  bool estimatePose(const std::vector<std::pair<int, std::pair<cv::Point2f, cv::Point3f> > > &correspondences, int flags = CV_EPNP);

  bool backproject2DPoint(const ObjectMesh *objMesh, const cv::Point2f &point2d, cv::Point3f &point3d);

  std::vector<cv::Point2f> verify_points(ObjectMesh *objMesh);
  cv::Point2f backproject3DPoint(const cv::Point3f &point);
  bool intersect_MollerTrumbore(Ray &R, Triangle &T, double *out);

};

#endif /* PNPPROBLEM_H_ */
