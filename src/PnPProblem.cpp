/*
 * PnPProblem.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#include <iostream>
#include <sstream>

#include "PnPProblem.h"
#include "ObjectMesh.h"

using namespace std;

Point3f CROSS(Point3f v1, Point3f v2)
{
  Point3f tmp_p;
  tmp_p.x =  v1.y*v2.z - v1.z*v2.y;
  tmp_p.y =  v1.z*v2.x - v1.x*v2.z;
  tmp_p.z =  v1.x*v2.y - v1.y*v2.x;
  return tmp_p;
}

double DOT(Point3f v1, Point3f v2)
{
  return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

Point3f SUB(Point3f v1, Point3f v2)
{
  Point3f tmp_p;
  tmp_p.x =  v1.x - v2.x;
  tmp_p.y =  v1.y - v2.y;
  tmp_p.z =  v1.z - v2.z;
  return tmp_p;
}

PnPProblem::PnPProblem()  // default constructor
{
  _A_matrix = Mat::zeros(3,3,DataType<double>::type);
  _P_matrix = Mat::zeros(3,4,DataType<double>::type);
  _A_matrix.at<double>(0,0) = 1;  // fx
  _A_matrix.at<double>(1,1) = 1;  // fy
  _A_matrix.at<double>(1,2) = 1;  // cx
  _A_matrix.at<double>(0,2) = 1;  // cy
  _A_matrix.at<double>(2,2) = 1;
}

PnPProblem::PnPProblem(const double params[])  // custom constructor
{
  _A_matrix = Mat::zeros(3,3,DataType<double>::type);
  _P_matrix = Mat::zeros(3,4,DataType<double>::type);
  _A_matrix.at<double>(0,0) = params[0];  // fx
  _A_matrix.at<double>(1,1) = params[1];  // fy
  _A_matrix.at<double>(0,2) = params[2];  // cx
  _A_matrix.at<double>(1,2) = params[3];  // cy
  _A_matrix.at<double>(2,2) = 1;
}

PnPProblem::PnPProblem(const PnPProblem& P)  // copy constructor
{
  _A_matrix = P._A_matrix;
  _P_matrix = P._P_matrix;
}


PnPProblem::~PnPProblem()
{
  // TODO Auto-generated destructor stub
}

void PnPProblem::set_Amatrix(const double params[])  // custom constructor
{
  _A_matrix.at<double>(0,0) = params[0];  // fx
  _A_matrix.at<double>(1,1) = params[1];  // fy
  _A_matrix.at<double>(1,2) = params[2];  // cx
  _A_matrix.at<double>(0,2) = params[3];  // cy
  _A_matrix.at<double>(2,2) = 1;
}

bool PnPProblem::estimatePose(const vector<pair<int,pair<Point2f,Point3f> > > &correspondences, int flags)
{

  Mat distCoeffs = Mat::zeros(4,1,DataType<double>::type);
  Mat rvec = Mat::zeros(3,1,cv::DataType<double>::type);
  Mat tvec = Mat::zeros(3,1,cv::DataType<double>::type);
  Mat R_Matrix = Mat::zeros(3,3,DataType<double>::type);

  vector<Point2f> points_2d;
  vector<Point3f> points_3d;

  for(size_t i = 0; i < correspondences.size(); i++)
  {
    cout << "Correspondence " << correspondences.at(i).first << endl;;
    cout << "P 2D " << correspondences.at(i).second.first << endl;
    cout << "P 3D " << correspondences.at(i).second.second << endl;

    points_2d.push_back(correspondences.at(i).second.first);
    points_3d.push_back(correspondences.at(i).second.second);
  }

  bool useExtrinsicGuess = false;
  cout << "A = "<< endl << " "  << _A_matrix << endl << endl;
  //cout << "Distance coefficients = "<< endl << " "  << distCoeffs << endl << endl;

  bool correspondence = solvePnP(points_3d, points_2d, _A_matrix, distCoeffs, rvec, tvec, useExtrinsicGuess, flags);

  // Transforms Rotation Vector to Matrix
  Rodrigues(rvec,R_Matrix);

  cout << "R = "<< endl << " "  << R_Matrix << endl << endl;
  cout << "t = "<< endl << " "  << tvec << endl << endl;

  // Rotation-Translation Matrix Definition
  _P_matrix.at<double>(0,0) = R_Matrix.at<double>(0,0);
  _P_matrix.at<double>(0,1) = R_Matrix.at<double>(0,1);
  _P_matrix.at<double>(0,2) = R_Matrix.at<double>(0,2);
  _P_matrix.at<double>(1,0) = R_Matrix.at<double>(1,0);
  _P_matrix.at<double>(1,1) = R_Matrix.at<double>(1,1);
  _P_matrix.at<double>(1,2) = R_Matrix.at<double>(1,2);
  _P_matrix.at<double>(2,0) = R_Matrix.at<double>(2,0);
  _P_matrix.at<double>(2,1) = R_Matrix.at<double>(2,1);
  _P_matrix.at<double>(0,3) = tvec.at<double>(0);
  _P_matrix.at<double>(1,3) = tvec.at<double>(1);
  _P_matrix.at<double>(2,3) = tvec.at<double>(2);

  cout << "P = "<< endl << " "  << _P_matrix << endl << endl;

  return correspondence;
}

vector<Point2f> PnPProblem::verify_points(ObjectMesh *objMesh)
{
  vector<Point2f> verified_points_2d;
  for( int i = 0; i < objMesh->getNumVertices(); i++)
  {
     Point3f tmp_point_3d = objMesh->getVertex(i).getPoint();
     Point2f tmp_computed_point_2d = this->backproject3DPoint(tmp_point_3d);
     verified_points_2d.push_back(tmp_computed_point_2d);

     /*cout << "Correspondence " << i << endl;;
     cout << "P 3D " << tmp_point_3d << endl;
     cout << "P 2D " << tmp_computed_point_2d << endl;*/
  }

  return verified_points_2d;
}

Point2f PnPProblem::backproject3DPoint(const Point3f &point)
{

  // Temporal 3d Point vector
  Mat tmp_3dpt = Mat::ones(4,1,cv::DataType<double>::type);
  tmp_3dpt.at<double>(0) = point.x;
  tmp_3dpt.at<double>(1) = point.y;
  tmp_3dpt.at<double>(2) = point.z;

  // Calculation of temporal [u v 1]'
  Mat tmp_uv = _A_matrix * _P_matrix * tmp_3dpt;

  // Normalization of [u v]
  Point2f tmp_2dpt;
  tmp_2dpt.x = tmp_uv.at<double>(0) / tmp_uv.at<double>(2);
  tmp_2dpt.y = tmp_uv.at<double>(1) / tmp_uv.at<double>(2);

  return tmp_2dpt;

}

bool PnPProblem::backproject2DPoint(const ObjectMesh *objMesh, const Point2f point2d, Point3f point3d) const
{
  if(1)
  {
      cout << "The point " << point2d << " is on the object surface" << endl;
      return true;
  }
  else
  {
      return false;
  }
}

bool PnPProblem::intersect3D_RayTriangle(Ray R, Triangle T, Point2f *I)
{
  const double EPSILON = 0.000001;

  Point3f edge1, edge2, tvec, pvec, qvec;
  float det, inv_det;


  //Find vectors for two edges sharing V0
  edge1 = SUB(T.getV1().getPoint(), T.getV0().getPoint());
  edge2 = SUB(T.getV2().getPoint(), T.getV0().getPoint());

  // Begin calculation determinant - also used to calculate U parameter
  pvec = CROSS(R.getP1(), edge2);

  // If determinant is near zero, ray lie in plane of triangle
  det = DOT(edge1, pvec);

  if(det < EPSILON) return false;

  return true;
}

void PnPProblem::check_intersect3D(Point2f ray_dir_2d, ObjectMesh *objMesh)
{
  Point3f camera_origin;
  Point3f ray_dir_3d;
  //Ray R(camera_origin,ray_dir_3d);




}


