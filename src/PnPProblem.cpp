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


/* Functions for Möller–Trumbore intersection algorithm
 * */
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

/* End functions for Möller–Trumbore intersection algorithm
 *  */

// Function to get the nearest 3D point given a set of them
Point3f get_nearest_3D_point(vector<Point3f> &points_list, Point3f origin)
{
  Point3f p1 = points_list[0];
  Point3f p2 = points_list[1];

  double d1 = sqrt( pow(p1.x-origin.x, 2) + pow(p1.y-origin.y, 2) + pow(p1.z-origin.z, 2) );
  double d2 = sqrt( pow(p2.x-origin.x, 2) + pow(p2.y-origin.y, 2) + pow(p2.z-origin.z, 2) );

  if(d1 < d2)
  {
      return p1;
  }
  else
  {
      return p2;
  }
}

PnPProblem::PnPProblem()  // default constructor
{
  _A_matrix = Mat::zeros(3,3,DataType<double>::type);
  _R_matrix = Mat::zeros(3,3,DataType<double>::type);
  _t_matrix = Mat::zeros(3,1,DataType<double>::type);
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
  _R_matrix = Mat::zeros(3,3,DataType<double>::type);
  _t_matrix = Mat::zeros(3,1,DataType<double>::type);
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
  _R_matrix = P._R_matrix;
  _t_matrix = P._t_matrix;
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

  // Build correspondences containers
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

  // Pose estimation
  bool correspondence = solvePnP(points_3d, points_2d, _A_matrix, distCoeffs, rvec, tvec, useExtrinsicGuess, flags);

  // Transforms Rotation Vector to Matrix
  Rodrigues(rvec,_R_matrix);
  _t_matrix = tvec;

  cout << "R = "<< endl << " "  << _R_matrix << endl << endl;
  cout << "t = "<< endl << " "  << _t_matrix << endl << endl;

  // Rotation-Translation Matrix Definition
  _P_matrix.at<double>(0,0) = _R_matrix.at<double>(0,0);
  _P_matrix.at<double>(0,1) = _R_matrix.at<double>(0,1);
  _P_matrix.at<double>(0,2) = _R_matrix.at<double>(0,2);
  _P_matrix.at<double>(1,0) = _R_matrix.at<double>(1,0);
  _P_matrix.at<double>(1,1) = _R_matrix.at<double>(1,1);
  _P_matrix.at<double>(1,2) = _R_matrix.at<double>(1,2);
  _P_matrix.at<double>(2,0) = _R_matrix.at<double>(2,0);
  _P_matrix.at<double>(2,1) = _R_matrix.at<double>(2,1);
  _P_matrix.at<double>(0,3) = _t_matrix.at<double>(0);
  _P_matrix.at<double>(1,3) = _t_matrix.at<double>(1);
  _P_matrix.at<double>(2,3) = _t_matrix.at<double>(2);

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

bool PnPProblem::backproject2DPoint(const ObjectMesh *objMesh, const Point2f &point2d, Point3f &point3d)
{

  // Triangles list of the object mesh
  vector< vector <int> > triangles_list = objMesh->getTrianglesList();

  double lambda = 8;
  double u = point2d.x;
  double v = point2d.y;

  // Point in vector form
  Mat tmp_2dpt = Mat::ones(3,1,cv::DataType<double>::type); // 3x1
  tmp_2dpt.at<double>(0) = u * lambda;
  tmp_2dpt.at<double>(1) = v * lambda;
  tmp_2dpt.at<double>(2) = lambda;

  // Point in camera coordinates
  Mat X_c = _A_matrix.inv() * tmp_2dpt ; // 3x1

  // Point in world coordinates
  Mat X_w = _R_matrix.inv() * ( X_c - _t_matrix ); // 3x1

  // Center of projection
  Mat C_op = Mat(_R_matrix.inv()).mul(-1) * _t_matrix; // 3x1

  // Ray direction vector
  Mat ray = X_w - C_op; // 3x1
  ray = ray / norm(ray); // 3x1

  // Set up Ray
  Ray R((Point3f)C_op, (Point3f)ray);

  // A vector to store the intersections found
  vector<Point3f> intersections_list;

  // Loop for all the triangles and check the intersection
  for (unsigned int i = 0; i < triangles_list.size(); i++)
  {
      Vertex V0 = objMesh->getVertex(triangles_list[i][0]);
      Vertex V1 = objMesh->getVertex(triangles_list[i][1]);
      Vertex V2 = objMesh->getVertex(triangles_list[i][2]);

      Triangle T(i, V0, V1, V2);

      double out;
      if(this->intersect_MollerTrumbore(R, T, &out))
      {
        Point3f tmp_pt = R.getP0() + out*R.getP1(); // P = O + t*D
        intersections_list.push_back(tmp_pt);
      }
  }

  // If there are intersection, find the nearest one
  if (!intersections_list.empty())
  {
      point3d = get_nearest_3D_point(intersections_list, R.getP0());
      return true;
  }
  else
  {
      return false;
  }
}

// Möller–Trumbore intersection algorithm
bool PnPProblem::intersect_MollerTrumbore(Ray &Ray, Triangle &Triangle, double *out)
{
  const double EPSILON = 0.000001;

  Point3f e1, e2;
  Point3f P, Q, T;
  double det, inv_det, u, v;
  double t;

  Point3f V1 = Triangle.getV0().getPoint();  // Triangle vertices
  Point3f V2 = Triangle.getV1().getPoint();
  Point3f V3 = Triangle.getV2().getPoint();

  Point3f O = Ray.getP0(); // Ray origin
  Point3f D = Ray.getP1(); // Ray direction


  //Find vectors for two edges sharing V1
  e1 = SUB(V2, V1);
  e2 = SUB(V3, V1);

  // Begin calculation determinant - also used to calculate U parameter
  P = CROSS(D, e2);

  // If determinant is near zero, ray lie in plane of triangle
  det = DOT(e1, P);

  //NOT CULLING
  if(det > -EPSILON && det < EPSILON) return false;
  inv_det = 1.f / det;

  //calculate distance from V1 to ray origin
  T = SUB(O, V1);

  //Calculate u parameter and test bound
  u = DOT(T, P) * inv_det;

  //The intersection lies outside of the triangle
  if(u < 0.f || u > 1.f) return false;

  //Prepare to test v parameter
  Q = CROSS(T, e1);

  //Calculate V parameter and test bound
  v = DOT(D, Q) * inv_det;

  //The intersection lies outside of the triangle
  if(v < 0.f || u + v  > 1.f) return false;

  t = DOT(e2, Q) * inv_det;

  if(t > EPSILON) { //ray intersection
    *out = t;
    return true;
  }

  // No hit, no win
  return false;
}



