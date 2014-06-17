/*
 * Utils.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#include <iostream>
#include <boost/lexical_cast.hpp>

#include "PnPProblem.h"
#include "ModelRegistration.h"
#include "Utils.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>


// For text
int fontFace = cv::FONT_ITALIC;
double fontScale = 0.75;
double thickness_font = 2;

// For circles
int lineType = 8;
int radius = 4;
double thickness_circ = -1;

// Draw a text with the question point
void drawQuestion(cv::Mat image, cv::Point3f point, cv::Scalar color)
{
  std::string x = boost::lexical_cast< std::string >((int)point.x);
  std::string y = boost::lexical_cast< std::string >((int)point.y);
  std::string z = boost::lexical_cast< std::string >((int)point.z);

  std::string text = " Where is point (" + x + ","  + y + "," + z + ") ?";
  cv::putText(image, text, cv::Point(25,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the number of entered points
void drawText(cv::Mat image, std::string text, cv::Scalar color)
{
  cv::putText(image, text, cv::Point(25,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the number of entered points
void drawText2(cv::Mat image, std::string text, cv::Scalar color)
{
  cv::putText(image, text, cv::Point(25,75), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the frame ratio
void drawFPS(cv::Mat image, double fps, cv::Scalar color)
{
  std::string fps_str = boost::lexical_cast< std::string >((int)fps);
  std::string text = fps_str + " FPS";
  cv::putText(image, text, cv::Point(500,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the frame ratio
void drawConfidence(cv::Mat image, double confidence, cv::Scalar color)
{
  std::string conf_str = boost::lexical_cast< std::string >((int)confidence);
  std::string text = conf_str + " %";
  cv::putText(image, text, cv::Point(500,75), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the number of entered points
void drawCounter(cv::Mat image, int n, int n_max, cv::Scalar color)
{
  std::string n_str = boost::lexical_cast< std::string >(n);
  std::string n_max_str = boost::lexical_cast< std::string >(n_max);
  std::string text = n_str + " of " + n_max_str + " points";
  cv::putText(image, text, cv::Point(500,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw the points and the coordinates
void drawPoints(cv::Mat image, std::vector<cv::Point2f> &list_points_2d, std::vector<cv::Point3f> &list_points_3d, cv::Scalar color)
{
  for (unsigned int i = 0; i < list_points_2d.size(); ++i)
  {
    cv::Point2f point_2d = list_points_2d[i];
    cv::Point3f point_3d = list_points_3d[i];

    // Draw Selected points
    cv::circle(image, point_2d, radius, color, -1, lineType );

    std::string idx = boost::lexical_cast< std::string >(i+1);
    std::string x = boost::lexical_cast< std::string >((int)point_3d.x);
    std::string y = boost::lexical_cast< std::string >((int)point_3d.y);
    std::string z = boost::lexical_cast< std::string >((int)point_3d.z);
    std::string text = "P" + idx + " (" + x + "," + y + "," + z +")";

    point_2d.x = point_2d.x + 10;
    point_2d.y = point_2d.y - 10;
    cv::putText(image, text, point_2d, fontFace, fontScale*0.5, color, thickness_font, 8);
  }
}

// Draw only the points
void draw2DPoints(cv::Mat image, std::vector<cv::Point2f> &list_points, cv::Scalar color)
{
  for( size_t i = 0; i < list_points.size(); i++)
  {
    cv::Point2f point_2d = list_points[i];

    // Draw Selected points
    cv::circle(image, point_2d, radius, color, -1, lineType );
  }
}

void drawArrow(cv::Mat image, cv::Point2i p, cv::Point2i q, cv::Scalar color, int arrowMagnitude, int thickness, int line_type, int shift)
{
  //Draw the principle line
  cv::line(image, p, q, color, thickness, line_type, shift);
  const double PI = 3.141592653;
  //compute the angle alpha
  double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
  //compute the coordinates of the first segment
  p.x = (int) ( q.x +  arrowMagnitude * cos(angle + PI/4));
  p.y = (int) ( q.y +  arrowMagnitude * sin(angle + PI/4));
  //Draw the first segment
  cv::line(image, p, q, color, thickness, line_type, shift);
  //compute the coordinates of the second segment
  p.x = (int) ( q.x +  arrowMagnitude * cos(angle - PI/4));
  p.y = (int) ( q.y +  arrowMagnitude * sin(angle - PI/4));
  //Draw the second segment
  cv::line(image, p, q, color, thickness, line_type, shift);
}

void draw3DCoordinateAxes(cv::Mat image, const std::vector<cv::Point2f> &list_points2d)
{
  cv::Scalar red(0, 0, 255);
  cv::Scalar green(0,255,0);
  cv::Scalar blue(255,0,0);
  cv::Scalar black(0,0,0);

  const double PI = 3.141592653;
  int length = 50;

  cv::Point2i origin = list_points2d[0];
  cv::Point2i pointX = list_points2d[1];
  cv::Point2i pointY = list_points2d[2];
  cv::Point2i pointZ = list_points2d[3];

  drawArrow(image, origin, pointX, red, 9, 2);
  drawArrow(image, origin, pointY, blue, 9, 2);
  drawArrow(image, origin, pointZ, green, 9, 2);
  cv::circle(image, origin, radius/2, black, -1, lineType );

}

// Draw the object mesh
void drawObjectMesh(cv::Mat image, const Mesh *mesh, PnPProblem *pnpProblem, cv::Scalar color)
{
  std::vector<std::vector<int> > list_triangles = mesh->getTrianglesList();
  for( size_t i = 0; i < list_triangles.size(); i++)
  {
    std::vector<int> tmp_triangle = list_triangles.at(i);

    cv::Point3f point_3d_0 = mesh->getVertex(tmp_triangle[0]);
    cv::Point3f point_3d_1 = mesh->getVertex(tmp_triangle[1]);
    cv::Point3f point_3d_2 = mesh->getVertex(tmp_triangle[2]);

    cv::Point2f point_2d_0 = pnpProblem->backproject3DPoint(point_3d_0);
    cv::Point2f point_2d_1 = pnpProblem->backproject3DPoint(point_3d_1);
    cv::Point2f point_2d_2 = pnpProblem->backproject3DPoint(point_3d_2);

    cv::line(image, point_2d_0, point_2d_1, color);
    cv::line(image, point_2d_1, point_2d_2, color);
    cv::line(image, point_2d_2, point_2d_0, color);
  }
}

bool equal_point(const cv::Point2f &p1, const cv::Point2f &p2)
{
  return ( (p1.x == p2.x) && (p1.y == p2.y) );
}

double get_translation_error(const cv::Mat &t_true, const cv::Mat &t)
{
  return cv::norm( t_true - t );
}

double get_rotation_error(const cv::Mat &R_true, const cv::Mat &R)
{
  cv::Mat error_vec, error_mat;
  error_mat = R_true * cv::Mat(R.inv()).mul(-1);
  cv::Rodrigues(error_mat, error_vec);

  return cv::norm(error_vec);
}

double get_variance(const std::vector<cv::Point3f> list_points3d)
{
  cv::Point3f p_mean, p_var;

  int n = list_points3d.size();
  for(unsigned int i = 0; i < n; ++i)
  {
    p_mean.x += list_points3d[i].x;
    p_mean.y += list_points3d[i].y;
    p_mean.z += list_points3d[i].z;
  }
  p_mean.x /= n;
  p_mean.y /= n;
  p_mean.z /= n;

  for(unsigned int i = 0; i < n; ++i)
  {
    p_var.x += list_points3d[i].x - p_mean.x ;
    p_var.y += list_points3d[i].y - p_mean.y ;
    p_var.z += list_points3d[i].z - p_mean.z ;
  }
  p_var.x /= n;
  p_var.y /= n;
  p_var.z /= n;

  // norm
  return sqrt( pow(p_var.x, 2) + pow(p_var.y, 2) + pow(p_var.z, 2) );

}

// w is equal to angular_velocity*time_between_frames
cv::Mat rot2quat(cv::Mat &rotationMatrix)
{
  cv::Mat q(4, 1, CV_64F);
  double qw, qx, qy, qz;

  double m00 = rotationMatrix.at<double>(0,0);
  double m01 = rotationMatrix.at<double>(0,1);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m21 = rotationMatrix.at<double>(2,1);
  double m22 = rotationMatrix.at<double>(2,2);

  double tr = m00 + m11 + m22;

  if (tr > 0)
  {
    double S = sqrt(tr+1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S;
    qz = (m10 - m01) / S;
  } else if ((m00 > m11)&(m00 > m22))
  {
    double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S;
    qz = (m02 + m20) / S;
  } else if (m11 > m22)
  {
    double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S;
    qy = 0.25 * S;
    qz = (m12 + m21) / S;
  }
  else
  {
    float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
    qw = (m10 - m01) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  }

  // normalisation
  double n = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw /= n;
  qx /= n;
  qy /= n;
  qz /= n;

  q.at<double>(0,0) = qw;
  q.at<double>(0,1) = qx;
  q.at<double>(0,2) = qy;
  q.at<double>(0,3) = qz;

  return q;
}

// w is equal to angular_velocity*time_between_frames
cv::Mat quat2rot(cv::Mat &q)
{
  cv::Mat rotationMatrix(3, 3, CV_64F);

  double w = q.at<double>(0);
  double x = q.at<double>(1);
  double y = q.at<double>(2);
  double z = q.at<double>(3);

  double n = w * w + x * x + y * y + z * z;
  double s = 0;
  if(n != 0) s = 2/n;

  double wx = s * w * x;
  double wy = s * w * y;
  double wz = s * w * z;
  double xx = s * x * x;
  double xy = s * x * y;
  double xz = s * x * z;
  double yy = s * y * y;
  double yz = s * y * z;
  double zz = s * z * z;

  rotationMatrix.at<double>(0,0) = 1 - ( yy + zz );
  rotationMatrix.at<double>(0,1) = xy - wz;
  rotationMatrix.at<double>(0,2) = xz + wy;
  rotationMatrix.at<double>(1,0) = xy + wz;
  rotationMatrix.at<double>(1,1) = 1 - ( xx + zz );
  rotationMatrix.at<double>(1,2) = yz - wx;
  rotationMatrix.at<double>(2,0) = xz - wy;
  rotationMatrix.at<double>(2,1) = yz + wx;
  rotationMatrix.at<double>(2,2) = 1 - ( xx + yy );

  return rotationMatrix;
}

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
  cv::Mat euler(3,1,CV_64F);

  double m00 = rotationMatrix.at<double>(0,0);
  double m01 = rotationMatrix.at<double>(0,1);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m21 = rotationMatrix.at<double>(2,1);
  double m22 = rotationMatrix.at<double>(2,2);

  double x, y, z;

  // Assuming the angles are in radians.
  if (m10 > 0.998) { // singularity at north pole
    x = 0;
    y = CV_PI/2;
    z = atan2(m02,m22);
  }
  else if (m10 < -0.998) { // singularity at south pole
    x = 0;
    y = -CV_PI/2;
    z = atan2(m02,m22);
  }
  else
  {
    x = atan2(-m12,m11);
    y = asin(m10);
    z = atan2(-m20,m00);
  }

  euler.at<double>(0) = x;
  euler.at<double>(1) = y;
  euler.at<double>(2) = z;

  return euler;
}

cv::Mat quat2euler(const cv::Mat & q)
{
  cv::Mat euler(3,1,CV_64F);

  double qw = q.at<double>(0);
  double qx = q.at<double>(1);
  double qy = q.at<double>(2);
  double qz = q.at<double>(3);

  double theta = atan2( 2*(qy*qz+qw*qx) , pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2) );
  double phi = asin( -2*(qx*qz-qw*qy) );
  double psi = atan2( 2*(qx*qy+qw*qz) , pow(qw,2)+pow(qx,2)-pow(qy,2)-pow(qz,2) );

  return euler;
}

cv::Mat euler2rot(const cv::Mat & euler)
{
  cv::Mat rotationMatrix(3,3,CV_64F);

  double x = euler.at<double>(0);
  double y = euler.at<double>(1);
  double z = euler.at<double>(2);

  // Assuming the angles are in radians.
  double ch = cos(z);
  double sh = sin(z);
  double ca = cos(y);
  double sa = sin(y);
  double cb = cos(x);
  double sb = sin(x);

  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  m00 = ch * ca;
  m01 = sh*sb - ch*sa*cb;
  m02 = ch*sa*sb + sh*cb;
  m10 = sa;
  m11 = ca*cb;
  m12 = -ca*sb;
  m20 = -sh*ca;
  m21 = sh*sa*cb + ch*sb;
  m22 = -sh*sa*sb + ch*cb;

  rotationMatrix.at<double>(0,0) = m00;
  rotationMatrix.at<double>(0,1) = m01;
  rotationMatrix.at<double>(0,2) = m02;
  rotationMatrix.at<double>(1,0) = m10;
  rotationMatrix.at<double>(1,1) = m11;
  rotationMatrix.at<double>(1,2) = m12;
  rotationMatrix.at<double>(2,0) = m20;
  rotationMatrix.at<double>(2,1) = m21;
  rotationMatrix.at<double>(2,2) = m22;

  return rotationMatrix;
}

