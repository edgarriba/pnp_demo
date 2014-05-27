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

#include <opencv2/imgproc.hpp>
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
    cv::Point2f point_2d = list_points.at(i);

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

void draw3DCoordinateAxes(cv::Mat image, cv::Scalar color)
{
  cv::Scalar red(0, 0, 255);
  cv::Scalar green(0,255,0);
  cv::Scalar blue(255,0,0);
  cv::Scalar black(0,0,0);

  const double PI = 3.141592653;
  int length = 50;

  cv::Point2i origin(625,425);
  cv::Point2i pointX(origin.x,origin.y-length);
  cv::Point2i pointY(origin.x+length,origin.y);
  cv::Point2i pointZ;
  pointZ.x = (int) ( origin.x - length * cos(30*PI/360 + PI/4));
  pointZ.y = (int) ( origin.y - length * cos(30*PI/360 + PI/4));

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

// Compute the ORB keypoints and descriptors of a given image
void computeKeyPoints(const cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{

  cv::Mat image_gray;
  cv::cvtColor( image, image_gray, CV_RGB2GRAY );

  /* ORB parameters */
  int nfeatures = 1000;
  float scaleFactor = 1.2f;
  int nlevels = 8;
  int edgeThreshold = 31;
  int firstLevel = 0;
  int WTA_K = 4;
  int scoreType = cv::ORB::HARRIS_SCORE;
  int patchSize = 31;

  cv::ORB orb(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);

  //-- Step 1: Calculate keypoints
  orb.detect( image_gray, keypoints );

  //-- Step 2: Calculate descriptors (feature vectors)
  orb.compute( image_gray, keypoints, descriptors );

}

