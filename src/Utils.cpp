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
#include <opencv2/calib3d.hpp>
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

// Compute the ORB keypoints and descriptors of a given image
void computeKeyPoints(const cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{

  cv::Mat image_gray;
  cv::cvtColor( image, image_gray, CV_RGB2GRAY );

  /* ORB parameters */
  int nfeatures = 5000;
  float scaleFactor = 1.2f;
  int nlevels = 8;
  int edgeThreshold = 31;
  int firstLevel = 0;
  int WTA_K = 2;
  int scoreType = cv::ORB::HARRIS_SCORE;
  int patchSize = 31;

  cv::ORB orb(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);

  //-- Step 1: Calculate keypoints
  orb.detect( image_gray, keypoints );

  //-- Step 2: Calculate descriptors (feature vectors)
  orb.compute( image_gray, keypoints, descriptors );

}

// Clear matches for which NN ratio is > than threshold
// return the number of removed points
// (corresponding entries being cleared,
// i.e. size will be 0)
int ratioTest(std::vector<std::vector<cv::DMatch> > &matches, double ratio)
{
  int removed=0;
  // for all matches
  for ( std::vector<std::vector<cv::DMatch> >::iterator
        matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
  {
    // if 2 NN has been identified
    if (matchIterator->size() > 1)
    {
       // check distance ratio
      if ((*matchIterator)[0].distance < ratio * (*matchIterator)[1].distance )
      // if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratio)
       {
          matchIterator->clear(); // remove match
          removed++;
       }
    }
    else
    { // does not have 2 neighbours
       matchIterator->clear(); // remove match
       removed++;
    }
  }
  return removed;
}

// Insert symmetrical matches in symMatches vector
void symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                   const std::vector<std::vector<cv::DMatch> >& matches2,
                   std::vector<cv::DMatch>& symMatches )
{
  // for all matches image 1 -> image 2
  for (std::vector<std::vector<cv::DMatch> >::const_iterator
      matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1)
  {
     // ignore deleted matches
     if (matchIterator1->size() < 2)
         continue;
     // for all matches image 2 -> image 1
     for (std::vector<std::vector<cv::DMatch> >::const_iterator
         matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2)
     {
         // ignore deleted matches
         if (matchIterator2->size() < 2)
            continue;
         // Match symmetry test
         if ((*matchIterator1)[0].queryIdx ==
             (*matchIterator2)[0].trainIdx &&
             (*matchIterator2)[0].queryIdx ==
             (*matchIterator1)[0].trainIdx) {
             // add symmetrical match
               symMatches.push_back(
                 cv::DMatch((*matchIterator1)[0].queryIdx,
                           (*matchIterator1)[0].trainIdx,
                           (*matchIterator1)[0].distance));
               break; // next match in image 1 -> image 2
         }
     }
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

cv::Point3f get_variance(const std::vector<cv::Point3f> list_points3d)
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

  return p_var;

}

double get_ratio(const cv::Point3f p1, const cv::Point3f p2)
{
  double x = p1.x / p2.x;
  double y = p1.y / p2.y;
  double z = p1.z / p2.z;

  return sqrt( x*x + y*y + z*z );
}



