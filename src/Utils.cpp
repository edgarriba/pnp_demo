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

#include "opencv2/nonfree/features2d.hpp"

using namespace std;

// For text
int fontFace = FONT_ITALIC;
double fontScale = 0.75;
double thickness_font = 2;

// For circles
int lineType = 8;
int radius = 4;
double thickness_circ = -1;

// Draw a text with the question point
void drawQuestion(Mat image, Point3f point, Scalar color)
{
  string x = boost::lexical_cast< string >((int)point.x);
  string y = boost::lexical_cast< string >((int)point.y);
  string z = boost::lexical_cast< string >((int)point.z);

  string text = " Where is point (" + x + ","  + y + "," + z + ") ?";
  putText(image, text, Point(25,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the number of entered points
void drawText(Mat image, string text, Scalar color)
{
  putText(image, text, Point(25,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw a text with the number of entered points
void drawCounter(Mat image, int n, int n_max, Scalar color)
{
  string n_str = boost::lexical_cast< string >(n);
  string n_max_str = boost::lexical_cast< string >(n_max);
  string text = n_str + " of " + n_max_str + " points";
  putText(image, text, Point(500,50), fontFace, fontScale, color, thickness_font, 8);
}

// Draw the points and the coordinates
void drawPoints(Mat image, ModelRegistration *modelReg, Scalar color)
{
  for ( int i = 0; i < modelReg->getNum(); i++)
  {
    pair<int,pair<Point2f,Point3f> > correspondence = modelReg->getCorrespondence(i);
    Point2f point_2d = correspondence.second.first;
    Point3f point_3d = correspondence.second.second;

    // Draw Selected points
    circle(image, point_2d, radius, color, -1, lineType );

    string idx = boost::lexical_cast< string >(correspondence.first);
    string x = boost::lexical_cast< string >((int)point_3d.x);
    string y = boost::lexical_cast< string >((int)point_3d.y);
    string z = boost::lexical_cast< string >((int)point_3d.z);
    string text = "P" + idx + " (" + x + "," + y + "," + z +")";

    point_2d.x = point_2d.x + 10;
    point_2d.y = point_2d.y - 10;
    putText(image, text, point_2d, fontFace, fontScale*0.5, color, thickness_font, 8);
  }
}

void draw2DPoints(Mat image, vector<Point2f> &list_points, Scalar color)
{
  for( size_t i = 0; i < list_points.size(); i++)
  {
    Point2f point_2d = list_points.at(i);

    // Draw Selected points
    circle(image, point_2d, radius, color, -1, lineType );

    string idx = boost::lexical_cast< string >(i+1);
    string text = "P" + idx;
    putText(image, text, point_2d, fontFace, fontScale*0.5, color, thickness_font, 8);

  }
}

void drawArrow(Mat image, Point2i p, Point2i q, Scalar color, int arrowMagnitude, int thickness, int line_type, int shift)
{
  //Draw the principle line
  line(image, p, q, color, thickness, line_type, shift);
  const double PI = 3.141592653;
  //compute the angle alpha
  double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
  //compute the coordinates of the first segment
  p.x = (int) ( q.x +  arrowMagnitude * cos(angle + PI/4));
  p.y = (int) ( q.y +  arrowMagnitude * sin(angle + PI/4));
  //Draw the first segment
  line(image, p, q, color, thickness, line_type, shift);
  //compute the coordinates of the second segment
  p.x = (int) ( q.x +  arrowMagnitude * cos(angle - PI/4));
  p.y = (int) ( q.y +  arrowMagnitude * sin(angle - PI/4));
  //Draw the second segment
  line(image, p, q, color, thickness, line_type, shift);
}

void draw3DCoordinateAxes(Mat image, Scalar color)
{
  Scalar red(0, 0, 255);
  Scalar green(0,255,0);
  Scalar blue(255,0,0);
  Scalar black(0,0,0);

  const double PI = 3.141592653;
  int length = 50;

  Point2i origin(625,425);
  Point2i pointX(origin.x,origin.y-length);
  Point2i pointY(origin.x+length,origin.y);
  Point2i pointZ;
  pointZ.x = (int) ( origin.x - length * cos(30*PI/360 + PI/4));
  pointZ.y = (int) ( origin.y - length * cos(30*PI/360 + PI/4));

  drawArrow(image, origin, pointX, red, 9, 2);
  drawArrow(image, origin, pointY, blue, 9, 2);
  drawArrow(image, origin, pointZ, green, 9, 2);
  circle(image, origin, radius/2, black, -1, lineType );

}

void drawObjectMesh(Mat image, const ObjectMesh *objMesh, PnPProblem *pnpProblem, Scalar color)
{
  vector<vector<int> > list_triangles = objMesh->getTrianglesList();
  for( size_t i = 0; i < list_triangles.size(); i++)
  {
      vector<int> tmp_triangle = list_triangles.at(i);

      Point3f point_3d_0 = objMesh->getVertex(tmp_triangle[0]).getPoint();
      Point3f point_3d_1 = objMesh->getVertex(tmp_triangle[1]).getPoint();
      Point3f point_3d_2 = objMesh->getVertex(tmp_triangle[2]).getPoint();

      Point2f point_2d_0 = pnpProblem->backproject3DPoint(point_3d_0);
      Point2f point_2d_1 = pnpProblem->backproject3DPoint(point_3d_1);
      Point2f point_2d_2 = pnpProblem->backproject3DPoint(point_3d_2);

      line(image, point_2d_0, point_2d_1, color);
      line(image, point_2d_1, point_2d_2, color);
      line(image, point_2d_2, point_2d_0, color);
  }
}


// Compute key points and draw it
void computeKeyPoints(const Mat image)
{
  string img_path = "../Images/resized_IMG_3873.JPG";
  Mat image_2 = imread(img_path);

  string window_name = "Key Points";
  namedWindow(window_name,CV_WINDOW_KEEPRATIO);

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;
  SurfFeatureDetector detector( minHessian );
  SiftFeatureDetector detect;

  std::vector<KeyPoint> keypoints_1, keypoints_2;
  detect.detect( image, keypoints_1 );
  detect.detect( image_2, keypoints_2 );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;
  SiftDescriptorExtractor extract;

  Mat descriptors_1, descriptors_2;
  extract.compute( image, keypoints_1, descriptors_1 );
  extract.compute( image_2, keypoints_2, descriptors_2 );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance <= max(2*min_dist, 0.02) )
    { good_matches.push_back( matches[i]); }
  }

  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches( image, keypoints_1, image_2, keypoints_2,
               good_matches, img_matches, Scalar(250,0,0), Scalar(0,0,255),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Draw keypoints
  Mat img_keypoints_1,img_keypoints_2;
  drawKeypoints( image, keypoints_1, img_keypoints_1, Scalar(250,0,0), DrawMatchesFlags::DEFAULT );
  drawKeypoints( image_2, keypoints_2, img_keypoints_2, Scalar(250,0,0), DrawMatchesFlags::DEFAULT );

  //-- Show detected (drawn) keypoints
  imshow(window_name,img_matches);

  waitKey(0);

  // close the window
  destroyWindow(window_name);

}

