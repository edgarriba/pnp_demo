/*
 * Utils.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <cv.h>

using namespace cv;

// Draw a text with the question point
void drawQuestion(Mat image, Point3f point, Scalar color);

// Draw a text in the left of the image
void drawText(Mat image, string text, Scalar color);

// Draw a text with the number of registered points
void drawCounter(Mat image, int n, int n_max, Scalar color);

// Draw the points and the coordinates
void drawPoints(Mat image, ModelRegistration *modelReg, Scalar color);

void draw2DPoints(Mat image, vector<Point2f> &list_points, Scalar color);

void drawObjectMesh(Mat image, const ObjectMesh *objMesh, PnPProblem *pnpProblem, Scalar color);

// Draw an arrow into the image
void drawArrow(Mat image, Point2i p, Point2i q, Scalar color, int arrowMagnitude = 9, int thickness=1, int line_type=8, int shift=0);

// Draw the 3D coordinate axes
void draw3DCoordinateAxes(Mat image, Scalar color);

// Compute the 2D points with the estimated pose
vector<Point2f> verification_points(PnPProblem *p);

void computeKeyPoints(const Mat image);


#endif /* UTILS_H_ */
